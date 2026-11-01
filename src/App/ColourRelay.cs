using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace Kinect2VMC.App
{
    /// <summary>
    /// Serves the Kinect's colour stream to the hand tracker over a loopback socket.
    ///
    /// Why the frames come from here
    /// ----------------------------
    /// The hand tracker needs camera frames, and the laptop webcam is both unreliable and often
    /// already in use by Warudo's own face capture. The Kinect's colour camera is the obvious
    /// alternative, but only ONE process can hold the sensor, and this program already does.
    ///
    /// Reading it a second time from Python is not viable either: the usual library for that
    /// (pykinect2) does not even import on a current Python. Its first struct assertion expects a
    /// 72-byte tagSTATSTG and gets 80, because the structure layouts moved on since 2016. Patching
    /// a third-party ctypes binding is not something to build a feature on.
    ///
    /// So this program, which already owns the sensor and already receives colour frames, forwards
    /// them. The hand tracker then just reads a socket instead of opening a device.
    ///
    /// Wire format, deliberately trivial:
    ///     "K2VC"          4-byte magic, once at connect
    ///     then per frame: width (int32 LE), height (int32 LE), then width*height*4 bytes BGRA
    /// </summary>
    public sealed class ColourRelay : IDisposable
    {
        private TcpListener _listener;
        private Thread _acceptThread;
        private volatile bool _running;

        private TcpClient _client;
        private NetworkStream _stream;
        private readonly object _gate = new object();

        // Frames wait here for the sender thread.
        //
        // This queue is what keeps the Kinect safe. Send() is called from the sensor's own frame
        // callback, and a BLOCKING socket write there stalls that callback -- which froze the whole
        // program and dropped the sensor, exactly as the user reported. So Send() only enqueues and
        // returns; a separate thread does the writing.
        //
        // Bounded, and the OLDEST frame is dropped when full. A tracker that cannot keep up should
        // lose frames, not build an ever-growing backlog of stale ones.
        private readonly System.Collections.Generic.Queue<byte[]> _pending =
            new System.Collections.Generic.Queue<byte[]>();
        private const int MaxPendingFrames = 2;

        private Thread _sendThread;
        private volatile bool _senderRunning;

        /// <summary>Port the hand tracker should connect to. 0 until started.</summary>
        public int Port { get; private set; }

        /// <summary>
        /// Longest side of the frame actually sent. 0 means send the sensor's own size.
        ///
        /// The Kinect's colour stream is 1920x1080 BGRA, which is 8.3 MB a frame and about 250 MB
        /// a second at 30 Hz. Sending that to a Python process that also runs MediaPipe does not
        /// work: the consumer cannot keep up, the socket write blocks, and the relay looks frozen
        /// however the sending is threaded.
        ///
        /// Hand tracking does not need the resolution. The image is reduced here, on the sensor
        /// side, so the network carries a fraction of the bytes.
        /// </summary>
        public int MaxDimension = 640;

        /// <summary>Frames per second actually forwarded. 0 means no limit.</summary>
        public double MaxFps = 20.0;

        private long _lastSendTicks;
        private long _framesSent;

        /// <summary>Frames forwarded since the last client connected.</summary>
        public long FramesSent;

        public bool HasClient
        {
            get
            {
                lock (_gate) { return _client != null && _client.Connected; }
            }
        }

        /// <summary>Starts listening on a loopback port. Returns false if that is not possible.</summary>
        public bool Start(int port)
        {
            if (_running) { return true; }

            try
            {
                // Loopback only. This stream is the user's camera and must never leave the machine.
                _listener = new TcpListener(IPAddress.Loopback, port);
                _listener.Start();
                Port = ((IPEndPoint)_listener.LocalEndpoint).Port;
                _running = true;

                _acceptThread = new Thread(AcceptLoop);
                _acceptThread.IsBackground = true;
                _acceptThread.Start();

                _senderRunning = true;
                _sendThread = new Thread(SendLoop);
                _sendThread.IsBackground = true;
                _sendThread.Start();
                return true;
            }
            catch
            {
                Stop();
                return false;
            }
        }

        private void AcceptLoop()
        {
            while (_running)
            {
                try
                {
                    TcpClient client = _listener.AcceptTcpClient();

                    lock (_gate)
                    {
                        if (_client != null)
                        {
                            // One consumer only; a second connection would interleave frames.
                            try { _client.Close(); } catch { }
                        }
                        _client = client;
                        _stream = client.GetStream();

                        // Even on the sender thread a write must not hang forever: a reader that
                        // accepts a connection and then stops reading would otherwise pin a thread
                        // and keep the queue permanently full.
                        _client.SendTimeout = 1000;

                        // A new reader wants the CURRENT frame, not whatever the last one left
                        // queued.
                        _pending.Clear();
                        FramesSent = 0;
                    }

                    try
                    {
                        _stream.Write(new byte[] { (byte)'K', (byte)'2', (byte)'V', (byte)'C' }, 0, 4);
                        _stream.Flush();
                    }
                    catch
                    {
                        DropClient();
                    }
                }
                catch
                {
                    // Listener closed, or accept failed. Either way the loop is ending or retrying.
                    if (!_running) { return; }
                    Thread.Sleep(200);
                }
            }
        }

        /// <summary>
        /// Queues one BGRA frame for the sender thread. NEVER blocks.
        ///
        /// Called from the Kinect frame callback, so the only thing it may do is hand the buffer
        /// over and return. Writing to the socket here is what froze the program: a client that
        /// stopped reading made the write block, which stalled the sensor callback and took the
        /// whole pipeline down with it.
        /// </summary>
        public void Send(byte[] bgra, int width, int height)
        {
            if (!_running || bgra == null) { return; }

            // Rate limit FIRST, before any per-frame work. A consumer that can only manage a
            // fraction of 30 Hz should not be paying for the frames it will never receive.
            if (MaxFps > 0.0)
            {
                long now = System.Diagnostics.Stopwatch.GetTimestamp();
                double minTicks = System.Diagnostics.Stopwatch.Frequency / MaxFps;
                if (now - _lastSendTicks < minTicks) { return; }
                _lastSendTicks = now;
            }

            // Downscale on this side, where the full-resolution data already is. 8.3 MB per frame
            // over loopback to a busy Python process was the actual cause of the freeze.
            int targetW = width;
            int targetH = height;
            if (MaxDimension > 0 && Math.Max(width, height) > MaxDimension)
            {
                double scale = (double)MaxDimension / Math.Max(width, height);
                targetW = Math.Max(1, (int)(width * scale));
                targetH = Math.Max(1, (int)(height * scale));
            }

            int size = targetW * targetH * 4;
            byte[] copy = new byte[8 + size];

            try
            {
                if (targetW == width && targetH == height)
                {
                    Buffer.BlockCopy(bgra, 0, copy, 8, size);
                }
                else
                {
                    ResizeBgraNearest(bgra, width, height, copy, 8, targetW, targetH);
                }
            }
            catch
            {
                return;
            }

            Buffer.BlockCopy(BitConverter.GetBytes(targetW), 0, copy, 0, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(targetH), 0, copy, 4, 4);

            lock (_gate)
            {
                if (_client == null) { return; }

                // Drop the oldest rather than the newest: a lagging reader is better served by the
                // current frame than by a backlog.
                while (_pending.Count >= MaxPendingFrames) { _pending.Dequeue(); }
                _pending.Enqueue(copy);
                Monitor.Pulse(_gate);
            }
        }

        /// <summary>
        /// Nearest-neighbour reduction of a BGRA image.
        ///
        /// Nearest neighbour on purpose: this runs once per forwarded frame inside the tracking
        /// process, and for hand detection the difference from a smoother filter is not worth the
        /// cost. Sampling one pixel per output pixel keeps it a cheap memory walk.
        /// </summary>
        private static void ResizeBgraNearest(byte[] source, int sourceW, int sourceH,
            byte[] destination, int destinationOffset, int targetW, int targetH)
        {
            int xRatio = (sourceW << 16) / targetW;
            int yRatio = (sourceH << 16) / targetH;
            int dst = destinationOffset;

            for (int y = 0; y < targetH; y++)
            {
                int sourceRow = ((y * yRatio) >> 16) * sourceW * 4;
                for (int x = 0; x < targetW; x++)
                {
                    int sourceIndex = sourceRow + (((x * xRatio) >> 16) * 4);
                    destination[dst] = source[sourceIndex];
                    destination[dst + 1] = source[sourceIndex + 1];
                    destination[dst + 2] = source[sourceIndex + 2];
                    destination[dst + 3] = source[sourceIndex + 3];
                    dst += 4;
                }
            }
        }

        private void SendLoop()
        {
            while (_senderRunning)
            {
                byte[] payload = null;

                lock (_gate)
                {
                    if (_pending.Count == 0)
                    {
                        Monitor.Wait(_gate, 100);
                    }
                    if (_pending.Count > 0) { payload = _pending.Dequeue(); }
                }

                if (payload == null) { continue; }

                NetworkStream stream;
                lock (_gate) { stream = _stream; }
                if (stream == null) { continue; }

                try
                {
                    stream.Write(payload, 0, payload.Length);
                    stream.Flush();
                    Interlocked.Increment(ref _framesSent);
                    FramesSent = _framesSent;
                }
                catch
                {
                    // A slow or dead reader is dropped here, on the sender thread, where it cannot
                    // reach the sensor callback.
                    DropClient();
                }
            }
        }

        private void DropClient()
        {
            lock (_gate)
            {
                try { if (_stream != null) { _stream.Close(); } } catch { }
                try { if (_client != null) { _client.Close(); } } catch { }
                _stream = null;
                _client = null;

                // Holding frames for a reader that has gone away only wastes memory and would be
                // stale by the time anything reconnects.
                _pending.Clear();
            }
        }

        public void Stop()
        {
            _running = false;
            _senderRunning = false;

            lock (_gate)
            {
                _pending.Clear();
                Monitor.PulseAll(_gate);
            }

            try { if (_listener != null) { _listener.Stop(); } } catch { }
            _listener = null;

            DropClient();
            Port = 0;
        }

        public void Dispose()
        {
            Stop();
        }
    }
}
