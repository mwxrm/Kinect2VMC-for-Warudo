using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Kinect2VMC.Core;

namespace Kinect2VMC.App
{
    /// <summary>
    /// Receives the measured palm orientation from the hand tracker.
    ///
    /// Why the two programs have to cooperate here
    /// -------------------------------------------
    /// A hand rotation needs two things that live in different places. The ARM comes from Kinect --
    /// only this program can read the sensor -- and the PALM comes from the camera, because Kinect's
    /// own hand bone is about 9 cm long and its orientation is derived from that very short, noisy
    /// segment, so it cannot say which way the palm faces.
    ///
    /// With the two sent separately they fight over one bone, which is exactly the twitching that
    /// was seen. So the tracker sends its measurement HERE, and this program composes it with
    /// Kinect's arm into one coherent rotation before anything reaches the avatar.
    ///
    /// Wire format, matching HandFrameSender in hand_tracker.py:
    ///     "K2VH"                      magic
    ///     uint32 little-endian        frame counter
    ///     'L' or 'R'                  which hand
    ///     '1' or '0'                  whether the hand was actually seen
    ///     two padding bytes
    ///     float x4 little-endian      palm orientation, in character space
    /// </summary>
    public sealed class HandFrameReceiver : IDisposable
    {
        public const int DefaultPort = 39542;

        /// <summary>Packets older than this are treated as absent.</summary>
        ///
        /// A stale orientation is worse than none: if the tracker stops, holding the last value
        /// would freeze the hand in a pose that no longer matches the arm, and the fingers would
        /// keep moving under a hand that is not there. Expiring it lets the hand fall back to
        /// following the forearm, which is at least consistent.
        public double TimeoutSeconds = 0.5;

        private UdpClient _client;
        private Thread _thread;
        private volatile bool _running;

        private readonly object _gate = new object();
        private readonly Quat[] _orientation = { Quat.Identity, Quat.Identity };
        private readonly bool[] _valid = { false, false };
        private readonly double[] _receivedAt = { 0.0, 0.0 };

        private long _packets;
        private long _lastCounter = -1;

        public int Port { get; private set; }
        public string LastError { get; private set; }

        /// <summary>Packets accepted since the last reset, for the diagnostics report.</summary>
        public long PacketsReceived { get { return Interlocked.Read(ref _packets); } }

        /// <summary>True when the tracker is currently sending usable orientations.</summary>
        public bool IsFresh(string side)
        {
            lock (_gate)
            {
                return _valid[SideIndex(side)] && Age(side) <= TimeoutSeconds;
            }
        }

        private double Age(string side)
        {
            int index = SideIndex(side);
            return Now() - _receivedAt[index];
        }

        /// <summary>The palm orientation for a side, or null when there is no fresh one.</summary>
        public Quat? GetOrientation(string side)
        {
            lock (_gate)
            {
                int index = SideIndex(side);
                if (!_valid[index]) { return null; }
                if (Now() - _receivedAt[index] > TimeoutSeconds) { return null; }
                return _orientation[index];
            }
        }

        private static int SideIndex(string side)
        {
            return side == "Left" ? 0 : 1;
        }

        private static double Now()
        {
            return (double)Environment.TickCount / 1000.0;
        }

        public bool Start(int port)
        {
            if (_running) { return true; }

            try
            {
                _client = new UdpClient(new IPEndPoint(IPAddress.Loopback, port));
                Port = ((IPEndPoint)_client.Client.LocalEndPoint).Port;
                _running = true;

                _thread = new Thread(ReceiveLoop);
                _thread.IsBackground = true;
                _thread.Start();
                return true;
            }
            catch (Exception ex)
            {
                LastError = "无法监听手掌朝向端口 " + port + ": " + ex.Message;
                Stop();
                return false;
            }
        }

        private void ReceiveLoop()
        {
            var remote = new IPEndPoint(IPAddress.Any, 0);

            while (_running)
            {
                try
                {
                    byte[] packet = _client.Receive(ref remote);
                    Parse(packet);
                }
                catch
                {
                    // Socket closed, or a malformed packet. Neither is worth stopping for.
                    if (!_running) { return; }
                }
            }
        }

        /// <summary>Parses one packet. Exposed so the self-test can drive it without a socket.</summary>
        public bool Parse(byte[] packet)
        {
            if (packet == null || packet.Length < 16) { return false; }
            if (packet[0] != (byte)'K' || packet[1] != (byte)'2'
                || packet[2] != (byte)'V' || packet[3] != (byte)'H') { return false; }

            uint counter = BitConverter.ToUInt32(packet, 4);

            // Older than the last one means a reordered or duplicated datagram; ignoring it keeps
            // the orientation from jumping backwards.
            if (_lastCounter >= 0 && counter == unchecked((uint)_lastCounter)) { return false; }
            _lastCounter = counter;

            char sideChar = (char)packet[8];
            bool seen = packet[9] == (byte)'1';
            if (sideChar != 'L' && sideChar != 'R') { return false; }

            float x = BitConverter.ToSingle(packet, 12);
            float y = BitConverter.ToSingle(packet, 16);
            float z = BitConverter.ToSingle(packet, 20);
            float w = BitConverter.ToSingle(packet, 24);

            var q = new Quat(x, y, z, w);
            if (!q.IsFinite()) { return false; }

            // A quaternion that is not unit-length is a rotation that is quietly wrong, and the
            // sender is the only place that can be checked cheaply.
            double length = Math.Sqrt((double)x * x + (double)y * y + (double)z * z + (double)w * w);
            if (length < 0.5 || length > 1.5) { return false; }

            int index = sideChar == 'L' ? 0 : 1;

            lock (_gate)
            {
                _valid[index] = seen;
                if (seen)
                {
                    _orientation[index] = new Quat(
                        (float)(x / length), (float)(y / length),
                        (float)(z / length), (float)(w / length));
                }
                _receivedAt[index] = Now();
            }

            Interlocked.Increment(ref _packets);
            return true;
        }

        public void Stop()
        {
            _running = false;

            try { if (_client != null) { _client.Close(); } } catch { }
            _client = null;
            Port = 0;

            lock (_gate)
            {
                _valid[0] = false;
                _valid[1] = false;
            }
        }

        public void Dispose()
        {
            Stop();
        }
    }
}
