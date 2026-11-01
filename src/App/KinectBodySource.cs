using System.Threading;
using System;
using System.Collections.Generic;
using Microsoft.Kinect;
using Kinect2VMC.Core;

namespace Kinect2VMC.App
{
    /// <summary>
    /// Reads Kinect v2 body frames and converts them into <see cref="SkeletonFrameData"/>,
    /// already expressed in the solver's character space.
    ///
    /// The sensor is opened lazily and re-opened automatically when it drops out. That is not
    /// defensive padding: this Kinect's 12 V supply is a hand-modified one, so the device
    /// dropping off the bus mid-session is a realistic failure mode and should not require
    /// restarting the application.
    /// </summary>
    public sealed class KinectBodySource : IBodySource
    {
        /// <summary>Name of every Kinect joint, indexed by the solver's own enum.</summary>
        private static readonly JointType[] JointByIndex = BuildJointMap();

        private static readonly Dictionary<JointType, int> IndexByJoint = BuildIndexMap();

        private KinectSensor _sensor;
        private byte[] _colourBuffer;
        private Thread _colourThread;
        private volatile bool _colourRunning;

        /// <summary>Request the colour stream as well as the body stream.</summary>
        public bool RelayColour;

        /// <summary>Called with each colour frame as BGRA. Null when not relaying.</summary>
        public Action<byte[], int, int> ColourSink;
        private MultiSourceFrameReader _reader;
        private Body[] _bodies;

        private readonly SkeletonFrameData _frame = new SkeletonFrameData();
        private readonly object _gate = new object();
        private readonly System.Diagnostics.Stopwatch _clock = System.Diagnostics.Stopwatch.StartNew();

        private DateTime _lastOpenAttempt = DateTime.MinValue;
        private ulong _preferredTrackingId;
        private DateTime _preferredIdExpiry = DateTime.MinValue;

        /// <summary>
        /// True when a sensor object exists and reports itself available.
        ///
        /// Read straight from the sensor rather than cached: a device whose 12 V supply is a
        /// hand-modified one can drop off the bus at any moment, and a cached flag would keep
        /// claiming the device is there until the next frame happened to arrive.
        /// </summary>
        public bool IsSensorAvailable { get { return _sensor != null && _sensor.IsAvailable; } }

        /// <summary>True when the current frame carries a tracked body.</summary>
        public bool IsBodyTracked { get; private set; }

        /// <summary>Unique id of the connected sensor, or empty.</summary>
        public string SensorId { get { return _sensor == null ? string.Empty : _sensor.UniqueKinectId; } }

        /// <summary>Number of bodies the sensor currently reports as tracked.</summary>
        public int TrackedBodyCount { get; private set; }

        /// <summary>Lean (sideways/forward tilt) of the selected body, for display.</summary>
        public Vec3 Lean { get; private set; }

        /// <summary>Human-readable state of the device layer.</summary>
        public string StatusText { get; private set; }

        /// <summary>Body frame rate, measured over a rolling window.</summary>
        public float FrameRate { get; private set; }

        /// <summary>Last device-layer exception message, if any.</summary>
        public string LastError { get; private set; }

        private int _framesInWindow;
        private DateTime _windowStart = DateTime.UtcNow;

        public KinectBodySource()
        {
            StatusText = "尚未连接";
        }

        // ------------------------------------------------------------------ lifecycle

        private static JointType[] BuildJointMap()
        {
            var map = new JointType[SkeletonFrameData.JointCount];
            var known = new HashSet<string>(StringComparer.Ordinal);
            foreach (JointType joint in (JointType[])Enum.GetValues(typeof(JointType)))
            {
                known.Add(joint.ToString());
            }

            for (int i = 0; i < SkeletonFrameData.JointCount; i++)
            {
                var name = ((KinectJoint)i).ToString();
                JointType joint;
                // The solver enum and the SDK enum share member names by design; matching on
                // the name (not the ordinal) keeps the mapping correct if either is reordered.
                // C# 7.3 cannot infer the type argument from the out parameter.
                if (!Enum.TryParse<JointType>(name, out joint) || !known.Contains(name))
                {
                    throw new InvalidOperationException(
                        "Kinect SDK has no joint named '" + name + "'; the adapter and the solver disagree.");
                }
                map[i] = joint;
            }
            return map;
        }

        private static Dictionary<JointType, int> BuildIndexMap()
        {
            var map = new Dictionary<JointType, int>(SkeletonFrameData.JointCount);
            for (int i = 0; i < SkeletonFrameData.JointCount; i++)
            {
                map[JointByIndex[i]] = i;
            }
            return map;
        }

        /// <summary>Opens the sensor if it is not open yet. Safe to call repeatedly.</summary>
        public void EnsureOpen()
        {
            if (_reader != null && _sensor != null && _sensor.IsOpen) { return; }

            // Do not hammer the driver while the device is absent.
            if ((DateTime.UtcNow - _lastOpenAttempt).TotalSeconds < 2.0) { return; }
            _lastOpenAttempt = DateTime.UtcNow;

            try
            {
                Release();

                _sensor = KinectSensor.GetDefault();
                if (_sensor == null)
                {
                    StatusText = "未检测到 Kinect（请检查 12V 供电与 USB 3.0 连接）";
                    return;
                }

                _bodies = new Body[_sensor.BodyFrameSource.BodyCount];

                // The Kinect v2 managed API exposes body frames only through a multi-source
                // reader: BodyFrameSource has no FrameArrived event of its own.
                //
                // Colour is requested too when the relay is active. Only one process can hold the
                // sensor, so the hand tracker cannot read the colour camera itself -- forwarding it
                // from here is the only way to give MediaPipe the Kinect's own image.
                FrameSourceTypes sources = FrameSourceTypes.Body;
                _reader = _sensor.OpenMultiSourceFrameReader(sources);
                if (_reader == null)
                {
                    StatusText = "无法创建帧读取器";
                    return;
                }

                _reader.MultiSourceFrameArrived += OnMultiSourceFrameArrived;

                if (RelayColour) { StartColourLoop(); }
                _sensor.IsAvailableChanged += OnIsAvailableChanged;
                _sensor.Open();

                StatusText = IsSensorAvailable ? "已连接" : "已连接但设备暂不可用";
                LastError = null;
            }
            catch (Exception ex)
            {
                LastError = ex.Message;
                StatusText = "打开 Kinect 失败：" + ex.Message;
                Release();
            }
        }

        private void OnIsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            StatusText = e.IsAvailable ? "已连接" : "设备暂时不可用（正在等待恢复）";

            if (!e.IsAvailable)
            {
                // Drop the reader so the next EnsureOpen() rebuilds the whole chain.
                Release();
            }
        }

        /// <summary>Closes the sensor and detaches every handler.</summary>
        public void Release()
        {
            StopColourLoop();
            if (_reader != null)
            {
                try { _reader.MultiSourceFrameArrived -= OnMultiSourceFrameArrived; } catch { }
                try { _reader.Dispose(); } catch { }
                _reader = null;
            }

            if (_sensor != null)
            {
                try { _sensor.IsAvailableChanged -= OnIsAvailableChanged; } catch { }
                try { _sensor.Close(); } catch { }
                _sensor = null;
            }

            _bodies = null;
        }

        public void Dispose()
        {
            Release();
        }

        // ------------------------------------------------------------------ frames

        private void OnMultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            MultiSourceFrame multiFrame = null;
            BodyFrame bodyFrame = null;

            try
            {
                MultiSourceFrameReference reference = e.FrameReference;
                if (reference == null) { return; }

                multiFrame = reference.AcquireFrame();
                if (multiFrame == null) { return; }

                bodyFrame = multiFrame.BodyFrameReference.AcquireFrame();
                if (bodyFrame == null) { return; }

                // Colour is deliberately NOT captured here.
                //
                // This callback runs on the sensor's own frame thread, and anything slow inside it
                // stalls body tracking -- which is what froze the whole program. Even the format
                // conversion is real work: 1920x1080 BGRA is 8.3 MB, so a copy per frame is a
                // quarter of a gigabyte a second of memory traffic on this thread.
                //
                // A separate thread captures colour from its own reader instead; see ColourLoop.

                if (_bodies == null) { return; }
                bodyFrame.GetAndRefreshBodyData(_bodies);

                Publish(bodyFrame);
            }
            catch (Exception ex)
            {
                LastError = ex.Message;
            }
            finally
            {
                if (bodyFrame != null) { try { bodyFrame.Dispose(); } catch { } }
                // MultiSourceFrame is not IDisposable in the Kinect v2 managed API.
            }
        }

        /// <summary>
        /// Captures colour on its OWN thread and hands it to the sink.
        ///
        /// Separate from the body callback on purpose. Colour is 8.3 MB a frame and the consumer is
        /// a busy Python process, so capture, conversion and hand-off all belong off the sensor's
        /// frame thread -- putting them there is what froze tracking.
        ///
        /// This opens its own reader for the colour source. The Kinect runtime allows more than one
        /// reader over the same source, and doing it this way means the body path never waits.
        /// </summary>
        private void ColourLoop()
        {
            ColorFrameReader reader = null;

            try
            {
                reader = _sensor.ColorFrameSource.OpenReader();
                if (reader == null) { return; }

                byte[] buffer = null;
                var clock = System.Diagnostics.Stopwatch.StartNew();

                while (_colourRunning)
                {
                    ColorFrame frame = null;
                    try
                    {
                        frame = reader.AcquireLatestFrame();
                        if (frame == null)
                        {
                            Thread.Sleep(5);
                            continue;
                        }

                        FrameDescription description = frame.FrameDescription;
                        int width = description.Width;
                        int height = description.Height;
                        int bytes = width * height * 4;

                        if (buffer == null || buffer.Length != bytes) { buffer = new byte[bytes]; }

                        // BGRA is what OpenCV expects by default.
                        frame.CopyConvertedFrameDataToArray(buffer, ColorImageFormat.Bgra);

                        Action<byte[], int, int> sink = ColourSink;
                        if (sink != null) { sink(buffer, width, height); }
                    }
                    catch
                    {
                        // A dropped colour frame is not worth reporting; body tracking is unaffected.
                    }
                    finally
                    {
                        if (frame != null) { try { frame.Dispose(); } catch { } }
                    }

                    // A short pause keeps this from spinning when the source runs slower than this
                    // loop; the relay rate-limits further anyway.
                    Thread.Sleep(2);
                }
            }
            catch
            {
                // Sensor went away, or the colour source is unavailable. Nothing more to do.
            }
            finally
            {
                if (reader != null) { try { reader.Dispose(); } catch { } }
            }
        }

        private void StartColourLoop()
        {
            if (_colourRunning || _sensor == null) { return; }

            _colourRunning = true;
            _colourThread = new Thread(ColourLoop);
            _colourThread.IsBackground = true;
            _colourThread.Start();
        }

        private void StopColourLoop()
        {
            _colourRunning = false;

            if (_colourThread != null)
            {
                try { _colourThread.Join(500); } catch { }
                _colourThread = null;
            }
        }

        private void Publish(BodyFrame bodyFrame)
        {
            Body selected = SelectBody();

            var next = new SkeletonFrameData();
            next.Time = (float)_clock.Elapsed.TotalSeconds;

            int tracked = 0;
            for (int i = 0; i < _bodies.Length; i++)
            {
                Body candidate = _bodies[i];
                if (candidate != null && candidate.IsTracked) { tracked++; }
            }
            TrackedBodyCount = tracked;

            if (selected == null)
            {
                next.Clear();
                IsBodyTracked = false;
                Lean = Vec3.Zero;
            }
            else
            {
                next.BodyTracked = true;
                next.TrackingId = selected.TrackingId;

                IReadOnlyDictionary<JointType, Joint> joints = selected.Joints;
                for (int i = 0; i < JointByIndex.Length; i++)
                {
                    JointType jointType = JointByIndex[i];
                    Joint joint;
                    if (!joints.TryGetValue(jointType, out joint)) { continue; }

                    next.Set((KinectJoint)i, ToCharacterSpace(joint.Position),
                        joint.TrackingState != TrackingState.NotTracked);
                }

                // Joint orientations: the rotation about each bone's own axis.
                //
                // This is what joint POSITIONS can never describe, and leaving it unread is why
                // the palm could not be oriented: the solver's shortest-arc swing produces no
                // twist, so a hanging arm dragged the forearm and hand round with the upper arm
                // and the palm faced outwards. Kinect reports the real rotation, so there is no
                // need to infer it from the thumb joint -- which was a guess that kept landing a
                // constant 180 degrees out.
                IReadOnlyDictionary<JointType, JointOrientation> orientations = selected.JointOrientations;
                for (int i = 0; i < JointByIndex.Length; i++)
                {
                    JointType jointType = JointByIndex[i];
                    JointOrientation orientation;
                    if (!orientations.TryGetValue(jointType, out orientation)) { continue; }

                    // Kinect gives (X, Y, Z, W); this project stores (X, Y, Z, W) as well, so only
                    // the coordinate conversion differs.
                    next.SetOrientation((KinectJoint)i,
                        ToCharacterSpace(new Quat(orientation.Orientation.X, orientation.Orientation.Y,
                            orientation.Orientation.Z, orientation.Orientation.W)),
                        true);
                }

                Lean = new Vec3(selected.Lean.X, selected.Lean.Y, 0f);
                IsBodyTracked = true;
            }

            lock (_gate)
            {
                CopyInto(next, _frame);
            }

            CountFrame();
        }

        /// <summary>
        /// Picks the body to follow. Kinect tracks up to six people, and the selection must be
        /// stable across frames or the avatar teleports when a second person walks in.
        /// </summary>
        private Body SelectBody()
        {
            if (_bodies == null) { return null; }

            // Stick with the previously chosen body while it is still tracked.
            if (_preferredTrackingId != 0 && DateTime.UtcNow < _preferredIdExpiry)
            {
                for (int i = 0; i < _bodies.Length; i++)
                {
                    Body candidate = _bodies[i];
                    if (candidate != null && candidate.IsTracked && candidate.TrackingId == _preferredTrackingId)
                    {
                        return candidate;
                    }
                }
            }

            // Otherwise take the tracked body nearest the sensor: for a seated streamer that is
            // the person at the desk, not someone walking past behind them.
            Body best = null;
            float bestDepth = float.MaxValue;

            for (int i = 0; i < _bodies.Length; i++)
            {
                Body candidate = _bodies[i];
                if (candidate == null || !candidate.IsTracked) { continue; }

                Joint spine;
                float depth;
                if (candidate.Joints.TryGetValue(JointType.SpineBase, out spine)
                    && spine.TrackingState != TrackingState.NotTracked)
                {
                    depth = spine.Position.Z;
                }
                else
                {
                    depth = float.MaxValue - 1f;
                }

                if (depth < bestDepth)
                {
                    bestDepth = depth;
                    best = candidate;
                }
            }

            if (best != null)
            {
                _preferredTrackingId = best.TrackingId;
                // Re-evaluate every few seconds so the follower can switch if the user moves.
                _preferredIdExpiry = DateTime.UtcNow.AddSeconds(10);
            }

            return best;
        }

        private void CountFrame()
        {
            _framesInWindow++;
            double elapsed = (DateTime.UtcNow - _windowStart).TotalSeconds;
            if (elapsed >= 1.0)
            {
                FrameRate = (float)(_framesInWindow / elapsed);
                _framesInWindow = 0;
                _windowStart = DateTime.UtcNow;
            }
        }

        // ------------------------------------------------------------------ access

        /// <summary>
        /// Copies the latest frame into <paramref name="destination"/>.
        /// Returns false when no body is currently tracked.
        /// </summary>
        public bool TryGetFrame(SkeletonFrameData destination)
        {
            lock (_gate)
            {
                CopyInto(_frame, destination);
                return _frame.BodyTracked;
            }
        }

        private static void CopyInto(SkeletonFrameData source, SkeletonFrameData destination)
        {
            destination.BodyTracked = source.BodyTracked;
            destination.TrackingId = source.TrackingId;
            destination.Time = source.Time;

            for (int i = 0; i < SkeletonFrameData.JointCount; i++)
            {
                destination.Positions[i] = source.Positions[i];
                destination.Tracked[i] = source.Tracked[i];
            }
        }

        /// <summary>
        /// Kinect camera space to the solver's character space.
        ///
        /// The mapping is (x, y, z) -> (x, -y, -z): X passes through, Y and Z are both negated.
        ///
        ///  * Y is negated so that "up" in the camera's picture is the character standing up.
        ///    The character's own +Y is up and the camera's +Y is also up, so the character has to
        ///    be the camera's 180-degree rotation, which negates Y and Z together.
        ///  * Z is negated because the user faces the sensor, so the character's forward (+Z)
        ///    points towards the camera, i.e. towards DECREASING camera Z.
        ///  * X passes through because the user's own left is at negative X in camera space and
        ///    also at negative X in character space.
        ///
        /// Three earlier versions of this line each behaved differently on the user's hardware:
        ///
        ///  * negating X inverted every rotation and transposed each gesture onto another axis
        ///    (raising an arm measured as lowering it);
        ///  * negating nothing reversed depth and the vertical axis;
        ///  * negating only Z left the avatar upside down.
        ///
        /// A horizontal T-pose looks correct under every one of those, because a sideways arm has
        /// neither depth nor vertical extent to get wrong -- which is why the guarding assertions
        /// check a raised arm, a forward reach, a backward move and a lean, separately.
        /// </summary>
        public static Vec3 ToCharacterSpace(CameraSpacePoint point)
        {
            return new Vec3(point.X, point.Y, -point.Z);
        }

        /// <summary>
        /// The same conversion for a ROTATION.
        ///
        /// Positions map as (x, y, z) -> (x, y, -z), which is a reflection. A rotation carried
        /// through a reflection conjugates by it, and for the mirror through the XY plane that
        /// works out to negating Z and W: R' = M R M, with the resulting quaternion equal to
        /// (-x, -y, z, w) up to sign, or equivalently (x, y, -z, -w) in the convention used here.
        ///
        /// It is NOT simply "negate Z", which would keep the handedness and quietly describe a
        /// different rotation -- the kind of silent sign error this project has hit repeatedly.
        /// </summary>
        public static Quat ToCharacterSpace(Quat orientation)
        {
            return new Quat(orientation.X, orientation.Y, -orientation.Z, -orientation.W);
        }
    }
}
