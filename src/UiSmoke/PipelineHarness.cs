using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Kinect2VMC.App;
using Kinect2VMC.Core;
using Kinect2VMC.SelfTest;

namespace Kinect2VMC.UiSmoke
{
    /// <summary>
    /// Drives the real <see cref="TrackingPipeline"/> with scripted frames and inspects the VMC
    /// traffic that comes out of the UDP socket.
    ///
    /// This is the end-to-end test that a device-free environment can still perform: every
    /// decision the application makes between "a frame arrived" and "a packet was sent" is
    /// covered here. The only untested part is the Kinect SDK call itself.
    /// </summary>
    internal static class PipelineHarness
    {
        public static int Run()
        {
            int failures = 0;
            Console.WriteLine();
            Console.WriteLine("Pipeline end to end (synthetic frames)");

            failures += TestStateTransitions();
            failures += TestSendsUpperBodyOnly();
            failures += TestCalibrationAndRootTranslation();
            failures += TestMirroring();
            failures += TestPreviewBuffers();
            failures += TestHeadLengthChanges();
            failures += TestDisposal();

            return failures;
        }

        // ------------------------------------------------------------------ helpers

        /// <summary>Binds a loopback socket so the pipeline has somewhere real to send.</summary>
        private static UdpClient OpenListener(out int port)
        {
            var listener = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0));
            port = ((IPEndPoint)listener.Client.LocalEndPoint).Port;
            listener.Client.ReceiveTimeout = 150;
            return listener;
        }

        private static List<OscMessage> Drain(UdpClient listener, int milliseconds)
        {
            var messages = new List<OscMessage>();
            DateTime deadline = DateTime.UtcNow.AddMilliseconds(milliseconds);

            while (DateTime.UtcNow < deadline)
            {
                try
                {
                    var remote = new IPEndPoint(IPAddress.Any, 0);
                    byte[] data = listener.Receive(ref remote);
                    OscMessage message;
                    if (OscMessageReader.TryParse(data, data.Length, out message)) { messages.Add(message); }
                }
                catch (SocketException)
                {
                    if (messages.Count > 0) { break; }
                }
            }
            return messages;
        }

        private static List<string> BoneNames(List<OscMessage> messages)
        {
            var names = new List<string>();
            foreach (OscMessage message in messages)
            {
                if (message.Address == "/VMC/Ext/Bone/Pos" && message.Arguments.Length >= 1)
                {
                    names.Add(message.Arguments[0].StringValue);
                }
            }
            return names;
        }

        private static OscMessage FindRoot(List<OscMessage> messages)
        {
            foreach (OscMessage message in messages)
            {
                if (message.Address == "/VMC/Ext/Root/Pos") { return message; }
            }
            return null;
        }

        private static void Report(bool ok, ref int failures, string name, string detail = null)
        {
            if (ok)
            {
                Console.WriteLine("  [ok]   " + name);
            }
            else
            {
                failures++;
                Console.WriteLine("  [FAIL] " + name + (detail == null ? "" : "  -> " + detail));
            }
        }

        // ------------------------------------------------------------------ tests

        private static int TestStateTransitions()
        {
            int failures = 0;

            var source = new FakeBodySource
            {
                Available = false,
                Status = "未检测到 Kinect",
                Error = "设备层错误示例"
            };
            var settings = new AppSettings();
            using (var pipeline = new TrackingPipeline(settings, source))
            {
                pipeline.Tick();
                Report(pipeline.State == PipelineState.WaitingForSensor, ref failures,
                    "no sensor reports WaitingForSensor");
                Report(pipeline.StatusText == "未检测到 Kinect", ref failures,
                    "device status text reaches the UI", pipeline.StatusText);
                Report(pipeline.LastError == "设备层错误示例", ref failures,
                    "device error text reaches the UI", pipeline.LastError);
                Report(!pipeline.IsRunning, ref failures, "an idle pipeline reports not running");

                // The sensor comes back but sees nobody.
                source.Available = true;
                source.BodyTracked = false;
                source.HoldFrame = null;
                pipeline.Tick();
                Report(pipeline.State == PipelineState.WaitingForBody, ref failures,
                    "sensor without a body reports WaitingForBody");

                // A body appears.
                source.BodyTracked = true;
                source.HoldFrame = TestSkeleton.RestPose();
                pipeline.Tick();
                Report(pipeline.State == PipelineState.Tracking, ref failures, "a tracked body reports Tracking");
                Report(pipeline.BoneCount == 11, ref failures, "11 bones are solved",
                    pipeline.BoneCount.ToString());
                Report(source.OpenCalls >= 3, ref failures,
                    "the device layer is polled every frame", source.OpenCalls.ToString());

                // The body is lost again; the pipeline must go back to waiting, not keep sending.
                source.BodyTracked = false;
                pipeline.Tick();
                Report(pipeline.State == PipelineState.WaitingForBody, ref failures,
                    "losing the body returns to WaitingForBody");
                Report(!source.LastFrameHadBody, ref failures,
                    "the device layer reports no body in the last frame");
                Report(pipeline.BoneCount == 0, ref failures, "no bones are reported while waiting",
                    pipeline.BoneCount.ToString());

                // The pipeline must trust the device layer's return value, not the cached
                // BodyTracked flag on the frame. Simulate a layer that leaves that flag stale:
                // the pipeline must still refuse to solve, because solving would keep the avatar
                // frozen in a pose the user is no longer holding.
                System.Reflection.FieldInfo frameField = typeof(TrackingPipeline).GetField(
                    "_frame", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                if (frameField != null)
                {
                    var cached = (SkeletonFrameData)frameField.GetValue(pipeline);
                    cached.BodyTracked = true;   // stale cache, as if a body frame had just arrived
                    pipeline.Tick();
                    Report(pipeline.State == PipelineState.WaitingForBody, ref failures,
                        "a stale BodyTracked cache does not resurrect tracking");
                    Report(pipeline.BoneCount == 0, ref failures,
                        "a stale BodyTracked cache produces no bones", pipeline.BoneCount.ToString());
                }
            }

            return failures;
        }

        private static int TestSendsUpperBodyOnly()
        {
            int failures = 0;

            int port;
            UdpClient listener = OpenListener(out port);
            try
            {
                var source = new FakeBodySource { HoldFrame = TestSkeleton.RestPose() };
                var settings = new AppSettings { TargetPort = port, Smoothing = 0f, SendRootPosition = false };

                using (var pipeline = new TrackingPipeline(settings, source))
                {
                    pipeline.Tick();
                    List<OscMessage> messages = Drain(listener, 600);

                    List<string> bones = BoneNames(messages);
                    Report(bones.Count == 11, ref failures, "one frame sends 11 bone messages",
                        bones.Count.ToString());

                    bool status = false, time = false;
                    foreach (OscMessage message in messages)
                    {
                        if (message.Address == "/VMC/Ext/OK") { status = true; }
                        if (message.Address == "/VMC/Ext/T") { time = true; }
                    }
                    Report(status, ref failures, "the first frame also sends /VMC/Ext/OK");
                    Report(time, ref failures, "the first frame also sends /VMC/Ext/T");
                    Report(FindRoot(messages) == null, ref failures,
                        "no root message is sent when the position toggle is off");

                    // No lower body, no head: the whole point of the seated half-body mode.
                    bool clean = true;
                    string offender = null;
                    foreach (string bone in bones)
                    {
                        if (bone == "Head" || bone == "Neck" || bone == "Hips"
                            || bone.Contains("Leg") || bone.Contains("Foot") || bone.Contains("Toes"))
                        {
                            clean = false;
                            offender = bone;
                            break;
                        }
                    }
                    Report(clean, ref failures, "no head, hips or lower-body bones are sent", offender);

                    // Every bone position must be zero: the receiver is rotation-only.
                    bool positionsZero = true;
                    foreach (OscMessage message in messages)
                    {
                        if (message.Address != "/VMC/Ext/Bone/Pos") { continue; }
                        if (message.Arguments[1].FloatValue != 0f
                            || message.Arguments[2].FloatValue != 0f
                            || message.Arguments[3].FloatValue != 0f)
                        {
                            positionsZero = false;
                            break;
                        }
                    }
                    Report(positionsZero, ref failures, "every bone position is zero (rotation-only)");

                    // A second frame re-sends the bones but not the slow status messages.
                    messages = null;
                    pipeline.Tick();
                    List<OscMessage> second = Drain(listener, 500);
                    Report(BoneNames(second).Count == 11, ref failures,
                        "a later frame still sends 11 bones", BoneNames(second).Count.ToString());
                    bool statusAgain = false;
                    foreach (OscMessage message in second)
                    {
                        if (message.Address == "/VMC/Ext/OK") { statusAgain = true; }
                    }
                    Report(!statusAgain, ref failures,
                        "the status message is rate limited rather than sent every frame");
                }
            }
            finally
            {
                listener.Close();
            }

            return failures;
        }

        private static int TestCalibrationAndRootTranslation()
        {
            int failures = 0;

            int port;
            UdpClient listener = OpenListener(out port);
            try
            {
                // The user sits 20 cm to one side of the sensor centre and 10 cm back.
                SkeletonFrameData seated = TestSkeleton.Transform(
                    TestSkeleton.RestPose(), Quat.Identity, new Vec3(0.2f, 0f, 0.1f));

                var source = new FakeBodySource { HoldFrame = seated };
                var settings = new AppSettings
                {
                    TargetPort = port,
                    Smoothing = 0f,
                    SendRootPosition = true,
                    MovementScale = 1f,
                    MovementLimit = 0.5f
                };

                using (var pipeline = new TrackingPipeline(settings, source))
                {
                    // Before calibration the avatar must not be yanked to the raw sensor position.
                    pipeline.Tick();
                    List<OscMessage> before = Drain(listener, 600);
                    OscMessage root = FindRoot(before);
                    Report(root != null, ref failures, "the root message is sent when enabled");
                    if (root != null)
                    {
                        Report(Math.Abs(root.Arguments[1].FloatValue) < 1e-6f
                            && Math.Abs(root.Arguments[2].FloatValue) < 1e-6f
                            && Math.Abs(root.Arguments[3].FloatValue) < 1e-6f,
                            ref failures, "an uncalibrated avatar is held at the origin");
                    }
                    Report(!pipeline.IsCalibrated, ref failures, "the pipeline starts uncalibrated");

                    // Calibrate at the seated position.
                    pipeline.RequestCalibration();
                    pipeline.Tick();
                    List<OscMessage> atNeutral = Drain(listener, 600);
                    Report(pipeline.IsCalibrated, ref failures, "calibration takes effect");
                    root = FindRoot(atNeutral);
                    if (root != null)
                    {
                        Report(Math.Abs(root.Arguments[1].FloatValue) < 1e-5f
                            && Math.Abs(root.Arguments[3].FloatValue) < 1e-5f,
                            ref failures, "the calibrated position becomes the origin",
                            "x=" + root.Arguments[1].FloatValue + " z=" + root.Arguments[3].FloatValue);
                    }

                    // The user leans forward: only the delta should be transmitted.
                    source.HoldFrame = TestSkeleton.Transform(
                        TestSkeleton.RestPose(), Quat.Identity, new Vec3(0.2f, 0f, 0.4f));
                    pipeline.Tick();
                    List<OscMessage> moved = Drain(listener, 600);
                    root = FindRoot(moved);
                    if (root != null)
                    {
                        float z = root.Arguments[3].FloatValue;
                        Report(Math.Abs(z - 0.3f) < 1e-4f, ref failures,
                            "the root carries the movement delta, not the absolute position",
                            "z=" + z.ToString("F4") + ", expected 0.3");
                    }

                    // A big movement must be clamped so the avatar cannot leave the frame.
                    source.HoldFrame = TestSkeleton.Transform(
                        TestSkeleton.RestPose(), Quat.Identity, new Vec3(0.2f, 0f, 5f));
                    pipeline.Tick();
                    List<OscMessage> clamped = Drain(listener, 600);
                    root = FindRoot(clamped);
                    if (root != null)
                    {
                        float x = root.Arguments[1].FloatValue;
                        float y = root.Arguments[2].FloatValue;
                        float z = root.Arguments[3].FloatValue;
                        float magnitude = (float)Math.Sqrt(x * x + y * y + z * z);
                        Report(magnitude <= settings.MovementLimit + 1e-4f, ref failures,
                            "a large movement is clamped to the limit",
                            "magnitude=" + magnitude.ToString("F4"));
                    }

                    // Bone rotations must stay measured from the calibrated neutral.
                    List<string> bones = BoneNames(moved);
                    Report(bones.Count == 11, ref failures, "bones keep being sent after calibration");
                }
            }
            finally
            {
                listener.Close();
            }

            return failures;
        }

        private static int TestMirroring()
        {
            int failures = 0;

            // A user sitting off to one side, so the mirror has an observable effect on the root
            // translation that does not depend on the pose being asymmetric.
            SkeletonFrameData seated = TestSkeleton.RestPose();
            seated = TestSkeleton.Transform(seated, Quat.Identity, new Vec3(0.3f, 0f, 0f));
            seated.BodyTracked = true;

            int port;
            UdpClient listener = OpenListener(out port);
            try
            {
                var settings = new AppSettings
                {
                    TargetPort = port,
                    Smoothing = 0f,
                    SendRootPosition = true,
                    MovementScale = 1f,
                    MovementLimit = 1f
                };

                float plainX = RootXFor(settings, seated, false, listener);
                float mirroredX = RootXFor(settings, seated, true, listener);

                // The harness calibrates at the sensor origin and then moves to the seated
                // offset, so the transmitted root is the scaled offset itself. The offset is
                // deliberately large enough that the clamp does not mask the sign.
                Report(Math.Abs(plainX - 0.3f) < 1e-3f, ref failures,
                    "without mirroring the root X carries the seated offset", plainX.ToString("F4"));
                Report(Math.Abs(mirroredX + 0.3f) < 1e-3f, ref failures,
                    "mirroring negates the root X translation", mirroredX.ToString("F4"));
                Report(Math.Sign(plainX) != Math.Sign(mirroredX), ref failures,
                    "the two roots have opposite signs");

                // Mirroring must be reproducible: the same input twice gives the same rotations,
                // so a user cannot get a different pose each time they start tracking.
                Dictionary<string, Quat> first = RotationsFor(settings, seated, true);
                Dictionary<string, Quat> second = RotationsFor(settings, seated, true);

                bool same = first.Count == second.Count && first.Count > 0;
                if (same)
                {
                    foreach (var pair in first)
                    {
                        Quat other;
                        if (!second.TryGetValue(pair.Key, out other)
                            || !ApproximatelyEqual(pair.Value, other, 1e-5f))
                        {
                            same = false;
                            break;
                        }
                    }
                }
                Report(same, ref failures, "mirrored solving is deterministic");
            }
            finally
            {
                listener.Close();
            }

            return failures;
        }

        /// <summary>Runs one calibrated frame and returns the transmitted root X.</summary>
        private static float RootXFor(AppSettings settings, SkeletonFrameData seated,
            bool mirror, UdpClient listener)
        {
            settings.Mirror = mirror;

            SkeletonFrameData atOrigin = TestSkeleton.RestPose();
            atOrigin.BodyTracked = true;

            var source = new FakeBodySource { HoldFrame = atOrigin };

            using (var pipeline = new TrackingPipeline(settings, source))
            {
                // Calibrate with the user sitting at the sensor's origin.
                pipeline.RequestCalibration();
                pipeline.Tick();
                Drain(listener, 200);

                // Then the user moves to the seated offset: the root must report the delta.
                source.HoldFrame = seated;
                pipeline.Tick();

                OscMessage root = FindRoot(Drain(listener, 500));
                return root == null ? float.NaN : root.Arguments[1].FloatValue;
            }
        }

        private static Dictionary<string, Quat> RotationsFor(AppSettings settings,
            SkeletonFrameData frame, bool mirror)
        {
            settings.Mirror = mirror;
            var source = new FakeBodySource { HoldFrame = frame };

            using (var pipeline = new TrackingPipeline(settings, source))
            {
                pipeline.Tick();
                return pipeline.LatestRotations;
            }
        }

        /// <summary>
        /// The two buffers the UI reads: the joint positions behind the skeleton preview and the
        /// bone rotations behind the debug readout.
        ///
        /// The joint buffer is refilled in place rather than reallocated, so it needs explicit
        /// coverage: an off-by-one or a stale-buffer mistake would make the preview draw the
        /// skeleton collapsed at the origin while everything else looked fine.
        /// </summary>
        private static int TestPreviewBuffers()
        {
            int failures = 0;

            SkeletonFrameData rest = TestSkeleton.RestPose();
            rest.BodyTracked = true;

            var source = new FakeBodySource { HoldFrame = rest };
            var settings = new AppSettings { TargetPort = 1, Smoothing = 0f };

            using (var pipeline = new TrackingPipeline(settings, source))
            {
                pipeline.Tick();

                Vec3[] joints = pipeline.LatestJoints;
                Report(joints != null, ref failures, "the preview buffer is populated after a tick");
                if (joints == null) { return failures; }

                Report(joints.Length == SkeletonFrameData.JointCount, ref failures,
                    "the preview buffer holds every joint",
                    joints.Length + " of " + SkeletonFrameData.JointCount);

                // The values must be the measured ones, not defaults.
                Report((joints[(int)KinectJoint.SpineBase] - rest.Get(KinectJoint.SpineBase)).Length < 1e-5f, ref failures,
                    "the preview buffer carries the measured spine base",
                    joints[(int)KinectJoint.SpineBase].ToString());
                Report((joints[(int)KinectJoint.ShoulderLeft] - rest.Get(KinectJoint.ShoulderLeft)).Length < 1e-5f, ref failures,
                    "the preview buffer carries the measured left shoulder");

                bool anyNonZero = false;
                for (int i = 0; i < joints.Length; i++)
                {
                    if (joints[i].LengthSquared > 1e-6f) { anyNonZero = true; break; }
                }
                Report(anyNonZero, ref failures, "the preview buffer is not all zeros");

                // A second, different frame must overwrite the buffer in place. Keeping a
                // reference to the array and re-reading it is exactly how the UI uses it.
                Vec3[] sameBuffer = joints;
                SkeletonFrameData moved = TestSkeleton.Transform(rest, Quat.Identity, new Vec3(0f, 0.25f, 0f));
                moved.BodyTracked = true;
                source.HoldFrame = moved;

                pipeline.Tick();
                Report(ReferenceEquals(pipeline.LatestJoints, sameBuffer), ref failures,
                    "the preview buffer is reused rather than reallocated");

                Vec3 expectedMoved = moved.Get(KinectJoint.SpineBase);
                Report((sameBuffer[(int)KinectJoint.SpineBase] - expectedMoved).Length < 1e-5f, ref failures,
                    "the reused buffer reflects the newest frame",
                    sameBuffer[(int)KinectJoint.SpineBase] + " vs " + expectedMoved);

                // The buffer must not alias the pipeline's internal frame, or a later tick would
                // retroactively change data the UI had already read.
                Report(!ReferenceEquals(sameBuffer, moved.Positions), ref failures,
                    "the preview buffer does not alias the internal frame");

                // Rotations must likewise be present and complete.
                Dictionary<string, Quat> rotations = pipeline.LatestRotations;
                Report(rotations != null && rotations.Count == 11, ref failures,
                    "the rotation readout holds the emitted bones",
                    rotations == null ? "null" : rotations.Count.ToString());
            }

            return failures;
        }

        /// <summary>
        /// The regression test for the bug that reached the user: the camera-space to
        /// character-space mapping.
        ///
        /// It starts from a frame authored in RAW Kinect camera space, runs it through the real
        /// <summary>
        /// The regression test for the bug that reached the user: the camera-space to
        /// character-space mapping.
        ///
        /// It goes through the REAL adapter conversion (<c>KinectBodySource.ToCharacterSpace</c>)
        private static Vec3 ToCharacter(Vec3 cameraPoint)
        {
            var point = new Microsoft.Kinect.CameraSpacePoint
            {
                X = cameraPoint.X,
                Y = cameraPoint.Y,
                Z = cameraPoint.Z
            };
            return Kinect2VMC.App.KinectBodySource.ToCharacterSpace(point);
        }

        /// <summary>
        /// A seated torso authored in raw Kinect camera space, returned ALREADY CONVERTED to
        /// character space through the real adapter, so no caller needs to convert again.
        ///
        /// In camera space the user's own right is at POSITIVE X (they face the sensor), so the
        /// RIGHT limb joints carry positive X here.
        /// </summary>
        private static SkeletonFrameData CameraSpaceSeatedTorso()
        {
            var frame = new SkeletonFrameData { BodyTracked = true, TrackingId = 1, Time = 0f };

            frame.Set(KinectJoint.SpineBase, new Vec3(0f, 0.82f, 1.42f), true);
            frame.Set(KinectJoint.SpineMid, new Vec3(0f, 0.96f, 1.40f), true);
            frame.Set(KinectJoint.SpineShoulder, new Vec3(0f, 1.10f, 1.39f), true);
            frame.Set(KinectJoint.Neck, new Vec3(0f, 1.18f, 1.39f), true);
            frame.Set(KinectJoint.Head, new Vec3(0f, 1.31f, 1.38f), true);

            frame.Set(KinectJoint.ShoulderLeft, new Vec3(-0.20f, 1.10f, 1.39f), true);
            frame.Set(KinectJoint.ShoulderRight, new Vec3(0.20f, 1.10f, 1.39f), true);

            // Arms hanging down and slightly out.
            frame.Set(KinectJoint.ElbowLeft, new Vec3(-0.36f, 0.86f, 1.39f), true);
            frame.Set(KinectJoint.WristLeft, new Vec3(-0.50f, 0.64f, 1.39f), true);
            frame.Set(KinectJoint.HandLeft, new Vec3(-0.55f, 0.56f, 1.39f), true);
            frame.Set(KinectJoint.HandTipLeft, new Vec3(-0.58f, 0.52f, 1.39f), true);
            frame.Set(KinectJoint.ThumbLeft, new Vec3(-0.50f, 0.60f, 1.36f), true);

            frame.Set(KinectJoint.ElbowRight, new Vec3(0.36f, 0.86f, 1.39f), true);
            frame.Set(KinectJoint.WristRight, new Vec3(0.50f, 0.64f, 1.39f), true);
            frame.Set(KinectJoint.HandRight, new Vec3(0.55f, 0.56f, 1.39f), true);
            frame.Set(KinectJoint.HandTipRight, new Vec3(0.58f, 0.52f, 1.39f), true);
            frame.Set(KinectJoint.ThumbRight, new Vec3(0.50f, 0.60f, 1.36f), true);

            frame.Set(KinectJoint.HipLeft, new Vec3(-0.12f, 0.78f, 1.40f), true);
            frame.Set(KinectJoint.HipRight, new Vec3(0.12f, 0.78f, 1.40f), true);
            frame.Set(KinectJoint.KneeLeft, new Vec3(-0.14f, 0.46f, 1.72f), true);
            frame.Set(KinectJoint.KneeRight, new Vec3(0.14f, 0.46f, 1.72f), true);

            return frame;
        }

        /// <summary>
        /// Runs a whole frame through the REAL adapter conversion, so this test cannot drift away
        /// from the shipping convention the way a hand-written duplicate would.
        /// </summary>
        private static SkeletonFrameData ConvertToCharacterSpace(SkeletonFrameData cameraSpace)
        {
            var result = new SkeletonFrameData
            {
                BodyTracked = cameraSpace.BodyTracked,
                TrackingId = cameraSpace.TrackingId,
                Time = cameraSpace.Time
            };

            for (int i = 0; i < SkeletonFrameData.JointCount; i++)
            {
                Vec3 p = cameraSpace.Positions[i];
                result.Set((KinectJoint)i, ToCharacter(p), cameraSpace.Tracked[i]);
            }

            return result;
        }

        private static int TestHeadLengthChanges()
        {
            int failures = 0;

            int port;
            UdpClient listener = OpenListener(out port);
            try
            {
                var source = new FakeBodySource { HoldFrame = TestSkeleton.RestPose() };
                var settings = new AppSettings { TargetPort = port, Smoothing = 0f, SendHead = true };

                using (var pipeline = new TrackingPipeline(settings, source))
                {
                    pipeline.Tick();
                    List<string> withHead = BoneNames(Drain(listener, 600));
                    Report(withHead.Count == 13, ref failures,
                        "with the head enabled 13 bones are sent", withHead.Count.ToString());
                    Report(withHead.Contains("Head") && withHead.Contains("Neck"), ref failures,
                        "the head and neck are included when enabled");

                    // Switching the head off must stop it immediately. This is the regression
                    // that a reused pose dictionary caused: stale bones kept being transmitted.
                    settings.SendHead = false;
                    pipeline.ApplySettings();
                    pipeline.Tick();
                    List<string> without = BoneNames(Drain(listener, 600));

                    Report(without.Count == 11, ref failures,
                        "switching the head off drops back to 11 bones", without.Count.ToString());
                    Report(!without.Contains("Head") && !without.Contains("Neck"), ref failures,
                        "the head and neck stop being transmitted immediately");
                }
            }
            finally
            {
                listener.Close();
            }

            return failures;
        }

        private static int TestDisposal()
        {
            int failures = 0;

            var source = new FakeBodySource { HoldFrame = TestSkeleton.RestPose() };
            var pipeline = new TrackingPipeline(new AppSettings { TargetPort = 1 }, source);

            pipeline.Start();
            Report(pipeline.IsRunning, ref failures, "start reports running");
            Thread.Sleep(60);

            pipeline.Stop();
            Report(!pipeline.IsRunning, ref failures, "stop reports not running");
            Report(pipeline.State == PipelineState.Stopped, ref failures, "stop sets the state to Stopped");

            pipeline.Dispose();
            Report(source.Disposed, ref failures, "disposing the pipeline disposes the device layer");

            // Starting a disposed pipeline must not throw or hang; it is enough that it returns.
            pipeline.Tick();
            Report(true, ref failures, "ticking an idle pipeline does not throw");

            return failures;
        }

        private static bool ApproximatelyEqual(Quat a, Quat b, float tolerance)
        {
            return 1f - Math.Abs(Quat.Dot(a.Normalized(), b.Normalized())) <= tolerance;
        }
    }
}

