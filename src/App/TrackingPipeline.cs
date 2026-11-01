using System;
using System.Collections.Generic;
using System.Threading;
using Kinect2VMC.Core;

namespace Kinect2VMC.App
{
    /// <summary>User-facing settings. Serialised to a small key=value file next to the exe.</summary>
    public sealed class AppSettings
    {
        public string TargetAddress = "127.0.0.1";
        public int TargetPort = 39539;

        /// <summary>0 = raw, higher = smoother. 0.45 is a reasonable default for 30 Hz data.</summary>
        public float Smoothing = 0.45f;

        /// <summary>Send Neck and Head from Kinect. Off by default: VSeeFace owns the head.</summary>
        public bool SendHead;

        /// <summary>Send the root position so the avatar follows the user's real movement.</summary>
        public bool SendRootPosition = true;

        /// <summary>Metres of real movement mapped to one metre of avatar movement.</summary>
        public float MovementScale = 1.0f;

        /// <summary>Clamp on how far the avatar may drift from its calibrated origin, in metres.</summary>
        public float MovementLimit = 0.5f;

        /// <summary>Mirror the tracking left/right, for a camera placed behind the user.</summary>
        public bool Mirror;

        /// <summary>
        /// Degrees to tilt the measured skeleton backwards to compensate for a Kinect mounted
        /// above the user and angled downwards.
        ///
        /// A downward-looking sensor reports the user's vertical axis as leaning away from it, so
        /// an upright seated user comes through as leaning forward and the avatar reproduces that
        /// lean. Rotating the whole frame about the character's right axis by this angle cancels
        /// it, which is usually preferable to moving the sensor: mounting it level is often
        /// impossible without the monitor occluding the view.
        ///
        /// Positive values tilt the skeleton backwards. 0 disables the correction.
        /// </summary>
        public float KinectTiltDegrees;
        /// <summary>Use the arms-hanging (A-pose) bind reference instead of horizontal T.</summary>
        public bool HangingArmReference;

        /// <summary>Emit Hips and the leg chains as well, for full-body tracking.</summary>
        public bool SendFullBody;

        /// <summary>Emit the root rotation so the avatar turns with the user.</summary>
        public bool SendRootRotation = true;

        /// <summary>Which way the fingers fold when a fist is detected.</summary>
        public bool FingerCurlInward = true;
        /// <summary>
        /// Emit LOCAL bone rotations (each relative to its parent) instead of world ones.
        /// This is what VMC documents for /VMC/Ext/Bone/Pos and what Warudo applies.
        /// Turning it off makes every joint compound its parent, so a straight arm bends.
        /// </summary>
        public bool EmitWorldRotations;

        /// <summary>
        /// Write the measured hand shape metrics to hand-log.csv beside the executable.
        /// Kinect reports the fingertip and thumb joints as inferred rather than tracked,
        /// so their usable range cannot be predicted -- it has to be measured on the real
        /// sensor before any fist threshold can be trusted.
        /// </summary>
        public bool LogHandMetrics;

        /// <summary>
        /// Send the finger bones so a closed hand reads as a fist. Off by default: the
        /// hand joints are inferred, so the state can flicker, and a jittering hand looks
        /// worse than one that simply stays open.
        /// </summary>
        public bool EmitFingers;

        /// <summary>
        /// Send the hand bone. Turn OFF to test whether a palm pointing the wrong way comes
        /// from this program or from the rig: with it off the hand keeps the rig's idle pose,
        /// so if the palm still flips, the cause is not here.
        /// </summary>
        public bool SendHand = true;

        /// <summary>
        /// 把整套骨架直接发给 Warudo 里的插件，不经过 VMC。
        ///
        /// 默认关闭：这条通路刚建立，要先和 VMC 对照验证。打开之后建议同时关掉 VMC 发送，
        /// 否则同一根骨骼有两个写入者，谁后写谁生效 —— IK 尤其抢不过。
        /// </summary>
        public bool SendSkeletonToPlugin;

        /// <summary>插件端口。与掌心帧同端口，靠包头 magic 区分。</summary>
        public int PluginPort = PluginBoneSender.DefaultPort;

        /// <summary>
        /// Forward the Kinect's colour stream to the hand tracker over loopback.
        ///
        /// For when the webcam is unavailable or already taken by Warudo's face capture.
        /// The Kinect has a wide field of view, so the hand is smaller in frame than a
        /// webcam would show it; whether MediaPipe still finds it is a question for the
        /// real setup rather than for arithmetic.
        /// </summary>
        public bool RelayColour;

        /// <summary>
        /// Use the camera-measured palm orientation for the hand bone, composed with
        /// Kinect's arm. This is the whole reason the tracker sends anything back: a
        /// hand rotation needs the arm from Kinect and the palm from the camera, and
        /// sending them separately makes the two fight over one bone.
        /// </summary>
        public bool UseCameraHand = true;

        /// <summary>
        /// How much of a palm turn the FOREARM takes, 0..1.
        ///
        /// A real wrist cannot twist far alone: most of a palm turn is the forearm
        /// rotating about its own axis. Putting the whole correction on the hand asked one
        /// joint to do two joints' work, and the wrist deformed instead of the palm
        /// turning. 0.6 is roughly the human proportion.
        /// </summary>
        public float ForearmTwistShare = 0.6f;

        /// <summary>
        /// Set the hand bone's rotation outright from the camera, instead of applying only
        /// the change since a reference. This is how Warudo's own MediaPipe tracker behaves,
        /// and it is what makes the hand MATCH a pose rather than merely follow it.
        /// </summary>
        public bool AbsoluteHand = true;

        /// <summary>Mirror the measured palm frame about the vertical plane.</summary>
        public bool MirrorHandY;

        /// <summary>
        /// Divide the arm out of the camera's palm orientation, as a bone local rotation
        /// implies.
        ///
        /// Correct in principle and harmful in practice here: it makes the hand's accuracy
        /// depend entirely on the ARM's accuracy, and Kinect's forearm comes from a 9 cm
        /// bone. A correct camera measurement minus a wrong parent is a wrong local
        /// rotation, which is what collapsed the hand into the arm.
        /// </summary>
        public bool DivideParentFromHand;

        /// <summary>Reflect the palm frame about the vertical plane (left/right mirror).</summary>
        public bool MirrorPlaneYZ = true;
        /// <summary>Reflect about the horizontal plane (up/down).</summary>
        public bool MirrorPlaneXZ;
        /// <summary>Reflect about the front/back plane.</summary>
        public bool MirrorPlaneXY;

        /// <summary>
        /// Which measured palm axis plays which role, and which way it points.
        ///
        /// Neither the VMC specification nor Warudo's documentation says what frame a bone
        /// rotation is expected in -- the specification only says a quaternion is sent. A wrong
        /// guess does not look like a bug: it is a plausible rotation about the wrong axis,
        /// which is exactly what the user saw, with the palm turning in its own plane while the
        /// model's hand swept out of that plane. So these are settings, not decisions.
        /// </summary>
        /// <summary>Which measured axis fills each role: 0=fingers, 1=across, 2=normal.</summary>
        public int HandAxisX;        // role: along the fingers
        public int HandAxisY = 1;    // role: across the knuckles
        public int HandAxisZ = 2;    // role: out of the palm
        public float HandSignX = 1f;
        public float HandSignY = 1f;
        public float HandSignZ = 1f;
        public bool MirrorHandAxis;

        /// <summary>UDP port the tracker sends palm orientations to.</summary>
        public int HandFramePort = HandFrameReceiver.DefaultPort;

        /// <summary>Port for the colour relay. 0 picks a free one.</summary>
        public int RelayPort;

        /// <summary>
        /// Use Kinect's own joint orientations, which carry the rotation about each bone's
        /// own axis. Without them the palm cannot be oriented: a shortest-arc swing has no
        /// twist, so a hanging arm drags the hand round with the upper arm.
        /// </summary>
        public bool UseJointOrientations = true;

        /// <summary>How far the fingers fold for a fist, in degrees.</summary>
        public float FingerCurlDegrees = 80f;

        public string ToKeyValueText()
        {
            var lines = new List<string>
            {
                "TargetAddress=" + TargetAddress,
                "TargetPort=" + TargetPort.ToString(System.Globalization.CultureInfo.InvariantCulture),
                "Smoothing=" + Smoothing.ToString(System.Globalization.CultureInfo.InvariantCulture),
                "SendHead=" + (SendHead ? "1" : "0"),
                "SendRootPosition=" + (SendRootPosition ? "1" : "0"),
                "MovementScale=" + MovementScale.ToString(System.Globalization.CultureInfo.InvariantCulture),
                "MovementLimit=" + MovementLimit.ToString(System.Globalization.CultureInfo.InvariantCulture),
                "Mirror=" + (Mirror ? "1" : "0"),
                "KinectTiltDegrees=" + KinectTiltDegrees.ToString(System.Globalization.CultureInfo.InvariantCulture),
                "HangingArmReference=" + (HangingArmReference ? "1" : "0"),
                "SendFullBody=" + (SendFullBody ? "1" : "0"),
                "SendRootRotation=" + (SendRootRotation ? "1" : "0"),
                "FingerCurlInward=" + (FingerCurlInward ? "1" : "0"),
                "EmitWorldRotations=" + (EmitWorldRotations ? "1" : "0"),
                "LogHandMetrics=" + (LogHandMetrics ? "1" : "0"),
                "EmitFingers=" + (EmitFingers ? "1" : "0"),
                "SendHand=" + (SendHand ? "1" : "0"),
                "SendSkeletonToPlugin=" + (SendSkeletonToPlugin ? "1" : "0"),
                "PluginPort=" + PluginPort.ToString(System.Globalization.CultureInfo.InvariantCulture),
                "UseJointOrientations=" + (UseJointOrientations ? "1" : "0"),
                "RelayColour=" + (RelayColour ? "1" : "0"),
                "RelayPort=" + RelayPort.ToString(System.Globalization.CultureInfo.InvariantCulture),
                "UseCameraHand=" + (UseCameraHand ? "1" : "0"),
                "ForearmTwistShare=" + ForearmTwistShare.ToString(System.Globalization.CultureInfo.InvariantCulture),
                "AbsoluteHand=" + (AbsoluteHand ? "1" : "0"),
                "MirrorHandY=" + (MirrorHandY ? "1" : "0"),
                "DivideParentFromHand=" + (DivideParentFromHand ? "1" : "0"),
                "MirrorPlaneYZ=" + (MirrorPlaneYZ ? "1" : "0"),
                "MirrorPlaneXZ=" + (MirrorPlaneXZ ? "1" : "0"),
                "MirrorPlaneXY=" + (MirrorPlaneXY ? "1" : "0"),
                "HandAxisX=" + HandAxisX.ToString(System.Globalization.CultureInfo.InvariantCulture),
                "HandAxisY=" + HandAxisY.ToString(System.Globalization.CultureInfo.InvariantCulture),
                "HandAxisZ=" + HandAxisZ.ToString(System.Globalization.CultureInfo.InvariantCulture),
                "HandSignX=" + HandSignX.ToString(System.Globalization.CultureInfo.InvariantCulture),
                "HandSignY=" + HandSignY.ToString(System.Globalization.CultureInfo.InvariantCulture),
                "HandSignZ=" + HandSignZ.ToString(System.Globalization.CultureInfo.InvariantCulture),
                "MirrorHandAxis=" + (MirrorHandAxis ? "1" : "0"),
                "HandFramePort=" + HandFramePort.ToString(System.Globalization.CultureInfo.InvariantCulture),
                "FingerCurlDegrees=" + FingerCurlDegrees.ToString(System.Globalization.CultureInfo.InvariantCulture)
            };
            return string.Join(Environment.NewLine, lines.ToArray()) + Environment.NewLine;
        }

        public void LoadFromKeyValueText(string text)
        {
            if (string.IsNullOrEmpty(text)) { return; }

            string[] lines = text.Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
            foreach (string raw in lines)
            {
                string line = raw.Trim();
                if (line.Length == 0 || line[0] == '#') { continue; }

                int split = line.IndexOf('=');
                if (split <= 0) { continue; }

                string key = line.Substring(0, split).Trim();
                string value = line.Substring(split + 1).Trim();

                switch (key)
                {
                    case "TargetAddress": if (value.Length > 0) { TargetAddress = value; } break;
                    case "TargetPort": TargetPort = ParseInt(value, TargetPort); break;
                    case "Smoothing": Smoothing = ParseFloat(value, Smoothing); break;
                    case "SendHead": SendHead = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "SendRootPosition": SendRootPosition = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "MovementScale": MovementScale = ParseFloat(value, MovementScale); break;
                    case "MovementLimit": MovementLimit = ParseFloat(value, MovementLimit); break;
                    case "Mirror": Mirror = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "EmitFingers": EmitFingers = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "SendHand": SendHand = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "SendSkeletonToPlugin": SendSkeletonToPlugin = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "PluginPort": { int n; if (int.TryParse(value, out n)) { PluginPort = n; } break; }
                    case "UseJointOrientations": UseJointOrientations = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "RelayColour": RelayColour = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "RelayPort": { int p; if (int.TryParse(value, out p)) { RelayPort = p; } break; }
                    case "UseCameraHand": UseCameraHand = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "ForearmTwistShare": { float f; if (float.TryParse(value, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out f)) { ForearmTwistShare = f; } break; }
                    case "AbsoluteHand": AbsoluteHand = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "MirrorHandY": MirrorHandY = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "DivideParentFromHand": DivideParentFromHand = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "MirrorPlaneYZ": MirrorPlaneYZ = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "MirrorPlaneXZ": MirrorPlaneXZ = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "MirrorPlaneXY": MirrorPlaneXY = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "HandAxisX": { int n; if (int.TryParse(value, out n)) { HandAxisX = n; } break; }
                    case "HandAxisY": { int n; if (int.TryParse(value, out n)) { HandAxisY = n; } break; }
                    case "HandAxisZ": { int n; if (int.TryParse(value, out n)) { HandAxisZ = n; } break; }
                    case "HandSignX": { float f; if (float.TryParse(value, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out f)) { HandSignX = f; } break; }
                    case "HandSignY": { float f; if (float.TryParse(value, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out f)) { HandSignY = f; } break; }
                    case "HandSignZ": { float f; if (float.TryParse(value, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out f)) { HandSignZ = f; } break; }
                    case "MirrorHandAxis": MirrorHandAxis = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "HandFramePort": { int p; if (int.TryParse(value, out p)) { HandFramePort = p; } break; }
                    case "FingerCurlDegrees":
                        {
                            float curl;
                            if (float.TryParse(value, System.Globalization.NumberStyles.Float,
                                System.Globalization.CultureInfo.InvariantCulture, out curl) && !float.IsNaN(curl))
                            {
                                FingerCurlDegrees = curl < 0f ? 0f : (curl > 110f ? 110f : curl);
                            }
                        }
                        break;
                    case "LogHandMetrics": LogHandMetrics = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "EmitWorldRotations": EmitWorldRotations = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "SendFullBody": SendFullBody = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "SendRootRotation": SendRootRotation = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "FingerCurlInward": FingerCurlInward = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "HangingArmReference": HangingArmReference = value == "1" || value.Equals("true", StringComparison.OrdinalIgnoreCase); break;
                    case "KinectTiltDegrees":
                        {
                            float tilt;
                            if (float.TryParse(value, System.Globalization.NumberStyles.Float,
                                System.Globalization.CultureInfo.InvariantCulture, out tilt) && !float.IsNaN(tilt))
                            {
                                KinectTiltDegrees = tilt < -60f ? -60f : (tilt > 60f ? 60f : tilt);
                            }
                        }
                        break;
                }
            }

            Clamp();
        }

        public void Clamp()
        {
            if (TargetPort <= 0 || TargetPort > 65535) { TargetPort = 39539; }
            if (Smoothing < 0f) { Smoothing = 0f; }
            if (Smoothing > 0.95f) { Smoothing = 0.95f; }
            if (MovementScale < 0f) { MovementScale = 0f; }
            if (MovementScale > 3f) { MovementScale = 3f; }
            if (MovementLimit < 0f) { MovementLimit = 0f; }
            if (MovementLimit > 3f) { MovementLimit = 3f; }
        }

        private static int ParseInt(string value, int fallback)
        {
            int parsed;
            return int.TryParse(value, System.Globalization.NumberStyles.Integer,
                System.Globalization.CultureInfo.InvariantCulture, out parsed) ? parsed : fallback;
        }

        private static float ParseFloat(string value, float fallback)
        {
            float parsed;
            return float.TryParse(value, System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out parsed) ? parsed : fallback;
        }
    }

    /// <summary>What the tracking pipeline is currently doing, for the status display.</summary>
    public enum PipelineState
    {
        Stopped,
        WaitingForSensor,
        WaitingForBody,
        Tracking
    }

    /// <summary>
    /// The device layer the pipeline depends on, factored out so the pipeline can be driven
    /// from synthetic frames.
    ///
    /// The Kinect cannot be opened from a restricted process, so without this seam the whole
    /// of <see cref="TrackingPipeline"/> would be untestable outside a real desktop session.
    /// With it, every decision the pipeline makes (state transitions, calibration, root
    /// translation, mirroring, settings) is exercised by an automated test.
    /// </summary>
    public interface IBodySource : IDisposable
    {
        /// <summary>Opens the device if needed. Safe to call repeatedly.</summary>
        void EnsureOpen();

        /// <summary>True when the device reports itself usable.</summary>
        bool IsSensorAvailable { get; }

        /// <summary>Unique device id, or empty when unknown.</summary>
        string SensorId { get; }

        /// <summary>Number of bodies the device currently reports as tracked.</summary>
        int TrackedBodyCount { get; }

        /// <summary>Measured frame rate of the device layer.</summary>
        float FrameRate { get; }

        /// <summary>Human-readable device state.</summary>
        string StatusText { get; }

        /// <summary>Last device-layer error, or null.</summary>
        string LastError { get; }

        /// <summary>Copies the latest frame out. Returns false when no body is tracked.</summary>
        bool TryGetFrame(SkeletonFrameData destination);
    }

    /// <summary>
    /// Owns the 30 Hz loop: pull a body frame, solve the upper body, send VMC.
    ///
    /// Runs on its own thread so the UI stays responsive and so a stalled sensor cannot freeze
    /// the window.
    /// </summary>
    public sealed class TrackingPipeline : IDisposable
    {
        private readonly IBodySource _source;
        private readonly UpperBodySolver _solver = new UpperBodySolver();
        private readonly VmcSender _sender = new VmcSender();
        private readonly PluginBoneSender _pluginSender = new PluginBoneSender();
        private readonly SkeletonFrameData _frame = new SkeletonFrameData();
        private readonly SkeletonPose _pose = new SkeletonPose();

        private Thread _thread;
        private volatile bool _running;
        private volatile bool _stopRequested;
        private volatile bool _calibrationRequested;
        private volatile bool _zeroRequested;
        private System.IO.StreamWriter _handLog;

        private Vec3 _origin;
        private bool _hasOrigin;

        public AppSettings Settings { get; private set; }

        public TrackingPipeline(AppSettings settings)
            : this(settings, new KinectBodySource())
        {
        }

        /// <summary>
        /// The colour relay, when one is set up.
        ///
        /// The hand tracker needs camera images, and only this process can hold the Kinect. With
        /// <see cref="AppSettings.RelayColour"/> on, the Kinect's own colour stream is forwarded to
        /// it over loopback instead.
        /// </summary>
        public ColourRelay Relay { get; private set; }

        /// <summary>Port the hand tracker should connect to for images, or 0 when not relaying.</summary>
        public int RelayPort { get { return Relay != null ? Relay.Port : 0; } }

        /// <summary>
        /// Replaces the hand bone's rotation with one built from the camera's palm reading.
        ///
        /// The composition, and why it happens here
        /// ----------------------------------------
        /// Kinect gives the hand a DIRECTION from a 9 cm bone and cannot describe its roll, so a
        /// palm can never be made to face the right way from Kinect alone. The camera measures the
        /// palm directly, and it is the only source that can.
        ///
        /// Two modes, because the correct one cannot be settled by reasoning
        /// ---------------------------------------------------------------
        /// ABSOLUTE: the measured palm frame becomes the hand bone's rotation. Warudo's own
        /// MediaPipe tracker works this way -- it hands over a full array of bone rotations and the
        /// avatar's hand lines up with the user's immediately, with no calibration step. If the
        /// frames line up, this is the answer, because it makes the hand's pose correct on its own
        /// rather than merely tracking how it changes.
        ///
        /// RELATIVE: only the change since a reference is applied, on top of whatever Kinect
        /// produced. This can never make the hand match a pose it did not start in, which is
        /// exactly the complaint -- the model's hand sat in its default orientation and merely
        /// rotated when the user turned their palm.
        ///
        /// Both are kept because the difference between them is a question about frames that no
        /// amount of reading Warudo's files has answered, and one look at the avatar settles it.
        /// </summary>
        private void ApplyCameraHand()
        {
            if (!Settings.UseCameraHand || HandFrames == null) { return; }

            for (int i = 0; i < HandSides.Length; i++)
            {
                string side = HandSides[i];
                string bone = side == "Left" ? VmcBones.LeftHand : VmcBones.RightHand;

                Quat? measuredNullable = HandFrames.GetOrientation(side);
                if (measuredNullable == null) { continue; }
                Quat measured = measuredNullable.Value;

                if (Settings.AbsoluteHand)
                {
                    // The measured palm frame is a WORLD orientation, but a bone rotation is LOCAL,
                    // so the parent's orientation has to be divided out.
                    //
                    // This is the one thing about the protocol that is actually documented -- in
                    // VirtualMotionCapture's own source, which is the reference implementation:
                    //
                    //     【送信側は変換不要】
                    //     送信は Vrm10Instance.Humanoid.GetBoneTransform をそのまま読めば
                    //     オリジナル姿勢が得られる。
                    //
                    // i.e. the sender writes `boneTransform.localRotation` and nothing else. Unity's
                    // localRotation is relative to the PARENT'S CURRENT orientation -- not to the
                    // bind pose, which is a receiver-side concern.
                    //
                    // Sending the world orientation directly was the reason the pose looked right
                    // while standing still and went wrong the moment the arm moved: with the arm
                    // near its reference the parent is close to identity and the error is small, and
                    // any arm movement exposes it and drags the hand along.
                    // Dividing the parent out is what the protocol implies, but there is a catch
                    // that makes it actively harmful when the arm is not trustworthy: the result is
                    // only as good as the parent. If Kinect's forearm rotation is wrong -- and it is,
                    // being derived from a 9 cm bone -- then subtracting it from a CORRECT camera
                    // measurement produces a WRONG local rotation, and the hand is dragged wherever
                    // the bad arm points. That is the hand-collapsing-into-the-arm regression.
                    //
                    // So it is a switch: with it on, the camera's accuracy is sacrificed to the
                    // arm's inaccuracy; with it off, the hand keeps the camera's orientation and the
                    // arm's error shows up instead as a mismatch between hand and forearm.
                    if (Settings.DivideParentFromHand)
                    {
                        Quat absoluteParent;
                        if (!TryGetChainWorldRotation(bone, out absoluteParent)) { continue; }

                        Quat desiredWorld = ApplyHandFrameFixes(measured, side);
                        _pose.BoneRotations[bone] = absoluteParent.Conjugate() * desiredWorld;
                    }
                    else
                    {
                        _pose.BoneRotations[bone] = ApplyHandFrameFixes(measured, side);
                    }

                    continue;
                }

                Quat reference;
                if (!_calibratedPalm.TryGetValue(side, out reference)) { continue; }

                // How far the palm has turned since the reference was taken.
                Quat delta = measured * reference.Conjugate();

                // Split the turn between the forearm and the hand.
                //
                // A real wrist cannot twist far on its own: most of a palm turn comes from the
                // forearm rotating along its own axis, and the hand contributes the rest. Putting
                // the WHOLE correction on the hand bone asks one joint to do the work of two, which
                // is why the wrist end deformed instead of the palm turning -- the twist was all
                // piled onto the last bone in the chain.
                //
                // The side of the split is a setting because how much a given rig tolerates at the
                // wrist varies; 0.6 forearm is the human proportion and the starting point.
                Quat handDelta, forearmDelta;
                SplitTwist(delta, Settings.ForearmTwistShare, out forearmDelta, out handDelta);

                // Apply to the forearm first, then rebuild the hand's parent frame from the result,
                // so the hand's own correction is measured against the arm as it now stands.
                string forearmBone = side == "Left" ? VmcBones.LeftLowerArm : VmcBones.RightLowerArm;
                if (forearmDelta.W != 1.0f || forearmDelta.X != 0f
                    || forearmDelta.Y != 0f || forearmDelta.Z != 0f)
                {
                    Quat forearmParent;
                    Quat forearmExisting;
                    if (TryGetChainWorldRotation(forearmBone, out forearmParent)
                        && _pose.BoneRotations.TryGetValue(forearmBone, out forearmExisting))
                    {
                        _pose.BoneRotations[forearmBone] =
                            (forearmParent.Conjugate() * forearmDelta * forearmParent) * forearmExisting;
                    }
                }

                // The hand bone is a LOCAL rotation, so a world-space delta cannot be dropped in
                // as-is: it has to be expressed in the parent's frame first.
                Quat parentWorld;
                if (!TryGetChainWorldRotation(bone, out parentWorld)) { continue; }

                // POST-multiplied onto the bone's existing rotation, not used in place of it.
                //
                // Kinect already decided which way the hand POINTS, from the wrist-to-fingertip
                // direction, and that part is sound. The camera only adds the roll that Kinect
                // cannot see. Replacing the bone outright threw the direction away as well, which
                // showed up as the whole forearm appearing to twist half a turn.
                Quat existing;
                if (!_pose.BoneRotations.TryGetValue(bone, out existing)) { existing = Quat.Identity; }

                _pose.BoneRotations[bone] = (parentWorld.Conjugate() * handDelta * parentWorld) * existing;
            }
        }

        /// <summary>
        /// Splits a rotation into a part about the limb's own axis and the remainder.
        ///
        /// The axis is taken to be the bone's local X -- the direction the chain runs in -- which is
        /// where pronation and supination actually happen. Anything not about that axis stays with
        /// the hand, because it is a bend rather than a twist and the wrist is the joint that does
        /// bending.
        /// </summary>
        private static void SplitTwist(Quat delta, float forearmShare,
            out Quat forearmDelta, out Quat handDelta)
        {
            forearmDelta = Quat.Identity;
            handDelta = delta;

            if (forearmShare <= 0f || forearmShare > 1f) { return; }

            double half = 2.0 * Math.Acos(Math.Min(1.0, Math.Abs(delta.W)));
            if (half < 1e-4) { return; }

            // The rotation's own axis, and how much of it lies along the limb.
            double axisLength = Math.Sqrt(
                (double)delta.X * delta.X + (double)delta.Y * delta.Y + (double)delta.Z * delta.Z);
            if (axisLength < 1e-6) { return; }

            double along = delta.X / axisLength;       // local X is the bone's own direction
            if (Math.Abs(along) < 0.5) { return; }     // mostly a bend, not a twist: leave it on the hand

            // Signed fraction of the total angle that is a twist about the limb.
            double twistAngle = half * along;
            if (Math.Abs(twistAngle) < 1e-4) { return; }

            var axis = new Vec3(along > 0 ? 1f : -1f, 0f, 0f);
            forearmDelta = Quat.AxisAngle(axis, (float)(twistAngle * forearmShare));
            handDelta = delta * forearmDelta.Conjugate();
        }

        /// <summary>
        /// Flip a rotation about the character's vertical plane.
        ///
        /// The camera's image arrives from the Kinect with Z negated to reach character space,
        /// but MediaPipe's own world frame has its Y pointing DOWN the image. Whether that
        /// needs undoing for the palm's frame is a question about two conventions meeting,
        /// and a sign error here is a plausible-looking rotation rather than an obvious one.
        /// Exposed as a switch so it can be settled by looking at the avatar.
        /// </summary>
        private static Quat MirrorY(Quat q)
        {
            // Conjugating by a mirror through the XZ plane negates X and Z.
            return new Quat(-q.X, q.Y, -q.Z, q.W);
        }

        /// <summary>
        /// Applies the last step of the frame conversion: which way each of the palm's axes points.
        ///
        /// The measured palm frame is built from landmarks, and the ORDER and SIGNS of its axes are
        /// a convention -- one that had to be guessed, because neither the VMC specification nor
        /// Warudo's documentation states what frame a bone rotation is expected in. The
        /// specification simply says a quaternion is sent.
        ///
        /// A wrong guess here does not look like a bug. It produces a perfectly plausible rotation
        /// that happens to be about the wrong axis, which is what the user saw: turning their palm
        /// in its own plane made the model's hand sweep out of that plane instead. Swapping two axes
        /// or flipping one sign produces exactly that, so they are settings rather than decisions.
        /// </summary>
        private Quat ApplyHandFrameFixes(Quat measured, string side)
        {
            Quat q = measured;

            // Which measured axis plays which role, chosen per axis.
            //
            // A fixed list of permutations was not enough: the one the data called for -- fingers
            // from the palm NORMAL, across from the fingers, normal from the across axis -- was not
            // in it, so no option could be correct. The measured axes are named forward (fingers),
            // side (across the knuckles) and normal (out of the palm), and each output slot picks
            // one of the three.
            q = new Quat(
                ComponentOf(q, Settings.HandAxisX),
                ComponentOf(q, Settings.HandAxisY),
                ComponentOf(q, Settings.HandAxisZ),
                q.W);

            // Per-axis sign, applied after the permutation.
            q = new Quat(
                q.X * Settings.HandSignX,
                q.Y * Settings.HandSignY,
                q.Z * Settings.HandSignZ,
                q.W);

            // Reflections, one per coordinate plane.
            //
            // A reflection is NOT an axis swap and cannot be expressed as one. Mirrored across a
            // plane, a rotation negates exactly the two components lying in that plane, so no
            // combination of the swaps offered earlier could ever produce it -- which is why every
            // permutation was tried without success.
            //
            // The user faces the avatar, so their left hand drives the model's right: that is a
            // reflection about the VERTICAL plane, and it was the one case missing. It is on by
            // default because facing the avatar is the normal way to stream.
            if (Settings.MirrorPlaneYZ) { q = new Quat(q.X, -q.Y, -q.Z, q.W); }
            if (Settings.MirrorPlaneXZ) { q = new Quat(-q.X, q.Y, -q.Z, q.W); }
            if (Settings.MirrorPlaneXY) { q = new Quat(-q.X, -q.Y, q.Z, q.W); }

            // The upper/lower mirror, which the user found changed the hand from pointing down to
            // pointing up. It was declared, saved, loaded and read from the settings file, but never
            // APPLIED anywhere -- so ticking the box did nothing at all. The same silent-replacement
            // mistake has now cost this project three switches; every one of them is checked for a
            // call site before being called done.
            if (Settings.MirrorHandY) { q = MirrorY(q); }

            // A mirrored rig needs one hand's frame reflected; the flip is its own inverse, so the
            // same operation is correct whichever convention the rig uses.
            if (Settings.MirrorHandAxis && side == "Right") { q = MirrorY(q); }

            return q;
        }

        /// <summary>
        /// One component of a quaternion's vector part, chosen by measured-axis index.
        ///
        /// 0 = forward (along the fingers), 1 = side (across the knuckles), 2 = normal (out of the
        /// palm). Naming the axes individually is what makes a wrong choice visible: the earlier
        /// fixed list of permutations hid the fact that the combination the measurements called for
        /// was not in it at all, so no option could ever be right.
        /// </summary>
        private static float ComponentOf(Quat q, int axis)
        {
            switch (axis)
            {
                case 0: return q.X;
                case 1: return q.Y;
                default: return q.Z;
            }
        }

        /// <summary>Rebuilds a quaternion with its axis components reordered.</summary>
        private static Quat Permute(Quat q, int i, int j, int k)
        {
            float[] v = { q.X, q.Y, q.Z };
            return new Quat(v[i], v[j], v[k], q.W);
        }

        private static readonly string[] HandSides = { "Left", "Right" };

        /// <summary>
        /// The accumulated world rotation of a bone's ancestors, up to the root.
        ///
        /// The FULL chain matters, not just the arm. A bone rotation is relative to its parent, so
        /// anything left out of this product is a rotation that never gets divided out -- and the
        /// spine bones are not negligible: leaning or turning moves them, and the hand would then be
        /// short by exactly that amount.
        /// </summary>
        private bool TryGetChainWorldRotation(string handBone, out Quat world)
        {
            world = Quat.Identity;

            bool left = handBone == VmcBones.LeftHand;
            string forearmName = left ? VmcBones.LeftLowerArm : VmcBones.RightLowerArm;
            string upperName = left ? VmcBones.LeftUpperArm : VmcBones.RightUpperArm;
            string shoulderName = left ? VmcBones.LeftShoulder : VmcBones.RightShoulder;

            // Anything absent contributes nothing, so a missing torso bone is not fatal -- but a
            // missing arm bone is, because then there is no meaningful parent to divide out.
            Quat hips, spine, chest, upperChest, shoulder, upper, forearm;
            if (!_pose.BoneRotations.TryGetValue(VmcBones.Hips, out hips)) { hips = Quat.Identity; }
            if (!_pose.BoneRotations.TryGetValue(VmcBones.Spine, out spine)) { spine = Quat.Identity; }
            if (!_pose.BoneRotations.TryGetValue(VmcBones.Chest, out chest)) { chest = Quat.Identity; }
            if (!_pose.BoneRotations.TryGetValue(VmcBones.UpperChest, out upperChest)) { upperChest = Quat.Identity; }

            if (!_pose.BoneRotations.TryGetValue(shoulderName, out shoulder)) { return false; }
            if (!_pose.BoneRotations.TryGetValue(upperName, out upper)) { return false; }
            if (!_pose.BoneRotations.TryGetValue(forearmName, out forearm)) { return false; }

            world = hips * spine * chest * upperChest * shoulder * upper * forearm;
            return world.IsFinite();
        }

        /// <summary>
        /// Takes the palm reference automatically, a moment after the tracker starts reporting.
        ///
        /// It used to require pressing the calibrate button, which turned out to be unusable in
        /// practice: it needs a free hand for a wired mouse, the arm has to be somewhere it can
        /// actually be, and the camera has to see the hand at that exact instant. A reference that
        /// can be taken automatically should be.
        ///
        /// Settling for a moment first, rather than using the very first packet, matters because the
        /// tracker's first frames are its least steady -- and a reference taken from a noisy frame
        /// offsets every correction that follows.
        /// </summary>
        private void AutoCapturePalmReferenceIfNeeded()
        {
            if (HandFrames == null) { return; }

            // Already have both, or already had a go.
            if (_calibratedPalm.Count >= HandSides.Length || _palmReferenceAttempted) { return; }

            if (!_palmReferencePending)
            {
                // Wait until the tracker is actually reporting something.
                for (int i = 0; i < HandSides.Length; i++)
                {
                    if (HandFrames.GetOrientation(HandSides[i]) != null)
                    {
                        _palmReferencePending = true;
                        _palmReferenceDue = Now() + PalmReferenceDelaySeconds;
                        break;
                    }
                }
                return;
            }

            if (Now() < _palmReferenceDue) { return; }

            _palmReferenceAttempted = true;
            CapturePalmReference();

            if (_calibratedPalm.Count > 0)
            {
                PalmReferenceStatus = "掌心基准: 已自动记录 " + _calibratedPalm.Count + " 只手";
            }
            else
            {
                PalmReferenceStatus = "掌心基准: 自动记录失败（手掌不在画面里？）";
            }
        }

        private void CapturePalmReference()
        {
            _calibratedPalm.Clear();
            if (HandFrames == null) { return; }

            for (int i = 0; i < HandSides.Length; i++)
            {
                Quat? current = HandFrames.GetOrientation(HandSides[i]);
                if (current != null) { _calibratedPalm[HandSides[i]] = current.Value; }
            }
        }

        /// <summary>Seconds to wait after the tracker starts reporting before taking the reference.</summary>
        public const double PalmReferenceDelaySeconds = 1.5;

        private bool _palmReferencePending;
        private bool _palmReferenceAttempted;
        private double _palmReferenceDue;

        /// <summary>What the palm reference is doing, for the UI to show.</summary>
        public string PalmReferenceStatus = "掌心基准: 等待手部识别";

        private static double Now()
        {
            return (double)Environment.TickCount / 1000.0;
        }

        private void CapturePalmReferenceForCalibration()
        {
            // Re-taking it on an explicit calibration is still allowed, and is the way to shift the
            // zero point once the automatic one has been established.
            _calibratedPalm.Clear();
            if (HandFrames == null) { return; }

            for (int i = 0; i < HandSides.Length; i++)
            {
                Quat? current = HandFrames.GetOrientation(HandSides[i]);
                if (current != null) { _calibratedPalm[HandSides[i]] = current.Value; }
            }
        }

        /// <summary>Receives the palm orientations the tracker measures. Null when not in use.</summary>
        public HandFrameReceiver HandFrames { get; private set; }

        /// <summary>
        /// Brings the palm-orientation receiver up or down to match the settings.
        ///
        /// The palm arrives in the CAMERA's frame, which is not the frame a bone rotation lives in.
        /// Rather than converting between them through a chain of assumptions, the reading taken at
        /// calibration is kept as the reference and only the CHANGE since then is applied. At the
        /// moment of calibration the correction is therefore zero by construction, whatever the
        /// user's arm happened to be doing -- the same trick that made the forearm twist behave.
        /// </summary>
        /// <summary>
        /// 手部朝向不再由本程序处理。
        ///
        /// 掌心朝向现在由 Warudo 里的插件直接写手骨（它读得到实时父级朝向，也不受 VMC 语义
        /// 限制）。本程序的 HandFrameReceiver 必须让出 39542 —— 那个端口现在归插件，
        /// 两边同时监听只会让插件绑定失败。
        ///
        /// 保留这个方法（而不是删掉）是因为调用点还在；它现在只负责确保监听器已关闭。
        /// </summary>
        private void EnsureHandFramesMatchSettings()
        {
            if (HandFrames != null)
            {
                try { HandFrames.Dispose(); } catch { }
                HandFrames = null;
            }
        }

        /// <summary>
        /// Brings the colour relay up or down to match the settings.
        ///
        /// Kept separate from the tracking loop's body work so a listener that fails to bind (the
        /// port being taken, say) reports once and then leaves tracking alone rather than throwing
        /// every frame.
        /// </summary>
        private void EnsureRelayMatchesSettings()
        {
            if (Settings.RelayColour)
            {
                var concrete = _source as KinectBodySource;
                if (concrete == null)
                {
                    // A synthetic source in the tests has no colour stream to forward.
                    return;
                }

                if (Relay == null)
                {
                    var relay = new ColourRelay();
                    if (!relay.Start(Settings.RelayPort))
                    {
                        LastError = "彩色中继无法监听端口 " + Settings.RelayPort;
                        relay.Dispose();
                        return;
                    }

                    Relay = relay;
                    concrete.ColourSink = relay.Send;
                    concrete.RelayColour = true;

                    // Force a reopen so the reader picks up the colour stream.
                    concrete.Release();
                }
            }
            else if (Relay != null)
            {
                var concrete = _source as KinectBodySource;
                if (concrete != null)
                {
                    concrete.ColourSink = null;
                    concrete.RelayColour = false;
                    concrete.Release();
                }

                Relay.Dispose();
                Relay = null;
            }
        }

        /// <summary>
        /// Builds a pipeline over an explicit device layer. Used by the tests, which drive it
        /// with synthetic frames instead of a sensor.
        /// </summary>
        public TrackingPipeline(AppSettings settings, IBodySource source)
        {
            if (settings == null) { throw new ArgumentNullException("settings"); }
            if (source == null) { throw new ArgumentNullException("source"); }

            Settings = settings;
            _source = source;
            ApplySettings();
        }

        // ------------------------------------------------------------------ state exposed to the UI

        public PipelineState State { get; private set; }

        /// <summary>True while the worker thread is alive. Safe to read from the UI thread.</summary>
        public bool IsRunning { get { return _running && _thread != null; } }

        public string StatusText { get; private set; }
        public string LastError { get; private set; }
        public bool SensorAvailable { get; private set; }
        public string SensorId { get; private set; }
        public int TrackedBodyCount { get; private set; }
        public float FrameRate { get; private set; }
        public long PacketsSent { get; private set; }
        public bool IsCalibrated { get; private set; }

        private bool _orientationReported;

        /// <summary>Palm orientation captured at calibration, per side.</summary>
        private readonly System.Collections.Generic.Dictionary<string, Quat> _calibratedPalm =
            new System.Collections.Generic.Dictionary<string, Quat>(StringComparer.Ordinal);
        private bool _turnReported;
        public int BoneCount { get; private set; }

        /// <summary>Latest solved bone rotations, for the debug readout. Replaced, never mutated.</summary>
        public Dictionary<string, Quat> LatestRotations { get; private set; }

        /// <summary>
        /// Latest measured joint positions, for the skeleton preview.
        ///
        /// A single buffer is reused and refilled rather than a fresh array being cloned out of
        /// the frame every tick: this is on the 30 Hz hot path, and the UI only ever reads the
        /// current contents.
        /// </summary>
        public Vec3[] LatestJoints { get; private set; }

        // ------------------------------------------------------------------ control

        public void ApplySettings()
        {
            Settings.Clamp();
            _sender.Configure(Settings.TargetAddress, Settings.TargetPort);
            _pluginSender.Configure("127.0.0.1", Settings.PluginPort);
            _pluginSender.SendRootPosition = Settings.SendRootPosition;
            _sender.SendRootPosition = Settings.SendRootPosition;
            _solver.Smoothing = Settings.Smoothing;
            _solver.SendHead = Settings.SendHead;
            _solver.EmitLocalRotations = !Settings.EmitWorldRotations;
            _solver.EmitFingers = Settings.EmitFingers;
            _solver.SendHand = Settings.SendHand;
            _solver.UseJointOrientations = Settings.UseJointOrientations;
            _solver.SendFullBody = Settings.SendFullBody;
            _solver.SendRootRotation = Settings.SendRootRotation;
            _solver.FingerCurlInward = Settings.FingerCurlInward;
            _solver.FingerCurlDegrees = Settings.FingerCurlDegrees;
            _solver.ArmReference = Settings.HangingArmReference
                ? UpperBodySolver.ArmReferencePose.HangingA
                : UpperBodySolver.ArmReferencePose.HorizontalT;
        }

        public void Start()
        {
            if (_running) { return; }

            _stopRequested = false;
            _running = true;
            _thread = new Thread(Loop);
            _thread.IsBackground = true;
            _thread.Name = "Kinect2VMC tracking";
            _thread.Start();
        }

        public void Stop()
        {
            _stopRequested = true;
            _running = false;

            Thread thread = _thread;
            _thread = null;
            if (thread != null)
            {
                try { thread.Join(1500); } catch { }
            }

            State = PipelineState.Stopped;
            StatusText = "已停止";
        }

        /// <summary>Requests that the next tracked frame becomes the neutral pose.</summary>
        public void RequestCalibration()
        {
            _calibrationRequested = true;
        }

        /// <summary>
        /// Requests that the avatar's current position becomes the movement origin, WITHOUT
        /// touching the solver's reference directions.
        ///
        /// Calibration and the movement origin used to be one action, which forced a choice: the
        /// neutral directions had to come from whatever pose the user happened to be sitting in
        /// when they zeroed the position. Decoupling them lets the position be zeroed while the
        /// rotation reference stays at the built-in horizontal T-pose -- the reference the arm
        /// motions actually orbit around.
        /// </summary>
        public void RequestZeroPosition()
        {
            _zeroRequested = true;
        }

        /// <summary>True when the solver is using captured neutral directions instead of the T-pose defaults.</summary>
        public bool UsesCapturedNeutral { get { return _solver.IsCalibrated; } }

        /// <summary>Coarse hand pose for the left hand as of the most recent solved frame.</summary>
        public HandMetrics LeftHand { get { return _solver.LeftHand; } }

        /// <summary>Coarse hand pose for the right hand as of the most recent solved frame.</summary>
        public HandMetrics RightHand { get { return _solver.RightHand; } }

        /// <summary>
        /// Drops the captured neutral directions and returns to the built-in horizontal T-pose
        /// reference. The movement origin is deliberately kept: the two are separate concerns.
        /// </summary>
        public void ClearCalibration()
        {
            _solver.ClearCalibration();
            _calibratedPalm.Clear();
            IsCalibrated = false;
            LatestRotations = new Dictionary<string, Quat>(StringComparer.Ordinal);
        }

        /// <summary>
        /// Writes one line of hand metrics per solved frame while logging is enabled, so the
        /// usable range of Kinect's inferred hand joints can be measured on the real sensor
        /// instead of guessed. The file lands beside the executable.
        /// </summary>
        private void LogHandFrame()
        {
            if (!Settings.LogHandMetrics)
            {
                if (_handLog != null) { _handLog.Dispose(); _handLog = null; }
                return;
            }

            if (_handLog == null)
            {
                try
                {
                    string path = System.IO.Path.Combine(
                        System.IO.Path.GetDirectoryName(typeof(TrackingPipeline).Assembly.Location),
                        "hand-log.csv");
                    bool fresh = !System.IO.File.Exists(path);
                    _handLog = new System.IO.StreamWriter(path, true);
                    if (fresh)
                    {
                        // Distances are in METRES. The valid flag says whether the fingertip and
                        // thumb joints were actually reported that frame, so a dropped hand can
                        // never be analysed as though it were a fist.
                        _handLog.WriteLine("L_tipToThumbM,L_tipToWristM,L_openness,L_pose,L_valid,"
                            + "R_tipToThumbM,R_tipToWristM,R_openness,R_pose,R_valid");
                    }
                }
                catch
                {
                    Settings.LogHandMetrics = false;
                    return;
                }
            }

            HandMetrics l = _solver.LeftHand;
            HandMetrics r = _solver.RightHand;
            _handLog.WriteLine(string.Join(",",
                F(l.TipToThumb), F(l.TipToWrist), F(l.Openness), l.Pose.ToString(), B(l.Valid),
                F(r.TipToThumb), F(r.TipToWrist), F(r.Openness), r.Pose.ToString(), B(r.Valid)));
        }

        private static string B(bool value) { return value ? "1" : "0"; }

        private static string F(float value)
        {
            return value.ToString("F4", System.Globalization.CultureInfo.InvariantCulture);
        }

        public void Dispose()
        {
            Stop();
            if (_handLog != null) { try { _handLog.Dispose(); } catch { } _handLog = null; }
            _sender.Dispose();
            _pluginSender.Dispose();

            // The relay and the palm receiver each own a socket and a thread. Leaving them running
            // would hold their ports and keep this process alive after the window has closed.
            if (Relay != null) { try { Relay.Dispose(); } catch { } Relay = null; }
            if (HandFrames != null) { try { HandFrames.Dispose(); } catch { } HandFrames = null; }

            _source.Dispose();
        }

        // ------------------------------------------------------------------ the loop

        private void Loop()
        {
            while (!_stopRequested)
            {
                try
                {
                    Tick();
                }
                catch (Exception ex)
                {
                    LastError = ex.Message;
                    StatusText = "内部错误：" + ex.Message;
                }

                // 30 Hz target. Kinect v2 body frames arrive at 30 Hz; sleeping a little less
                // keeps latency low without spinning.
                Thread.Sleep(8);
            }
        }

        /// <summary>
        /// Runs one iteration of the pipeline synchronously.
        ///
        /// Exposed so tests can advance the pipeline frame by frame with deterministic input;
        /// the worker thread simply calls this in a loop.
        /// </summary>
        public void Tick()
        {
            // Start or stop the colour relay to match the setting, before the sensor is touched so
            // the frame reader can ask for the colour stream on the very first open.
            EnsureRelayMatchesSettings();
            EnsureHandFramesMatchSettings();

            _source.EnsureOpen();

            SensorAvailable = _source.IsSensorAvailable;
            SensorId = _source.SensorId;
            TrackedBodyCount = _source.TrackedBodyCount;
            FrameRate = _source.FrameRate;
            LastError = _source.LastError;

            if (!SensorAvailable)
            {
                State = PipelineState.WaitingForSensor;
                StatusText = _source.StatusText;
                return;
            }

            // Trust the return value alone. A device layer that reports "no body" but leaves a
            // stale BodyTracked flag on the destination would otherwise be solved from the
            // previous frame's joints, which the user would see as the avatar freezing in a
            // pose it no longer holds.
            if (!_source.TryGetFrame(_frame))
            {
                State = PipelineState.WaitingForBody;
                StatusText = "等待检测到人体（坐到 Kinect 正前方）";
                BoneCount = 0;
                return;
            }

            _frame.BodyTracked = true;

            if (Settings.Mirror)
            {
                MirrorInPlace(_frame);
            }

            // Compensate for a Kinect mounted above the user and angled down. Applied BEFORE the
            // solver runs, so every bone -- including the spine, which is what makes an upright
            // sitter look like they are leaning forward -- is measured from the corrected pose.
            if (Settings.KinectTiltDegrees != 0f)
            {
                TiltInPlace(_frame, Settings.KinectTiltDegrees);
            }

            if (_calibrationRequested)
            {
                _calibrationRequested = false;
                if (_solver.Calibrate(_frame))
                {
                    _origin = _frame.Get(KinectJoint.SpineBase);
                    _hasOrigin = true;
                    IsCalibrated = true;

                    // Record the palm's orientation as the user holds it right now. Everything the
                    // camera contributes afterwards is measured as a CHANGE from this, which is what
                    // makes the correction zero in the calibrated pose -- no assumption about where
                    // the camera is or how the rig is bound has to be right for that to hold.
                    CapturePalmReferenceForCalibration();
                    PalmReferenceStatus = "掌心基准: 手动重置为当前手掌朝向";
                }
            }

            if (_zeroRequested)
            {
                // Movement origin only: leave the solver's reference directions alone.
                _zeroRequested = false;
                _origin = _frame.Get(KinectJoint.SpineBase);
                _hasOrigin = true;
            }

            // The pose object is reused across frames, so it must be reset before each solve.
            // Without this, a bone that stops being emitted (for example when the head toggle
            // is switched off, or a frame is rejected) would keep its previous rotation
            // forever and quietly keep being transmitted.
            _pose.BoneRotations.Clear();

            // Report once whether Kinect is actually supplying joint orientations.
            //
            // Everything about the palm's roll depends on this data existing. If the sensor or the
            // runtime does not provide it, the symptom is identical to the solver ignoring it, so
            // saying so plainly beats another round of guessing.
            if (!_orientationReported)
            {
                _orientationReported = true;
                bool orientated = _frame.IsOriented(KinectJoint.ElbowLeft)
                    || _frame.IsOriented(KinectJoint.ElbowRight);
                Console.WriteLine(orientated
                    ? "关节朝向: 可用 (Kinect 正在提供 JointOrientations)"
                    : "关节朝向: 不可用 (Kinect 未提供，掌心方向无法修正)");
            }
            if (!_solver.Solve(_frame, _pose))
            {
                // Nothing usable this frame: stay silent rather than transmitting a partial
                // pose, so the receiver keeps the last good frame.
                State = PipelineState.WaitingForBody;
                StatusText = "躯干未稳定跟踪，等待中";
                BoneCount = 0;
                return;
            }

            // Root translation: the avatar follows the body, but only by how far it moved from
            // the calibrated neutral position, scaled and clamped so it cannot wander off screen.
            if (_hasOrigin && Settings.SendRootPosition)
            {
                Vec3 delta = (_pose.RootPosition - _origin) * Settings.MovementScale;
                delta = delta.ClampedMagnitude(Settings.MovementLimit);
                _pose.RootPosition = delta;
                _pose.HasRootPosition = true;
            }
            else if (!Settings.SendRootPosition)
            {
                _pose.HasRootPosition = false;
            }

            // Compose the camera's palm reading with Kinect's arm.
            AutoCapturePalmReferenceIfNeeded();
            ApplyCameraHand();

            // Report the turn state once, the same way the orientation availability is reported.            //
            // Turning depends on three separate things agreeing -- the setting, the root-position
            // switch it is gated behind, and a captured shoulder line -- and when it stops working
            // the symptom is identical whichever one is at fault. Printing the combination means
            // the next report says which, instead of another round of guessing.
            if (!_turnReported && _pose.HasRootRotation)
            {
                _turnReported = true;
                double yawDegrees = 2.0 * Math.Acos(Math.Min(1.0, Math.Abs(_pose.RootRotation.W)))
                    * 57.29577951308232;
                // 把基准肩线的朝向一起打出来：它如果在 ±90° 附近，说明基准是在你侧着身
                // 的时候建立的，之后"正对镜头"就会读出偏差 —— 那一眼就能看出来。
                Console.WriteLine("转身: 开启 (当前偏航 " + yawDegrees.ToString("F1") + " 度"
                    + "，基准肩线朝向 " + _solver.DiagnosticHeadingReferenceDegrees.ToString("F1")
                    + " 度[相机系，正对镜头应接近 0 或 180])");
            }
            else if (!_turnReported && !Settings.SendRootRotation)
            {
                _turnReported = true;
                Console.WriteLine("转身: 关闭 (界面上的「转身」没有勾选)");
            }
            else if (!_turnReported && !Settings.SendRootPosition)
            {
                _turnReported = true;
                Console.WriteLine("转身: 未发送 (/VMC/Ext/Root/Pos 依赖界面上的「位移」勾选，"
                    + "因为偏航和位移在同一条消息里)");
            }
            else
            {
                // Not calibrated yet: hold the avatar still rather than snapping it to the
                // raw sensor position, which would drop it half a metre on the first frame.
                _pose.RootPosition = Vec3.Zero;
                _pose.HasRootPosition = true;
            }

            _pose.Time = _frame.Time;
            _pose.Tracking = true;

            _sender.Send(_pose);

            // 两条路可以同时开着对照；验证通过后再关掉 VMC 那一条。
            if (Settings.SendSkeletonToPlugin)
            {
                _pluginSender.Send(_pose);
            }

            PacketsSent = _sender.PacketsSent;
            BoneCount = _solver.LastBoneCount;
            LogHandFrame();
            LatestRotations = new Dictionary<string, Quat>(_pose.BoneRotations, StringComparer.Ordinal);

            // Refill the reused joint buffer instead of allocating a new array every tick.
            if (LatestJoints == null) { LatestJoints = new Vec3[SkeletonFrameData.JointCount]; }
            for (int i = 0; i < SkeletonFrameData.JointCount; i++)
            {
                LatestJoints[i] = _frame.Positions[i];
            }

            State = PipelineState.Tracking;
            StatusText = "追踪中";
        }

        /// <summary>Negates X on every joint, for a camera placed behind the user.</summary>
        /// <summary>
        /// Rotates every joint of the frame about the character's right axis (+X) by the given
        /// pitch, in degrees, around the pelvis.
        ///
        /// Positive tilts the skeleton BACKWARDS. A Kinect mounted above the user looks down at
        /// them, which its own coordinate frame reports as the user's vertical axis leaning away
        /// from the sensor; an upright sitter therefore arrives looking like they lean forward,
        /// and the avatar copies it. This cancels that.
        /// </summary>
        private static void TiltInPlace(SkeletonFrameData frame, float degrees)
        {
            const float DegreesToRadians = 0.0174532925f;
            Quat pitch = Quat.AxisAngle(new Vec3(1f, 0f, 0f), degrees * DegreesToRadians);
            Vec3 pivot = frame.Get(KinectJoint.SpineBase);

            for (int i = 0; i < SkeletonFrameData.JointCount; i++)
            {
                if (!frame.Tracked[i]) { continue; }
                frame.Positions[i] = pivot + (pitch * (frame.Positions[i] - pivot));
            }
        }
        private static void MirrorInPlace(SkeletonFrameData frame)
        {
            for (int i = 0; i < SkeletonFrameData.JointCount; i++)
            {
                Vec3 p = frame.Positions[i];
                frame.Positions[i] = new Vec3(-p.X, p.Y, p.Z);
            }
        }
    }
}
