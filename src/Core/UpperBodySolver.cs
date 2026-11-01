using System;
using System.Collections.Generic;

namespace Kinect2VMC.Core
{
    /// <summary>
    /// Mirror of <c>Microsoft.Kinect.JointType</c> member names.
    ///
    /// The numeric values are irrelevant: only the names matter, because the adapter and
    /// the solver communicate through this enum. That keeps the solver free of any
    /// Kinect assembly dependency, so its maths can be exercised without a sensor.
    /// </summary>
    public enum KinectJoint
    {
        SpineBase = 0,
        SpineMid,
        Neck,
        Head,
        ShoulderLeft,
        ElbowLeft,
        WristLeft,
        HandLeft,
        ShoulderRight,
        ElbowRight,
        WristRight,
        HandRight,
        HipLeft,
        KneeLeft,
        AnkleLeft,
        FootLeft,
        HipRight,
        KneeRight,
        AnkleRight,
        FootRight,
        SpineShoulder,
        HandTipLeft,
        ThumbLeft,
        HandTipRight,
        ThumbRight,
        Count
    }

    /// <summary>
    /// One frame of Kinect joint data, already converted into Unity's left-handed
    /// world convention and expressed in metres.
    /// </summary>
    public sealed class SkeletonFrameData
    {
        public const int JointCount = (int)KinectJoint.Count;

        public readonly Vec3[] Positions = new Vec3[JointCount];
        public readonly bool[] Tracked = new bool[JointCount];

        /// <summary>
        /// Each joint's FULL orientation, which is the one thing joint positions cannot carry.
        ///
        /// Kinect v2 exposes this as Body.JointOrientations. It was never read here -- the solver
        /// worked from positions alone, and a position says nothing about rotation about the bone's
        /// own axis. That is the entire reason the palm could not be controlled: with the arm
        /// hanging, the upper arm's swing carried the forearm and hand round rigidly and nothing
        /// put the roll back, so the palm ended up facing outwards.
        ///
        /// The alternative was to infer the roll from the thumb joint. That is a guess built on a
        /// guess, and every version of it produced a constant 180 degree error on one arm.
        /// Reading the real orientation removes the guessing entirely.
        ///
        /// Already converted into character space, and false where Kinect did not report one.
        /// </summary>
        public readonly Quat[] Orientations = new Quat[JointCount];
        public readonly bool[] HasOrientation = new bool[JointCount];

        /// <summary>True when the body this frame came from is being tracked at all.</summary>
        public bool BodyTracked;

        /// <summary>Kinect body tracking id, used to detect identity switches.</summary>
        public ulong TrackingId;

        /// <summary>Seconds since the sender started, used for VMC timing.</summary>
        public float Time;

        public void Clear()
        {
            BodyTracked = false;
            TrackingId = 0;
            for (int i = 0; i < JointCount; i++)
            {
                Positions[i] = Vec3.Zero;
                Tracked[i] = false;
                Orientations[i] = Quat.Identity;
                HasOrientation[i] = false;
            }
        }

        public void Set(KinectJoint joint, Vec3 position, bool tracked)
        {
            int index = (int)joint;
            if (index < 0 || index >= JointCount) { return; }
            Positions[index] = position;
            Tracked[index] = tracked;
        }

        public void SetOrientation(KinectJoint joint, Quat orientation, bool valid)
        {
            int index = (int)joint;
            if (index < 0 || index >= JointCount) { return; }
            Orientations[index] = orientation;
            HasOrientation[index] = valid;
        }

        public Quat GetOrientation(KinectJoint joint) { return Orientations[(int)joint]; }
        public bool IsOriented(KinectJoint joint) { return HasOrientation[(int)joint]; }

        public Vec3 Get(KinectJoint joint) { return Positions[(int)joint]; }
        public bool IsTracked(KinectJoint joint) { return Tracked[(int)joint]; }

        public bool HasStableTorso()
        {
            return IsTracked(KinectJoint.SpineBase)
                && IsTracked(KinectJoint.SpineShoulder)
                && IsTracked(KinectJoint.ShoulderLeft)
                && IsTracked(KinectJoint.ShoulderRight);
        }
    }

    /// <summary>
    /// Solves bone rotations from Kinect joint positions.
    ///
    /// The approach: every bone rotation is produced as an orthonormal frame whose
    /// primary axis is the measured bone direction and whose secondary axis is a
    /// reference taken from the body's own frame. The frame is emitted as a
    /// character-space (world) rotation, which is exactly what VMC Protocol expects and
    /// what Warudo consumes for its per-bone rotation override.
    ///
    /// This is preferred over Kinect's <c>JointOrientation</c> values, which are noisy
    /// and carry no usable twist for the limb segments.
    /// </summary>
    public sealed class UpperBodySolver
    {
        /// <summary>
        /// One bone the solver produces.
        ///
        /// The solver's model is deliberately minimal: a bone's rotation is the shortest arc that
        /// carries its neutral direction onto its measured direction. That is why there is no
        /// reference axis here. Kinect supplies joint positions only, so the bone's roll about
        /// its own axis has no unique solution; the shortest arc is the well-defined choice, and
        /// it keeps the roll stable across frames instead of letting it flip.
        /// </summary>
        private struct BoneDefinition
        {
            public string Name;             // VMC / Unity HumanBodyBones name
            public KinectJoint From;        // proximal joint, defines the bone origin
            public KinectJoint To;          // distal joint, defines the bone direction
            public Vec3 RestPrimary;        // bone direction in the neutral pose, character space
            public string Parent;           // parent bone's VMC name, or null for a chain root

            public BoneDefinition(string name, KinectJoint from, KinectJoint to, Vec3 restPrimary)
                : this(name, from, to, restPrimary, null)
            {
            }

            public BoneDefinition(string name, KinectJoint from, KinectJoint to, Vec3 restPrimary,
                string parent)
            {
                Name = name;
                From = from;
                To = to;
                RestPrimary = restPrimary;
                Parent = parent;
            }
        }        // Character space is left-handed, Y up, Z forward (the direction the character faces).
        // The user faces the Kinect, so the character faces -Z in Kinect's frame; after the
        // X mirror the character's own right/left/up are as written here.
        private static readonly Vec3 CharUp = new Vec3(0f, 1f, 0f);

        // Hanging arm rest direction, used when the avatar's bind pose is an A-pose. Almost
        // straight down but slightly out from the body. Not perfectly vertical on purpose: a
        // vertical arm is parallel to the character's yaw axis, and a shortest-arc swing between
        // collinear directions is undefined, so turning on the spot could not be represented.
        private static readonly Vec3 CharDown = new Vec3(-0.174f, -0.985f, 0f).Normalized();

        // The same hanging direction mirrored to the other side, for the RIGHT arm.
        //
        // The right arm originally used CharRight (straight out sideways) while the left used
        // CharDown, so the two sides disagreed about where a resting arm points. That is not a
        // small offset: a hanging right arm is about 76 degrees away from "sideways", so the
        // solver produced a large spurious rotation on the right hand and left the palm facing
        // the wrong way -- while the left, using the correct reference, behaved.
        private static readonly Vec3 CharDownRight = new Vec3(0.174f, -0.985f, 0f).Normalized();

        // The shoulder bones run sideways from the base of the neck out to the shoulder joint and
        // stay horizontal in either bind pose.
        private static readonly Vec3 CharLeft = new Vec3(-1f, 0f, 0f);
        private static readonly Vec3 CharRight = new Vec3(1f, 0f, 0f);
        private static readonly Vec3 CharForward = new Vec3(0f, 0f, 1f);
        private static readonly Vec3 CharBackward = new Vec3(0f, 0f, -1f);

        /// <summary>
        /// Which arm rest direction to use for the arm bones.
        ///
        /// This must match the AVATAR's BIND pose, which is the pose its bones are skinned in --
        /// NOT the pose a freshly loaded model happens to be showing. A model can display an
        /// idle animation with the arms lowered while its skeleton is still bound in a T-pose,
        /// and picking the wrong one produces large, systematic errors at every joint of the arm.
        /// </summary>
        public enum ArmReferencePose
        {
            /// <summary>Arms straight out to the sides. The usual rig/skinning bind pose.</summary>
            HorizontalT,

            /// <summary>Arms hanging down: an A-pose bind.</summary>
            HangingA
        }

        private ArmReferencePose _armReference = ArmReferencePose.HorizontalT;

        /// <summary>
        /// The arm rest direction currently in use. Changing this invalidates any captured neutral
        /// pose, because a captured pose is expressed relative to the reference it was taken with.
        /// </summary>
        public ArmReferencePose ArmReference
        {
            get { return _armReference; }
            set
            {
                if (_armReference == value) { return; }
                _armReference = value;
                ClearCalibration();
            }
        }

        /// <summary>The rest direction the arm bones use right now.</summary>
        public Vec3 ArmRestDirectionLeft
        {
            get { return _armReference == ArmReferencePose.HorizontalT ? CharLeft : CharDown; }
        }

        /// <summary>
        /// The right arm's rest direction: the exact mirror of the left one across the body's
        /// median plane. Flipping the sign of X here, rather than normalizing the same literals a
        /// second time, keeps the two bit-for-bit symmetric so a symmetric input pose really does
        /// produce mirrored rotations on the two sides.
        /// </summary>
        public Vec3 ArmRestDirectionRight
        {
            get { Vec3 l = ArmRestDirectionLeft; return new Vec3(-l.X, l.Y, l.Z); }
        }

        /// <summary>
        /// The built-in rest direction for the arm bones, kept for callers that need one value.
        /// Prefer <see cref="ArmRestDirectionLeft"/> now that the reference pose is selectable.
        /// </summary>
        public static Vec3 DefaultArmRestDirection { get { return CharLeft; } }

        private static readonly BoneDefinition[] Definitions = new[]
        {
            // Spine chain: each segment runs upward in the neutral pose. The parent names matter
            // because VMC carries each bone's rotation relative to its parent.
            new BoneDefinition(VmcBones.Spine,       KinectJoint.SpineBase,     KinectJoint.SpineMid,      CharUp),
            new BoneDefinition(VmcBones.Chest,       KinectJoint.SpineMid,      KinectJoint.SpineShoulder, CharUp,    VmcBones.Spine),
            new BoneDefinition(VmcBones.UpperChest,  KinectJoint.SpineShoulder, KinectJoint.Neck,          CharUp,    VmcBones.Chest),

            // Optional head chain. Off by default so the face tracking source owns it.
            new BoneDefinition(VmcBones.Neck,        KinectJoint.Neck,          KinectJoint.Head,          CharUp,    VmcBones.UpperChest),
            new BoneDefinition(VmcBones.Head,        KinectJoint.Neck,          KinectJoint.Head,          CharUp,    VmcBones.Neck),

            // Shoulders run from the base of the neck out to the shoulder joint: horizontal.
            new BoneDefinition(VmcBones.LeftShoulder, KinectJoint.SpineShoulder, KinectJoint.ShoulderLeft,  CharLeft, VmcBones.UpperChest),
            new BoneDefinition(VmcBones.LeftUpperArm, KinectJoint.ShoulderLeft,  KinectJoint.ElbowLeft,     CharDown, VmcBones.LeftShoulder),
            new BoneDefinition(VmcBones.LeftLowerArm, KinectJoint.ElbowLeft,     KinectJoint.WristLeft,     CharDown, VmcBones.LeftUpperArm),
            new BoneDefinition(VmcBones.LeftHand,     KinectJoint.WristLeft,     KinectJoint.HandTipLeft,   CharDown, VmcBones.LeftLowerArm),

            // The finger-tip joint drives the right hand direction because Kinect's Hand
            // joint sits almost on the wrist and yields a very short, noisy bone.
            new BoneDefinition(VmcBones.RightShoulder, KinectJoint.SpineShoulder, KinectJoint.ShoulderRight, CharRight, VmcBones.UpperChest),
            new BoneDefinition(VmcBones.RightUpperArm, KinectJoint.ShoulderRight, KinectJoint.ElbowRight,    CharDownRight, VmcBones.RightShoulder),
            new BoneDefinition(VmcBones.RightLowerArm, KinectJoint.ElbowRight,    KinectJoint.WristRight,    CharDownRight, VmcBones.RightUpperArm),
            new BoneDefinition(VmcBones.RightHand,     KinectJoint.WristRight,    KinectJoint.HandTipRight,  CharDownRight, VmcBones.RightLowerArm),

            // Wrist roll. A bone direction alone cannot describe a hand's twist, so the palm
            // direction is recovered from the thumb: the thumb root is the only landmark Kinect
            // gives that sits across the palm rather than along the fingers. These bones are not
            // part of the required upper-body set and are only emitted when finger tracking is on.
            new BoneDefinition(VmcBones.LeftThumb,  KinectJoint.WristLeft,  KinectJoint.ThumbLeft,  CharDown,  VmcBones.LeftHand),
            new BoneDefinition(VmcBones.RightThumb, KinectJoint.WristRight, KinectJoint.ThumbRight, CharRight, VmcBones.RightHand),

            // Full-body mode only. The legs are opt-in: from a seated position Kinect cannot see
            // them reliably, and a wrong leg pose looks worse than leaving the idle animation.
            new BoneDefinition(VmcBones.Hips,          KinectJoint.HipLeft,    KinectJoint.HipRight,   CharLeft),
            new BoneDefinition(VmcBones.LeftUpperLeg,  KinectJoint.HipLeft,    KinectJoint.KneeLeft,   CharDown, VmcBones.Hips),
            new BoneDefinition(VmcBones.LeftLowerLeg,  KinectJoint.KneeLeft,   KinectJoint.AnkleLeft,  CharDown, VmcBones.LeftUpperLeg),
            new BoneDefinition(VmcBones.LeftFoot,      KinectJoint.AnkleLeft,  KinectJoint.FootLeft,   CharDown, VmcBones.LeftLowerLeg),
            new BoneDefinition(VmcBones.RightUpperLeg, KinectJoint.HipRight,   KinectJoint.KneeRight,  CharDown, VmcBones.Hips),
            new BoneDefinition(VmcBones.RightLowerLeg, KinectJoint.KneeRight,  KinectJoint.AnkleRight, CharDown, VmcBones.RightUpperLeg),
            new BoneDefinition(VmcBones.RightFoot,     KinectJoint.AnkleRight, KinectJoint.FootRight,  CharDown, VmcBones.RightLowerLeg)
        };
        private readonly Dictionary<string, Quat> _smoothed = new Dictionary<string, Quat>(StringComparer.Ordinal);
        private readonly Dictionary<string, Vec3> _lastGoodPosition = new Dictionary<string, Vec3>(StringComparer.Ordinal);
        private readonly Dictionary<string, Quat> _worldByBone = new Dictionary<string, Quat>(StringComparer.Ordinal);

        /// <summary>Coarse hand pose for the left hand, refreshed every solve.</summary>
        public HandMetrics LeftHand;

        /// <summary>Coarse hand pose for the right hand, refreshed every solve.</summary>
        public HandMetrics RightHand;

        /// <summary>Latched two-state hand poses, used only when <see cref="EmitFingers"/> is on.</summary>
        public HandPose LeftHandPose = HandPose.Open;

        // Time-based debounce state, so a jittering reading cannot flip the state per frame.
        private HandPose _leftCandidate = HandPose.Open;
        private float _leftCandidateSince;
        private HandPose _rightCandidate = HandPose.Open;
        private float _rightCandidateSince;

        /// <summary>Latched two-state hand pose for the right hand.</summary>
        public HandPose RightHandPose = HandPose.Open;

        /// <summary>
        /// Emit the finger bones so a closed hand reads as a fist. Off by default: Kinect's
        /// hand joints are inferred, so the state can flicker, and that is worse than a hand
        /// that simply stays open.
        /// </summary>
        public bool EmitFingers;

        /// <summary>How a new reading is folded into the latched state.</summary>
        public HandPoseEstimator.HandStabilizer HandStabilizer = HandPoseEstimator.HandStabilizer.Hysteresis;

        /// <summary>How far the fingers fold when the hand is read as a fist, in degrees.</summary>
        public float FingerCurlDegrees = 80f;

        /// <summary>
        /// Carry the measured palm roll through to the fingers using the thumb direction.
        /// Off leaves the fingers aligned to the bone as before.
        /// </summary>
        public bool PalmRollEnabled = false;

        /// <summary>
        /// Which way the fingers fold. Settled on the real model rather than derived: the
        /// sign of forward x thumb is NOT a reliable palm indicator, because the thumb
        /// direction is not perpendicular to the finger direction and therefore flips that
        /// cross product for some hand poses. Exposed so it can be switched without a
        /// rebuild.
        /// </summary>
        public bool FingerCurlInward = true;

        /// <summary>
        /// The neutral pose's wrist-to-thumb direction, used to derive the palm's rest
        /// frame. Set from the user's own rig via calibration when available.
        /// </summary>
        public Vec3 DefaultRestThumbDirection = new Vec3(-0.955f, 0f, 0.297f).Normalized();

        /// <summary>VMC finger bone names, in Unity HumanBodyBones order.</summary>
        private static readonly string[] LeftFingerBones =
        {
            "LeftThumbProximal", "LeftThumbIntermediate", "LeftThumbDistal",
            "LeftIndexProximal", "LeftIndexIntermediate", "LeftIndexDistal",
            "LeftMiddleProximal", "LeftMiddleIntermediate", "LeftMiddleDistal",
            "LeftRingProximal", "LeftRingIntermediate", "LeftRingDistal",
            "LeftLittleProximal", "LeftLittleIntermediate", "LeftLittleDistal"
        };

        private static readonly string[] RightFingerBones =
        {
            "RightThumbProximal", "RightThumbIntermediate", "RightThumbDistal",
            "RightIndexProximal", "RightIndexIntermediate", "RightIndexDistal",
            "RightMiddleProximal", "RightMiddleIntermediate", "RightMiddleDistal",
            "RightRingProximal", "RightRingIntermediate", "RightRingDistal",
            "RightLittleProximal", "RightLittleIntermediate", "RightLittleDistal"
        };

        /// <summary>
        /// Bone directions measured during calibration, per bone name. When a bone has an
        /// entry it overrides the built-in default rest direction, which is what makes the
        /// solver independent of the avatar's bind pose and of how the user happens to sit.
        /// </summary>
        private readonly Dictionary<string, Vec3> _calibratedRestPrimary = new Dictionary<string, Vec3>(StringComparer.Ordinal);
        private readonly Dictionary<string, Vec3> _calibratedRestThumbRef = new Dictionary<string, Vec3>(StringComparer.Ordinal);
        private readonly Dictionary<string, Quat> _calibratedJointRotation = new Dictionary<string, Quat>(StringComparer.Ordinal);

        /// <summary>
        /// Emit each bone's rotation relative to its parent (VMC's documented meaning for
        /// /VMC/Ext/Bone/Pos) instead of its rotation in character space.
        ///
        /// With this off, every joint compounds its parent's rotation, so an error grows
        /// down the chain and a straight arm bends progressively at the elbow and wrist.
        /// Turn it off only for a receiver that really does want world rotations.
        /// </summary>
        public bool EmitLocalRotations = true;

        /// <summary>0 = no smoothing (raw), 1 = frozen. Applied in quaternion space.</summary>
        public float Smoothing = 0.45f;

        /// <summary>Emit Neck and Head from Kinect. Leave false when VSeeFace owns the head.</summary>
        public bool SendHead;

        /// <summary>Emit Hips. Leave false for seated tracking so the idle animation keeps the pelvis.</summary>
        public bool SendHips;

        /// <summary>Emit Hips and the leg chains, for full-body tracking.</summary>
        public bool SendFullBody;

        /// <summary>Emit the root rotation so the avatar turns with the user.</summary>
        public bool SendRootRotation;

        /// <summary>
        /// Emit the LeftHand / RightHand bones.
        ///
        /// Default ON, but exposed as a switch because the hand's palm orientation cannot be
        /// determined from Kinect's joint positions at all: those give a DIRECTION only, and the
        /// solver's shortest-arc rotation therefore fixes where the hand points while leaving its
        /// roll to whatever the rig inherits from the forearm.
        ///
        /// Probing the solver showed the hand bone's own rotation is 0 degrees with the arm
        /// hanging, and that neither jitter, nor a degenerate axis, nor roll inherited down the
        /// arm chain can account for a 180 degree palm flip. That leaves the rig's own hand
        /// orientation (or how it is applied) as the remaining candidate, so being able to turn
        /// this bone off entirely is what tells the two apart.
        /// </summary>
        /// <summary>
        /// Emit the LeftHand / RightHand bones. ON by default.
        ///
        /// The hand's orientation is better driven from the camera. The user established this on
        /// the real setup: with the hand script running and this program not, the palm could be
        /// turned and rolled correctly, while Kinect's hand data managed at best a wave and never
        /// got the palm's facing right. That is expected rather than surprising -- Kinect's Hand
        /// joint sits almost on the wrist, so the bone is about 9 cm long and its orientation is
        /// derived from a very short, noisy segment.
        ///
        /// Turning this off leaves the palm entirely to the hand script. It is still a switch
        /// because the hand script is optional, and with it absent the rig's idle hand is all
        /// there is.
        /// </summary>
        public bool SendHand = true;

        /// <summary>
        /// Give the forearm the twist that turns the palm, using the thumb as the
        /// reference. This is the missing degree of freedom: a joint POSITION says nothing
        /// about rotation about the bone axis, and a shortest-arc swing introduces none, so
        /// without this the palm simply drags round with the upper arm.
        /// </summary>
        public bool SendForearmTwist = true;

        /// <summary>Per-bone twist angle in degrees from the last solve, for diagnosis.</summary>
        public System.Collections.Generic.Dictionary<string, float> DiagnosticTwist =
            new System.Collections.Generic.Dictionary<string, float>();

        /// <summary>True once a neutral pose has been captured.</summary>
        public bool IsCalibrated { get { return _calibratedRestPrimary.Count > 0; } }

        /// <summary>
        /// Maps a bone definition to the rest direction in force right now. Torso bones keep their
        /// built-in value; the arm bones follow the selected reference pose.
        /// </summary>
        private Vec3 ResolveRestPrimary(BoneDefinition def)
        {
            string name = def.Name;

            if (name == VmcBones.LeftUpperArm || name == VmcBones.LeftLowerArm || name == VmcBones.LeftHand)
            {
                return ArmRestDirectionLeft;
            }

            if (name == VmcBones.RightUpperArm || name == VmcBones.RightLowerArm || name == VmcBones.RightHand)
            {
                return ArmRestDirectionRight;
            }

            return def.RestPrimary;
        }

        /// <summary>Number of bones produced by the last solve.</summary>
        public int LastBoneCount { get; private set; }

        /// <summary>Bones that could not be solved this frame (missing or degenerate joints).</summary>
        public readonly List<string> LastMissingBones = new List<string>(4);

        /// <summary>Diagnostic: last solved primary direction and reference axis, per bone.</summary>
        public readonly Dictionary<string, Vec3> DiagnosticPrimary = new Dictionary<string, Vec3>(StringComparer.Ordinal);
        public readonly Dictionary<string, Quat> DiagnosticResult = new Dictionary<string, Quat>(StringComparer.Ordinal);
        /// <summary>Diagnostic: the measured body frame, per solve.</summary>
        public Vec3 DiagnosticBodyUp, DiagnosticBodyRight, DiagnosticBodyForward;
        public Quat DiagnosticBodyFrame = Quat.Identity;
        /// <summary>Diagnostic: the rest direction actually used, per bone.</summary>
        public readonly Dictionary<string, Vec3> DiagnosticRestPrimary = new Dictionary<string, Vec3>(StringComparer.Ordinal);

        public void Reset()
        {
            _smoothed.Clear();
            _lastGoodPosition.Clear();
        }

        /// <summary>Drops the captured neutral pose and falls back to the built-in defaults.</summary>
        public void ClearCalibration()
        {
            _calibratedShoulderLine = default(Vec3);
            _calibratedRestPrimary.Clear();
            _smoothed.Clear();

            // 转身的基准也一起作废，并且把平滑/静止状态清掉 —— 这样用户坐定之后
            // 会重新自动建立基准，而不必依赖"点一次校准"。
            _headingSmoothedValid = false;
        }

        /// <summary>
        /// Captures the current frame as the neutral pose.
        ///
        /// Each bone's neutral direction becomes whatever it measures right now, so solving
        /// the same pose again yields the identity rotation for every bone. That is exactly
        /// the property Warudo needs: the avatar's bind pose lines up with the user's own
        /// neutral pose. Adopting the live bone directions also makes the solver independent
        /// of the user's build and of how they happen to sit.
        /// </summary>
        public bool Calibrate(SkeletonFrameData frame)
        {
            if (frame == null || !frame.BodyTracked || !frame.HasStableTorso()) { return false; }

            var capturedPrimary = new Dictionary<string, Vec3>(StringComparer.Ordinal);

            for (int i = 0; i < Definitions.Length; i++)
            {
                BoneDefinition def = Definitions[i];
                if (!SendHead && (def.Name == VmcBones.Head || def.Name == VmcBones.Neck)) { continue; }
                // The thumb bones exist only to carry the palm's roll to the fingers.
                if (!EmitFingers && (def.Name == VmcBones.LeftThumb || def.Name == VmcBones.RightThumb)) { continue; }

                Vec3 from = frame.Get(def.From);
                Vec3 to = frame.Get(def.To);
                Vec3 primary = (to - from).Normalized();
                if (primary.LengthSquared < 0.5f) { continue; }

                capturedPrimary[def.Name] = primary;
            }

            if (capturedPrimary.Count == 0) { return false; }

            // 肩线参考（转身的零点）= 校准这一刻的肩线方向。优先取平滑后的方向：
            // 单帧采样带着几度关节噪声，拿它当基准会让"正对镜头"也读出几度偏航
            //（上半身朝一侧偏、腰部扭）。
            Vec3 sl = _headingSmoothedValid
                ? new Vec3(_headingSmoothed.X, 0f, _headingSmoothed.Z)
                : (frame.Get(KinectJoint.ShoulderLeft) - frame.Get(KinectJoint.ShoulderRight));
            if (sl.IsFinite() && sl.LengthSquared >= 1e-6f)
            {
                _calibratedShoulderLine = sl.Normalized();
                DiagnosticHeadingReferenceDegrees = (float)(Math.Atan2(
                    _calibratedShoulderLine.Z, _calibratedShoulderLine.X) * 57.29577951308232);
            }
            else
            {
                _calibratedShoulderLine = default(Vec3);
            }
            _calibratedRestPrimary.Clear();
            foreach (var pair in capturedPrimary) { _calibratedRestPrimary[pair.Key] = pair.Value; }

            // Also capture where each thumb sits relative to its forearm, in this pose.
            //
            // This is what makes "the neutral pose means no twist" exactly true for THIS user on
            // THIS rig, instead of relying on a hard-coded thumb direction. A guessed reference
            // produced a constant 180 degree forearm twist on one arm in every pose -- the
            // reference simply landed on the wrong side of the bone. Measuring it instead removes
            // that whole class of error: at the moment of calibration the twist is zero by
            // definition, whatever the person's natural arm position happens to be.
            _calibratedRestThumbRef.Clear();
            _calibratedJointRotation.Clear();
            for (int i = 0; i < Definitions.Length; i++)
            {
                BoneDefinition def = Definitions[i];
                if (def.Name != VmcBones.LeftLowerArm && def.Name != VmcBones.RightLowerArm) { continue; }

                Vec3 refDir;
                if (TryBuildThumbReference(frame, def.Name, capturedPrimary[def.Name], out refDir))
                {
                    _calibratedRestThumbRef[def.Name] = refDir;
                }
            }

            // And each arm bone's orientation-derived rotation in this pose, so the neutral pose
            // reports the identity on the orientation path too. Without this the neutral pose
            // would carry whatever absolute rotation Kinect happens to report.
            for (int i = 0; i < Definitions.Length; i++)
            {
                BoneDefinition def = Definitions[i];
                KinectJoint parentJoint, childJoint;
                if (!OrientationJointsFor(def.Name, out parentJoint, out childJoint)) { continue; }

                Quat oriented;
                if (TryBuildJointRotation(frame, parentJoint, childJoint, out oriented))
                {
                    _calibratedJointRotation[def.Name] = oriented;
                }
            }

            _smoothed.Clear();
            return true;
        }

        /// <summary>Which parent/child joint pair supplies a bone's orientation.</summary>
        private bool OrientationJointsFor(string boneName, out KinectJoint parentJoint,
            out KinectJoint childJoint)
        {
            switch (boneName)
            {
                case VmcBones.LeftUpperArm:
                    parentJoint = KinectJoint.ShoulderLeft; childJoint = KinectJoint.ElbowLeft; return true;
                case VmcBones.RightUpperArm:
                    parentJoint = KinectJoint.ShoulderRight; childJoint = KinectJoint.ElbowRight; return true;
                case VmcBones.LeftLowerArm:
                    parentJoint = KinectJoint.ElbowLeft; childJoint = KinectJoint.WristLeft; return true;
                case VmcBones.RightLowerArm:
                    parentJoint = KinectJoint.ElbowRight; childJoint = KinectJoint.WristRight; return true;
                case VmcBones.LeftHand:
                    parentJoint = KinectJoint.WristLeft; childJoint = KinectJoint.HandLeft; return true;
                case VmcBones.RightHand:
                    parentJoint = KinectJoint.WristRight; childJoint = KinectJoint.HandRight; return true;
                default:
                    parentJoint = KinectJoint.SpineBase; childJoint = KinectJoint.SpineMid; return false;
            }
        }

        /// <summary>
        /// Solves the upper body and fills <paramref name="destination"/>.
        /// Returns true when the torso was tracked well enough to produce a usable pose.
        /// </summary>
        public bool Solve(SkeletonFrameData frame, SkeletonPose destination)
        {
            if (frame == null) { throw new ArgumentNullException("frame"); }
            if (destination == null) { throw new ArgumentNullException("destination"); }

            LastBoneCount = 0;
            LastMissingBones.Clear();

            if (!frame.BodyTracked || !frame.HasStableTorso())
            {
                return false;
            }

            Vec3 up, right, forward;
            if (!TryBuildCharacterFrame(frame, out up, out right, out forward))
            {
                return false;
            }

            // Each bone's rotation is the deviation of its measured direction from its neutral
            // direction, both expressed in character space.
            //
            // This is deliberately measured in character space rather than relative to the
            // body's own frame. The body-frame variant is "more correct" on paper (a rigid
            // turn of the whole body would produce no bone rotation at all), but it requires
            // an exactly consistent body frame, and any error there leaks into every bone as
            // a constant offset. The character-space form is self-consistent by construction:
            // it uses the very same rest directions the neutral pose defines, so the neutral
            // pose maps to the identity rotation exactly and no synthetic offset can appear.
            //
            // The visible consequence is that turning the whole body swings the arms through
            // an arc, because a horizontal arm genuinely does change its world direction when
            // its owner turns on the spot. For a seated streamer this is acceptable: the root
            // position follows the body, and anyone who wants the avatar to turn with them
            // should send root rotation (see SkeletonPose.HasRootRotation).
            _worldByBone.Clear();
            DiagnosticBodyUp = up;
            DiagnosticBodyRight = right;
            DiagnosticBodyForward = forward;

            Vec3 spineBase = frame.Get(KinectJoint.SpineBase);

            // Coarse hand pose, measured from Kinect's five hand joints. Kinect provides no
            // per-finger joint angles, but a fist pulls the fingertip towards the palm and the
            // thumb while an open hand pushes it away, so the hand's shape is recoverable even
            // though the individual fingers are not.
            // The fingertip and thumb joints must have actually been reported. Kinect zeroes them
            // when it loses the hand, and a zeroed pair looks exactly like a fist.
            LeftHand = HandPoseEstimator.Compute(
                frame.Get(KinectJoint.WristLeft), frame.Get(KinectJoint.HandLeft),
                frame.Get(KinectJoint.HandTipLeft), frame.Get(KinectJoint.ThumbLeft),
                frame.Tracked[(int)KinectJoint.HandTipLeft] && frame.Tracked[(int)KinectJoint.ThumbLeft]);
            RightHand = HandPoseEstimator.Compute(
                frame.Get(KinectJoint.WristRight), frame.Get(KinectJoint.HandRight),
                frame.Get(KinectJoint.HandTipRight), frame.Get(KinectJoint.ThumbRight),
                frame.Tracked[(int)KinectJoint.HandTipRight] && frame.Tracked[(int)KinectJoint.ThumbRight]);

            for (int i = 0; i < Definitions.Length; i++)
            {
                BoneDefinition def = Definitions[i];

                if (!SendHead && (def.Name == VmcBones.Head || def.Name == VmcBones.Neck)) { continue; }
                // The thumb bones exist only to carry the palm's roll to the fingers.
                if (!EmitFingers && (def.Name == VmcBones.LeftThumb || def.Name == VmcBones.RightThumb)) { continue; }
                if (!SendHips && def.Name == VmcBones.Hips) { continue; }

                // Off means the hand keeps whatever the rig's own idle pose gives it, so the
                // fingers still work while the palm is left alone.
                if (!SendHand && (def.Name == VmcBones.LeftHand || def.Name == VmcBones.RightHand)) { continue; }

                // The pelvis and the legs are opt-in, so a seated user keeps the idle pose.
                if (!SendFullBody && IsLegOrHips(def.Name)) { continue; }

                // The pelvis and the legs are opt-in, so a seated user keeps the idle pose.

                Vec3 from = ResolvePosition(frame, def.From);
                Vec3 to = ResolvePosition(frame, def.To);

                Vec3 primary = to - from;
                if (!primary.IsFinite() || primary.LengthSquared < 1e-6f)
                {
                    LastMissingBones.Add(def.Name);
                    continue;
                }
                primary = primary.Normalized();

                // The arm bones take their rest direction from the selected reference pose, so
                // the user can switch between a T-pose and an A-pose bind without a rebuild.
                Vec3 restPrimary = LookupOverride(_calibratedRestPrimary, def.Name,
                    ResolveRestPrimary(def));

                DiagnosticPrimary[def.Name] = primary;
                DiagnosticRestPrimary[def.Name] = restPrimary;


                Quat target;
                if (!TryBuildRotation(restPrimary, primary, out target))
                {
                    LastMissingBones.Add(def.Name);
                    continue;
                }

                // Non-arm bones, and optionally the arms, take their rotation from Kinect's own
                // joint orientations. Those carry the twist that a position cannot describe.
                if (UseJointOrientations
                    && (def.Name == VmcBones.LeftUpperArm || def.Name == VmcBones.RightUpperArm
                        || def.Name == VmcBones.LeftLowerArm || def.Name == VmcBones.RightLowerArm
                        || def.Name == VmcBones.LeftHand || def.Name == VmcBones.RightHand))
                {
                    // Which joint pair supplies this bone's orientation.
                    //
                    // This used to be an inline chain of ternaries covering only the upper arm and
                    // the forearm, with everything else falling through to the forearm's pair. The
                    // HAND bones therefore received the FOREARM's rotation, which gave them no
                    // independent rotation at all -- so the wrist could not turn.
                    //
                    // Routing through the shared helper keeps ONE definition instead of a chain
                    // that silently runs out of cases.
                    KinectJoint parentJoint, childJoint;
                    if (!OrientationJointsFor(def.Name, out parentJoint, out childJoint))
                    {
                        parentJoint = KinectJoint.ElbowLeft;
                        childJoint = KinectJoint.WristLeft;
                    }

                    Quat oriented;
                    if (TryBuildJointRotation(frame, parentJoint, childJoint, out oriented))
                    {
                        // Measure it against the same quantity in the rest pose, so the neutral
                        // pose is the identity and the result is in the bone's own local frame.
                        Quat restOriented;
                        if (_calibratedJointRotation.TryGetValue(def.Name, out restOriented))
                        {
                            target = oriented * restOriented.Conjugate();
                        }
                        else
                        {
                            target = oriented;
                        }
                    }
                }
                // The forearm carries the palm's roll.
                //
                // A shortest-arc swing fixes direction only, so nothing in the chain ever turned
                // the palm -- with the arm hanging, the upper arm's swing dragged the whole forearm
                // and hand round and the palm ended up facing outwards. See TryBuildForearmTwist:
                // the reference comes from the thumb, measured against the rest pose's own geometry
                // so that the neutral pose means no twist by construction.
                //
                // Applied on the FOREARM, not the hand: the user proved the fault survives with the
                // hand bone switched off entirely, and the hand and fingers both inherit this.
                if (SendForearmTwist
                    && (def.Name == VmcBones.LeftLowerArm || def.Name == VmcBones.RightLowerArm))
                {
                    Quat twist;
                    if (TryBuildForearmTwist(frame, def.Name, out twist))
                    {
                        // Post-multiplied so the twist happens about the bone's own axis in its
                        // local frame; pre-multiplying would swing the direction instead.
                        target = target * twist;
                        DiagnosticTwist[def.Name] = (float)(2.0 * Math.Acos(Math.Min(1.0, Math.Abs(twist.W))) * 57.29578);
                    }
                }

                // The hand bones carry the palm's twist as well as their direction.
                //
                // The shortest arc only fixes the direction the hand points, so on its own the
                // palm can never roll -- and the fingers, which inherit this bone, cannot roll
                // either. Multiplying by the palm frame's rotation adds exactly the missing twist
                // without changing the direction, because that rotation maps the rest frame onto
                // the measured one and therefore leaves the shared forward axis alone.
                // NOTE: the palm's twist is deliberately NOT applied here yet. The frame maths for
                // it is written (TryBuildPalmTwist) but not trustworthy: with the rest pose it
                // produced a constant 90-180 degree error instead of identity, and the version that
                // tried to fix it by deriving the rest frame from DefaultRestThumbDirection crashed
                // the solver outright. So the hand bone still carries direction only, and a wrist
                // turn is reported on the fingers by nothing at all -- the palm simply does not
                // roll. Shipping a constant wrist error would be worse than shipping no roll.
                // Accumulate the chain so children can express their rotation relative to their
                // parent. The dictionary is keyed by bone name and only ever holds this frame's
                // values, so it is cleared per solve.
                _worldByBone[def.Name] = target;

                // VMC's /VMC/Ext/Bone/Pos carries the bone's LOCAL rotation, relative to its
                // parent. Emitting the world rotation instead makes every joint compound its
                // parent's rotation, so the error grows down the chain: an arm that should be
                // straight bends a little at the elbow and again at the wrist, which is exactly
                // the symptom the user reported as "the forearm and then the hand fold by 90
                // degrees each".
                Quat output = target;
                if (EmitLocalRotations)
                {
                    Quat parentWorld;
                    if (def.Parent != null && _worldByBone.TryGetValue(def.Parent, out parentWorld))
                    {
                        output = parentWorld.Conjugate() * target;
                    }
                }

                output = ApplySmoothing(def.Name, output);
                DiagnosticResult[def.Name] = output;
                destination.SetBone(def.Name, output);
                LastBoneCount++;
            }

            // Fingers last, because their rotation is expressed relative to the hand bone.
            if (EmitFingers)
            {
                // Debounced over time, not just by distance: the hand joints are inferred and
                // jitter, so a threshold alone flips the state every frame, which the user saw as
                // the fist twitching continuously.
                LeftHandPose = HandPoseEstimator.StabilizeOverTime(
                    LeftHand.TipToThumb, LeftHandPose, frame.Time,
                    ref _leftCandidate, ref _leftCandidateSince, HandStabilizer);
                RightHandPose = HandPoseEstimator.StabilizeOverTime(
                    RightHand.TipToThumb, RightHandPose, frame.Time,
                    ref _rightCandidate, ref _rightCandidateSince, HandStabilizer);

                EmitFingerBones(destination, VmcBones.LeftHand, LeftHandPose == HandPose.Fist,
                    frame, KinectJoint.WristLeft, KinectJoint.HandTipLeft, KinectJoint.ThumbLeft);
                EmitFingerBones(destination, VmcBones.RightHand, RightHandPose == HandPose.Fist,
                    frame, KinectJoint.WristRight, KinectJoint.HandTipRight, KinectJoint.ThumbRight);
            }

            // Root translation: the character follows the body, but only by the amount the
            // body moved away from its calibrated neutral position.
            destination.RootPosition = spineBase;
            destination.HasRootPosition = true;
            destination.Tracking = true;
            destination.Time = frame.Time;
            destination.Status = VmcTrackingStatus.Loaded | VmcTrackingStatus.Tracking;

            // Root yaw, from the line across the shoulders. Without it the character keeps
            // facing one way and a small turn shows up as the arms drifting instead.
            if (SendRootRotation)
            {
                Quat yaw;
                if (TryBuildRootYaw(frame, out yaw))
                {
                    destination.RootRotation = yaw;
                    destination.HasRootRotation = true;
                }
            }

            return LastBoneCount > 0;
        }

        /// <summary>
        /// Emits one hand's finger bones, either identity (open) or folded (fist).
        ///
        /// Kinect supplies no per-finger joint angles, so all four fingers curl together by the
        /// same amount. A fist is a strong, unambiguous visual, so a uniform fold reads correctly
        /// even though the individual fingers are not tracked.
        ///
        /// The rotation is applied in the HAND's frame, taken from the real wrist, fingertip and
        /// thumb positions: forward is along the fingers, up is the palm normal, and curling means
        /// rotating about the palm's transverse axis. Building the frame from measurements rather
        /// than assuming an identity hand means the fold stays correct however the wrist is turned.
        /// </summary>
        /// <summary>
        /// The rotation that carries the hand's rest frame onto its measured frame.
        ///
        /// This is the palm's twist: it maps the rest forward axis to the measured one, so applying
        /// it does not change the direction the hand points -- only which way the palm faces.
        /// A single bone direction cannot express this, which is why the thumb direction is needed
        /// to build a second axis.
        /// </summary>
        /// <summary>
        /// The user's heading: the rotation about the vertical axis that carries the REFERENCE
        /// shoulder line onto the current one.
        ///
        /// Measured from the shoulder line rather than from a single bone, because a turn rotates
        /// the whole torso and the shoulders span it. It is expressed relative to the reference
        /// pose, so a user who sits slightly askew still counts as facing forward until they turn.
        /// Both lines are projected onto the ground plane first, so leaning forward cannot be
        /// mistaken for turning.
        ///
        /// 为什么参考要平滑、要等坐定（这一条是实测踩出来的）
        /// ------------------------------------------------
        /// 肩线是两根关节位置之差：基线只有 40 cm 上下，而 Kinect 单帧关节点噪声有 1~3 cm，
        /// 于是"某一帧的肩线方向"本身就带几度误差。参考只取一帧，那几度就固化成永久的
        /// 朝向偏移 —— 用户看到的是"正对镜头，模型却朝一侧偏几度、腰是扭的"。所以：
        ///   ① 当前肩线做指数平滑；
        ///   ② 参考从平滑后的方向取，并且要等肩线静止一小段时间（用户坐定）才建立 ——
        ///      刚启动那几帧还在抖、用户可能正伸手去点开始，都不能当基准；
        ///   ③ 手点「校准」随时把基准重新归零（见 Calibrate）。
        /// 未建立基准时不产生转身量，而不是拿一个坏基准硬转。
        /// </summary>
        private bool TryBuildRootYaw(SkeletonFrameData frame, out Quat yaw)
        {
            yaw = Quat.Identity;

            Vec3 left = frame.Get(KinectJoint.ShoulderLeft);
            Vec3 right = frame.Get(KinectJoint.ShoulderRight);

            // 只取水平分量：前倾、侧倾都不该被当成转身。
            Vec3 current = new Vec3(left.X - right.X, 0f, left.Z - right.Z);
            if (!current.IsFinite() || current.LengthSquared < 1e-6f) { return false; }
            current = current.Normalized();

            if (!_headingSmoothedValid)
            {
                _headingSmoothedValid = true;
                _headingSmoothed = current;
                _headingStillAnchor = current;
                _headingStillSince = frame.Time;
            }
            else
            {
                Vec3 blended = Vec3.Lerp(_headingSmoothed, current, HeadingSmoothing);
                if (blended.IsFinite() && blended.LengthSquared > 1e-6f)
                {
                    _headingSmoothed = blended.Normalized();
                }

                // 静止判定必须看"平滑后"的方向，不能看单帧：单帧噪声会让判定永远不成立，
                // 那样基准一直建不出来 —— 表现就是"转身完全没反应"。
                // 超过阈值就把锚点挪到当前值并重新计时，于是持续转身时永远不会判定为静止。
                if (AngleBetweenDegrees(_headingSmoothed, _headingStillAnchor) > HeadingStillDegrees)
                {
                    _headingStillAnchor = _headingSmoothed;
                    _headingStillSince = frame.Time;
                }
            }

            if (!_calibratedShoulderLine.IsFinite() || _calibratedShoulderLine.LengthSquared < 1e-6f)
            {
                // 还没有基准：等肩线静止足够久再建立，本次不产生转身量。
                if (frame.Time - _headingStillSince >= HeadingSettleSeconds)
                {
                    Vec3 reference = new Vec3(_headingSmoothed.X, 0f, _headingSmoothed.Z);
                    if (reference.LengthSquared < 1e-8f) { return false; }
                    _calibratedShoulderLine = reference.Normalized();
                    DiagnosticHeadingReferenceDegrees = (float)(Math.Atan2(
                        _calibratedShoulderLine.Z, _calibratedShoulderLine.X) * 57.29577951308232);
                }
                return false;
            }

            Vec3 a = new Vec3(_calibratedShoulderLine.X, 0f, _calibratedShoulderLine.Z);
            if (a.LengthSquared < 1e-8f) { return false; }
            a = a.Normalized();

            float dot = Vec3.Dot(a, _headingSmoothed);
            if (dot > 1f) { dot = 1f; } else if (dot < -1f) { dot = -1f; }

            // 没转身：yaw 保持单位四元数（仍然算"有效"，好让接收端在这一刻建立自己的基准）。
            if (dot > 0.999999f) { return true; }

            // 正好背对：最短弧退化，显式取竖直轴转 180°，否则会得到一个"翻跟头"式的旋转。
            if (dot < -0.999999f)
            {
                yaw = Quat.AxisAngle(CharUp, (float)Math.PI);
                return true;
            }

            Vec3 axis = Vec3.Cross(a, _headingSmoothed);
            yaw = Quat.AxisAngle(axis, (float)Math.Acos(dot));
            return yaw.IsFinite();
        }

        private static float AngleBetweenDegrees(Vec3 a, Vec3 b)
        {
            float dot = Vec3.Dot(a, b);
            if (dot > 1f) { dot = 1f; } else if (dot < -1f) { dot = -1f; }
            return (float)(Math.Acos(dot) * 57.29577951308232);
        }

        /// <summary>The shoulder line at calibration time, used as the heading reference.</summary>
        private Vec3 _calibratedShoulderLine;

        /// <summary>肩线的指数平滑值（水平方向，已归一化）。</summary>
        private Vec3 _headingSmoothed;
        private bool _headingSmoothedValid;
        /// <summary>静止判定的锚点：平滑后的肩线最后一次"明显移动"时所在的方向。</summary>
        private Vec3 _headingStillAnchor;
        /// <summary>平滑后的肩线保持在上面的锚点附近不动的起始时刻。</summary>
        private float _headingStillSince;

        /// <summary>肩线平滑系数（每帧向当前值靠拢的比例）：越小越稳、越迟钝。</summary>
        private const float HeadingSmoothing = 0.25f;

        /// <summary>平滑后的肩线偏离锚点超过这个角度（度）就算"在动"，重新锚定并计时。</summary>
        private const float HeadingStillDegrees = 4f;

        /// <summary>静止持续这么久（秒）才自动建立基准，也就是"用户已经坐定"。</summary>
        private const float HeadingSettleSeconds = 1f;

        /// <summary>基准肩线相对相机 +X 的朝向（度），仅用于诊断。</summary>
        public float DiagnosticHeadingReferenceDegrees { get; private set; }

        private Vec3 CalibratedOrDefault(string boneName, Vec3 fallback)
        {
            Vec3 v;
            return _calibratedRestPrimary.TryGetValue(boneName, out v) ? v : fallback;
        }

        /// <summary>
        /// The forearm's twist: the rotation ABOUT the forearm's own axis that turns the palm.
        ///
        /// Why the arm needs this
        /// ----------------------
        /// Every bone here is solved as a shortest-arc swing from its rest direction to its
        /// measured one. A swing is rotation-minimising, so it introduces NO twist about the bone's
        /// axis -- which is exactly why nothing in the chain ever turned the palm. The user found
        /// the real symptom: with the arm hanging, the whole lower arm appeared to roll half a
        /// turn, because the upper arm's swing carries the forearm and hand round rigidly and
        /// nothing puts the roll back. Real arms get it from humeral rotation and forearm
        /// pronation, and a joint POSITION (which is all Kinect sends for the arm) says nothing
        /// about either.
        ///
        /// The reference comes from the thumb. The thumb sits off the forearm's axis, so where it
        /// is relative to the wrist says how the palm is turned -- the one piece of orientation
        /// information the hand landmarks-less skeleton still carries.
        ///
        /// Measured against the REST pose's own geometry rather than a hard-coded palm normal.
        /// That is what makes "neutral pose means no twist" true BY CONSTRUCTION, whatever a given
        /// rig's thumb happens to do: if the measurement equals the rest, the two perpendicular
        /// references coincide and the angle is zero. Assuming a palm normal instead is what made
        /// this ship once as a constant 90-180 degree error.
        /// </summary>
        private bool TryBuildForearmTwist(SkeletonFrameData frame, string forearmBone, out Quat twist)
        {
            twist = Quat.Identity;

            bool left = forearmBone == VmcBones.LeftLowerArm;
            KinectJoint elbow = left ? KinectJoint.ElbowLeft : KinectJoint.ElbowRight;
            KinectJoint wrist = left ? KinectJoint.WristLeft : KinectJoint.WristRight;
            KinectJoint thumb = left ? KinectJoint.ThumbLeft : KinectJoint.ThumbRight;

            Vec3 axis = frame.Get(wrist) - frame.Get(elbow);
            Vec3 toThumb = frame.Get(thumb) - frame.Get(wrist);
            if (!axis.IsFinite() || axis.LengthSquared < 1e-8f
                || !toThumb.IsFinite() || toThumb.LengthSquared < 1e-8f)
            {
                return false;
            }
            axis = axis.Normalized();

            // The thumb's offset from the forearm axis: a direction perpendicular to it, which is
            // the part that actually describes the palm's roll.
            Vec3 measuredRef = toThumb - axis * Vec3.Dot(toThumb, axis);
            if (!measuredRef.IsFinite() || measuredRef.LengthSquared < 1e-8f) { return false; }
            measuredRef = measuredRef.Normalized();

            // The same construction in the rest pose, from the calibrated rest geometry.
            Vec3 restAxis = LookupOverride(_calibratedRestPrimary, forearmBone,
                ResolveRestPrimary(def_placeholder_for_rest(forearmBone)));

            // The reference comes from the CALIBRATED rest pose when one was captured, which makes
            // "neutral means no twist" exact for this user on this rig.
            //
            // The hard-coded thumb direction below is only a fallback for before calibration, and
            // it is the path that produced a constant 180 degree twist on one arm in every pose:
            // a guessed reference easily lands on the wrong side of the bone. Mirroring it by
            // flipping X, or X and Z, both just moved the half turn from one arm to the other.
            // Measuring the reference instead removes the guess entirely.
            // Only a MEASURED reference is trustworthy.
            //
            // The fallback used to guess a thumb direction from a constant and mirror it for the
            // other arm. Every version of that guess produced a constant 180 degree twist on one
            // arm in EVERY pose, including the neutral one, because the guessed reference kept
            // landing on the wrong side of the bone. Flipping X, or X and Z, only moved the half
            // turn from one arm to the other.
            //
            // Without a calibration the twist is therefore not applied at all. Direction never
            // depended on it, so the arm still tracks correctly; only the palm's roll waits for
            // the user to calibrate in their own rest pose, which is the one moment the correct
            // reference is known for certain.
            Vec3 restRef;
            if (!_calibratedRestThumbRef.TryGetValue(forearmBone, out restRef))
            {
                twist = Quat.Identity;
                return false;
            }
            // Rotate the rest reference onto the measured one, about the forearm's axis.
            Quat swing;
            if (!TryBuildRotation(restAxis, axis, out swing)) { return false; }
            Vec3 restRefInSwung = (swing * restRef).Normalized();

            Vec3 cross = Vec3.Cross(restRefInSwung, measuredRef);
            float dot = Vec3.Dot(restRefInSwung, measuredRef);
            if (dot > 1f) { dot = 1f; } else if (dot < -1f) { dot = -1f; }

            if (cross.LengthSquared < 1e-12f)
            {
                // Already aligned, or exactly opposed (a half turn about the forearm).
                Vec3 degAxis = (swing.Conjugate() * axis).Normalized();
                twist = dot >= 0f || !degAxis.IsFinite() || degAxis.LengthSquared < 1e-10f
                    ? Quat.Identity : Quat.AxisAngle(degAxis, (float)Math.PI);
                return twist.IsFinite();
            }

            float angle = (float)Math.Acos(dot);
            // The sign of the twist follows the direction the reference has to travel.
            if (Vec3.Dot(cross, axis) < 0f) { angle = -angle; }

            // The axis must be expressed in the BONE'S OWN frame.
            //
            // `twist` is post-multiplied onto a local rotation, so its axis is read in the bone's
            // local space -- not in world space, which is where `axis` comes from. Feeding the
            // world axis here made the twist rotate the bone OFF its measured direction, which the
            // "direction ok" checks caught: the forearm ended up pointing somewhere else entirely.
            // Undoing the swing first maps the world axis into the local frame.
            Vec3 localAxis = (swing.Conjugate() * axis).Normalized();
            if (!localAxis.IsFinite() || localAxis.LengthSquared < 1e-10f) { return false; }

            twist = Quat.AxisAngle(localAxis, angle);
            return twist.IsFinite();
        }

        private BoneDefinition def_placeholder_for_rest(string name)
        {
            for (int i = 0; i < Definitions.Length; i++)
            {
                if (Definitions[i].Name == name) { return Definitions[i]; }
            }
            return Definitions[0];
        }

        /// <summary>
        /// The thumb's offset from a forearm's axis, as a unit direction perpendicular to it.
        ///
        /// This is the reference the forearm's twist is measured against. It is taken from the
        /// skeleton rather than assumed, so it is correct for whichever side and whatever pose it
        /// is measured in.
        /// </summary>
        private bool TryBuildThumbReference(SkeletonFrameData frame, string forearmBone,
            Vec3 axis, out Vec3 reference)
        {
            reference = default(Vec3);

            bool left = forearmBone == VmcBones.LeftLowerArm;
            KinectJoint wrist = left ? KinectJoint.WristLeft : KinectJoint.WristRight;
            KinectJoint thumb = left ? KinectJoint.ThumbLeft : KinectJoint.ThumbRight;

            Vec3 toThumb = frame.Get(thumb) - frame.Get(wrist);
            if (!axis.IsFinite() || axis.LengthSquared < 1e-8f) { return false; }
            if (!toThumb.IsFinite() || toThumb.LengthSquared < 1e-8f) { return false; }

            Vec3 perpendicular = toThumb - axis * Vec3.Dot(toThumb, axis);
            if (!perpendicular.IsFinite() || perpendicular.LengthSquared < 1e-10f) { return false; }

            reference = perpendicular.Normalized();
            return true;
        }

        /// <summary>
        /// Rotation from a parent joint to a child, as Kinect reports their orientations.
        ///
        /// Two conventions are common and picking the wrong one gives a rotation that looks
        /// plausible but is wrong, so it is switchable rather than guessed:
        ///   false (default) = child * parent^-1, the standard relative-rotation form
        ///   true            = parent^-1 * child, the form game engines use for bone locals
        /// </summary>
        public bool UseEngineBoneOrder;

        /// <summary>
        /// Use Kinect's own joint orientations instead of inferring the palm's roll.
        ///
        /// The solver otherwise works from joint POSITIONS, and a position says nothing about
        /// rotation about a bone's own axis. That is exactly why the palm could not be oriented:
        /// a shortest-arc swing introduces no twist, so with the arm hanging the upper arm carried
        /// the forearm and hand round rigidly and the palm faced outwards.
        ///
        /// Kinect v2 reports each joint's full orientation (Body.JointOrientations). Reading it is
        /// the difference between knowing the roll and guessing it from the thumb joint -- and
        /// every version of the guess landed a constant 180 degrees out on one arm.
        /// </summary>
        public bool UseJointOrientations = true;

        private bool TryBuildJointRotation(SkeletonFrameData frame, KinectJoint parent,
            KinectJoint child, out Quat rotation)
        {
            rotation = Quat.Identity;

            if (!frame.IsOriented(parent) || !frame.IsOriented(child)) { return false; }

            Quat p = frame.GetOrientation(parent);
            Quat c = frame.GetOrientation(child);

            rotation = UseEngineBoneOrder ? (p.Conjugate() * c) : (c * p.Conjugate());
            return rotation.IsFinite();
        }

        private bool TryBuildPalmTwist(SkeletonFrameData frame, string handBone, out Quat twist)
        {
            twist = Quat.Identity;

            bool left = handBone == VmcBones.LeftHand;
            KinectJoint wrist = left ? KinectJoint.WristLeft : KinectJoint.WristRight;
            KinectJoint tip = left ? KinectJoint.HandTipLeft : KinectJoint.HandTipRight;
            KinectJoint thumb = left ? KinectJoint.ThumbLeft : KinectJoint.ThumbRight;

            Vec3 forward = frame.Get(tip) - frame.Get(wrist);
            Vec3 toThumb = frame.Get(thumb) - frame.Get(wrist);
            if (!forward.IsFinite() || forward.LengthSquared < 1e-8f
                || !toThumb.IsFinite() || toThumb.LengthSquared < 1e-8f)
            {
                return false;
            }

            forward = forward.Normalized();
            Vec3 up = Vec3.Cross(forward, toThumb).Normalized();
            if (!up.IsFinite() || up.LengthSquared < 1e-8f) { return false; }

            // Derive the rest frame from the rest pose's OWN geometry instead of assuming a palm
            // normal. A hard-coded normal does not match every rig, and when it is wrong the twist
            // is a constant 90 or 180 degrees even in the neutral pose -- which is exactly the
            // failure this had. Deriving it makes "neutral pose means no twist" true by
            // construction, whatever the rig's thumb sits.
            Vec3 restForward = CalibratedOrDefault(handBone, left ? CharLeft : CharRight);
            Vec3 restThumb = left
                ? DefaultRestThumbDirection
                : new Vec3(-DefaultRestThumbDirection.X, DefaultRestThumbDirection.Y,
                    DefaultRestThumbDirection.Z);

            // ORDER MATTERS: the triple (right, up, forward) must be right-handed, i.e.
            // right x up = forward, or QuatFromAxes silently returns the identity. Deriving right
            // as forward x up gives a LEFT-handed triple here and produces exactly that silent
            // failure, which is how this shipped once already.
            // BOTH frames must be built the same way, or the difference between them is a sign
            // flip rather than a twist. Building one right-handed and the other left-handed
            // produced a constant 180 degree error even in the neutral pose.
            //
            // Each frame is: up = normalize(fingers x thumb), right = up x forward, and the
            // resulting triple (right, up, forward) is right-handed because right x up = forward.
            Vec3 measuredUp = up;
            Vec3 measuredRight = Vec3.Cross(measuredUp, forward).Normalized();
            if (!measuredRight.IsFinite() || measuredRight.LengthSquared < 1e-8f) { return false; }

            Vec3 restUp = Vec3.Cross(restForward, restThumb).Normalized();
            if (!restUp.IsFinite() || restUp.LengthSquared < 1e-8f) { return false; }
            Vec3 restRight = Vec3.Cross(restUp, restForward).Normalized();

            Quat restFrame, measuredFrame;
            if (!TryBuildRotation(restRight, restUp, restForward, out restFrame)) { return false; }
            if (!TryBuildRotation(measuredRight, measuredUp, forward, out measuredFrame)) { return false; }

            twist = measuredFrame * restFrame.Conjugate();
            return twist.IsFinite();
        }
        private void EmitFingerBones(SkeletonPose destination, string handBone, bool closed,
            SkeletonFrameData frame, KinectJoint wrist, KinectJoint tip, KinectJoint thumb)
        {
            Vec3 forward = frame.Get(tip) - frame.Get(wrist);
            Vec3 toThumb = frame.Get(thumb) - frame.Get(wrist);

            if (!forward.IsFinite() || forward.LengthSquared < 1e-8f
                || !toThumb.IsFinite() || toThumb.LengthSquared < 1e-8f)
            {
                return;
            }

            forward = forward.Normalized();
            Vec3 up = Vec3.Cross(forward, toThumb).Normalized();
            if (!up.IsFinite() || up.LengthSquared < 1e-8f) { return; }

            // (the left/right distinction is no longer needed: the curl axis is the palm normal)

            // Fingers carry the CURL ONLY. The palm's twist belongs on the hand bone, which the
            // fingers inherit: putting it here instead made a wrist turn spin the fingers
            // themselves rather than turning the hand, which is exactly what the user reported.
            //
            // The rotation carries the fingertips towards the palm. The direction was settled by
            // the user trying both on the real model: a negative rotation about forward x toThumb
            // bends the fingers the wrong way at the joint. The sign of that cross product is not
            // a usable guide, because toThumb is not perpendicular to forward.
            Quat curl = closed
                ? Quat.AxisAngle(FingerCurlInward ? up : -up, FingerCurlDegrees * 0.0174532925f)
                : Quat.Identity;

            string[] bones = handBone == VmcBones.LeftHand ? LeftFingerBones : RightFingerBones;
            for (int i = 0; i < bones.Length; i++)
            {
                destination.SetBone(bones[i], curl);
            }
        }


        private static bool IsLegOrHips(string name)
        {
            return name == VmcBones.Hips
                || name == VmcBones.LeftUpperLeg || name == VmcBones.LeftLowerLeg || name == VmcBones.LeftFoot
                || name == VmcBones.RightUpperLeg || name == VmcBones.RightLowerLeg || name == VmcBones.RightFoot;
        }

        private static Vec3 LookupOverride(Dictionary<string, Vec3> overrides, string boneName, Vec3 fallback)
        {
            Vec3 value;
            if (overrides.TryGetValue(boneName, out value)) { return value; }
            return fallback;
        }

        /// <summary>
        /// Builds the body's own orthonormal frame from the hip line and the shoulder line.
        /// </summary>
        private static bool TryBuildCharacterFrame(SkeletonFrameData frame, out Vec3 up, out Vec3 right, out Vec3 forward)
        {
            Vec3 shoulderLeft = frame.Get(KinectJoint.ShoulderLeft);
            Vec3 shoulderRight = frame.Get(KinectJoint.ShoulderRight);
            Vec3 spineBase = frame.Get(KinectJoint.SpineBase);
            Vec3 spineShoulder = frame.Get(KinectJoint.SpineShoulder);

            up = spineShoulder - spineBase;
            if (!up.IsFinite() || up.LengthSquared < 1e-6f)
            {
                up = CharUp;
            }
            up = up.Normalized();

            // In the mirrored, left-handed character space the character's own right hand
            // is at +X when the user faces the sensor.
            Vec3 shoulderAxis = shoulderRight - shoulderLeft;
            if (!shoulderAxis.IsFinite() || shoulderAxis.LengthSquared < 1e-6f)
            {
                shoulderAxis = CharRight;
            }

            // Gram-Schmidt: remove the component of the shoulder axis that lies along `up`.
            right = (shoulderAxis - up * Vec3.Dot(shoulderAxis, up)).Normalized();
            if (right.LengthSquared < 0.5f)
            {
                // Degenerate shoulder line (extremely rare): pick any perpendicular axis.
                Vec3 fallback = Math.Abs(Vec3.Dot(up, CharRight)) < 0.9f ? CharRight : CharBackward;
                right = Vec3.Cross(up, fallback).Normalized();
                if (right.LengthSquared < 0.5f) { right = CharRight; }
            }

            forward = Vec3.Cross(up, right).Normalized();
            if (forward.LengthSquared < 0.5f)
            {
                forward = CharBackward;
            }

            return true;
        }

        /// <summary>
        /// The shortest-arc rotation that re-points <paramref name="from"/> onto
        /// <paramref name="to"/>.
        ///
        /// This is the bone's deviation from its neutral direction, which is precisely the
        /// local pose rotation a bind-pose-relative consumer such as Warudo expects. Using
        /// the minimal swing deliberately discards the bone's own roll: Kinect supplies joint
        /// positions, and its <c>JointOrientation</c> values are too noisy to be worth the
        /// twist information they claim to provide.
        /// </summary>
        /// <summary>
        /// Builds the rotation carrying one right-handed orthonormal frame onto another.
        ///
        /// Used for the palm: a single bone direction cannot express a hand's twist, so the palm
        /// needs two axes. Both frames must be right-handed ordered triples; feeding a left-handed
        /// one silently yields a 180 degree error.
        /// </summary>
        private static bool TryBuildRotation(Vec3 right, Vec3 up, Vec3 forward, out Quat rotation)
        {
            rotation = Quat.Identity;

            if (!right.IsFinite() || !up.IsFinite() || !forward.IsFinite()) { return false; }
            if (right.LengthSquared < 0.5f || up.LengthSquared < 0.5f || forward.LengthSquared < 0.5f)
            {
                return false;
            }

            rotation = QuatFromAxes(right.Normalized(), up.Normalized(), forward.Normalized());
            return rotation.IsFinite();
        }

        private static bool TryBuildRotation(Vec3 from, Vec3 to, out Quat rotation)
        {
            rotation = Quat.Identity;

            Vec3 restAxis = from.Normalized();
            Vec3 target = to.Normalized();
            if (restAxis.LengthSquared < 0.5f || target.LengthSquared < 0.5f) { return false; }

            float dot = Vec3.Dot(restAxis, target);
            if (dot < -1f) { dot = -1f; } else if (dot > 1f) { dot = 1f; }

            if (dot > 0.999999f)
            {
                rotation = Quat.Identity;
            }
            else if (dot < -0.999999f)
            {
                // Antiparallel: every perpendicular axis gives the same 180 degree rotation,
                // so pick one deterministically instead of trusting a degenerate cross product.
                Vec3 axis = Vec3.Cross(restAxis, CharUp);
                if (axis.LengthSquared < 1e-6f) { axis = Vec3.Cross(restAxis, CharRight); }
                rotation = Quat.AxisAngle(axis.Normalized(), (float)Math.PI);
            }
            else
            {
                Vec3 axis = Vec3.Cross(restAxis, target);
                rotation = Quat.AxisAngle(axis, (float)Math.Acos(dot));
            }

            return rotation.IsFinite();
        }

        /// <summary>
        /// Converts an orthonormal basis into a quaternion. Columns are the images of the
        /// local X, Y and Z axes and must satisfy det(x, y, z) = +1, i.e. form a right-handed
        /// ordered triple; feeding it a left-handed one silently yields a 180 degree error.
        ///
        /// The result is returned in canonical form. A quaternion and its negation describe
        /// the same rotation, so they are indistinguishable as rotations, but they are NOT
        /// interchangeable inside the conjugation identities this solver relies on. The trace
        /// algorithm happens to return opposite signs depending on which diagonal element is
        /// largest, which is exactly what happens for the 180 degree rotations that the arm
        /// bones' neutral frames are. Canonicalising removes that ambiguity.
        /// </summary>
        public static Quat QuatFromAxes(Vec3 x, Vec3 y, Vec3 z)
        {
            float m00 = x.X, m01 = y.X, m02 = z.X;
            float m10 = x.Y, m11 = y.Y, m12 = z.Y;
            float m20 = x.Z, m21 = y.Z, m22 = z.Z;

            // Shepperd's method: branch on the largest of trace and the three diagonal
            // elements, which is the numerically stable choice and never divides by zero.
            float trace = m00 + m11 + m22;
            Quat q;

            if (trace > 0f)
            {
                float s = (float)Math.Sqrt(trace + 1f) * 2f;
                q = new Quat((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s);
            }
            else if (m00 >= m11 && m00 >= m22)
            {
                float s = (float)Math.Sqrt(1f + m00 - m11 - m22) * 2f;
                q = new Quat(0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s);
            }
            else if (m11 >= m22)
            {
                float s = (float)Math.Sqrt(1f + m11 - m00 - m22) * 2f;
                q = new Quat((m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s);
            }
            else
            {
                float s = (float)Math.Sqrt(1f + m22 - m00 - m11) * 2f;
                q = new Quat((m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s);
            }

            q = q.Normalized();
            return Canonicalize(q);
        }

        /// <summary>
        /// Picks one of the two quaternions that describe a rotation: the one whose largest
        /// component is positive. Used so that round-tripping a rotation through
        /// <see cref="QuatFromAxes"/> is deterministic.
        /// </summary>
        public static Quat Canonicalize(Quat q)
        {
            float x = Math.Abs(q.X), y = Math.Abs(q.Y), z = Math.Abs(q.Z), w = Math.Abs(q.W);
            if (w >= x && w >= y && w >= z)
            {
                return q.W < 0f ? new Quat(-q.X, -q.Y, -q.Z, -q.W) : q;
            }
            if (x >= y && x >= z)
            {
                return q.X < 0f ? new Quat(-q.X, -q.Y, -q.Z, -q.W) : q;
            }
            if (y >= z)
            {
                return q.Y < 0f ? new Quat(-q.X, -q.Y, -q.Z, -q.W) : q;
            }
            return q.Z < 0f ? new Quat(-q.X, -q.Y, -q.Z, -q.W) : q;
        }

        private Vec3 ResolvePosition(SkeletonFrameData frame, KinectJoint joint)
        {
            int index = (int)joint;
            if (frame.Tracked[index])
            {
                _lastGoodPosition[joint.ToString()] = frame.Positions[index];
                return frame.Positions[index];
            }

            Vec3 remembered;
            if (_lastGoodPosition.TryGetValue(joint.ToString(), out remembered))
            {
                return remembered;
            }
            return frame.Positions[index];
        }

        private Quat ApplySmoothing(string boneName, Quat target)
        {
            if (Smoothing <= 0.0001f)
            {
                _smoothed[boneName] = target;
                return target;
            }

            Quat previous;
            if (!_smoothed.TryGetValue(boneName, out previous))
            {
                _smoothed[boneName] = target;
                return target;
            }

            // Exponential smoothing expressed as a slerp step; independent of frame rate
            // only in the sense that it is re-tuned by the user, which is the honest
            // trade-off for a 30 Hz source feeding a 60 Hz renderer.
            Quat blended = Quat.Slerp(previous, target, 1f - Smoothing);
            _smoothed[boneName] = blended;
            return blended;
        }
    }
}
