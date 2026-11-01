using System;
using Kinect2VMC.Core;

namespace Kinect2VMC.SelfTest
{
    /// <summary>
    /// Synthesises Kinect joint frames so the solver can be exercised without a sensor.
    ///
    /// Two coordinate spaces are involved and mixing them up is the classic mistake here:
    ///
    ///  * KINECT CAMERA SPACE - what the sensor reports. +X is the viewer's right, +Y is up,
    ///    +Z points away from the sensor, metres. Because the user faces the sensor, the
    ///    user's own LEFT ends up at NEGATIVE X.
    ///
    ///  * CHARACTER SPACE - the solver's space, after the X mirror that converts the sensor's
    ///    right-handed frame into Unity's left-handed one. +X is the CHARACTER's right, +Y is
    ///    up, +Z is the direction the character faces. A character facing the sensor faces
    ///    -Z, so the user's own LEFT is at NEGATIVE X here as well.
    ///
    /// So in both spaces the user's left is at negative X. That is a useful sanity check: the
    /// Kinect's <c>ShoulderLeft</c> joint must land at negative X in character space too.
    ///
    /// Arm joints are placed at exact angles and lengths so the expected rotations can be
    /// computed analytically instead of eyeballed.
    /// </summary>
    public static class TestSkeleton
    {
        // Torso landmarks in character space. The character faces -Z (towards the sensor), so
        // the spine leans very slightly forward as it rises, as a seated person does.
        //
        // The spine is kept exactly vertical, matching the solver's declared neutral spine
        // direction of (0, 1, 0). That is what makes the built-in neutral pose the identity
        // rotation for the spine chain; a tilted synthetic spine would correctly report that
        // tilt as a constant offset, which would make the identity check meaningless. Real
        // users who sit differently are handled by calibration.
        public const float SpineBaseY = 0.82f;
        public const float SpineMidY = 0.98f;
        public const float SpineShoulderY = 1.14f;
        public const float NeckY = 1.30f;
        public const float HeadY = 1.43f;

        // All torso landmarks share one depth, so the body's forward axis is exactly -Z and
        // the shoulder line is exactly horizontal.
        public const float TorsoZ = 1.40f;

        // The shoulder line sits at the top of the spine, where the arms attach.
        public const float ShoulderY = 1.14f;
        public const float ShoulderHalfWidth = 0.20f;

        public const float UpperArmLength = 0.28f;
        public const float LowerArmLength = 0.25f;
        public const float HandLength = 0.09f;

        /// <summary>
        /// The solver's built-in default rest direction for the left arm bone. Taken from the
        /// solver itself so the two can never drift apart: a horizontal arm out to the side.
        /// </summary>
        public static Vec3 RestArmDirection()
        {
            return UpperBodySolver.DefaultArmRestDirection;
        }

        /// <summary>
        /// The right arm's rest direction: the exact mirror of the left one, taken from the
        /// solver's own constant rather than recomputed from literals.
        ///
        /// Recomputing the mirror here would introduce a rounding difference from the solver's
        /// value, which the identity assertions would rightly report as a small but real rotation
        /// error. The test geometry and the solver must share one source of truth.
        /// </summary>
        public static Vec3 RestArmDirectionRight()
        {
            Vec3 left = UpperBodySolver.DefaultArmRestDirection;
            return new Vec3(-left.X, left.Y, left.Z);
        }

        /// <summary>
        /// A seated adult with the arms hanging down, which is the solver's built-in default arm
        /// rest direction and also the bind pose of Warudo/VRM characters. That makes this pose
        /// the exact identity rotation for every bone, so a deviation measured from it is a real
        /// deviation rather than a mismatch between two different conventions.
        ///
        /// The arms are not perfectly vertical: a natural seated/standing A-pose holds them
        /// slightly out from the body. That matters for the yaw test below, where a fully vertical
        /// arm would lie exactly along the turn axis and could not move at all.
        /// </summary>
        public static SkeletonFrameData RestPose()
        {
            SkeletonFrameData frame = NeutralTorso();
            PlaceArms(frame, RestArmDirection());
            return frame;
        }

        /// <summary>Torso, head and legs only; the arms are left at the shoulder joints.</summary>
        public static SkeletonFrameData NeutralTorso()
        {
            var frame = new SkeletonFrameData { BodyTracked = true, TrackingId = 1, Time = 0f };

            // The spine is a straight line from the pelvis to the neck, which is what the
            // torso's own up axis is derived from. Keeping the chain collinear means the
            // neutral pose really is the solver's identity pose; a bent synthetic chain would
            // legitimately show up as a constant offset rotation.
            frame.Set(KinectJoint.SpineBase, new Vec3(0f, SpineBaseY, TorsoZ), true);
            frame.Set(KinectJoint.SpineMid, new Vec3(0f, SpineMidY, TorsoZ), true);
            frame.Set(KinectJoint.SpineShoulder, new Vec3(0f, SpineShoulderY, TorsoZ), true);
            frame.Set(KinectJoint.Neck, new Vec3(0f, NeckY, TorsoZ), true);
            frame.Set(KinectJoint.Head, new Vec3(0f, HeadY, TorsoZ), true);

            // The user's own left is at negative X, in character space as in camera space.
            frame.Set(KinectJoint.ShoulderLeft, new Vec3(-ShoulderHalfWidth, ShoulderY, TorsoZ), true);
            frame.Set(KinectJoint.ShoulderRight, new Vec3(ShoulderHalfWidth, ShoulderY, TorsoZ), true);

            // Seated lower body: unreliable in reality, irrelevant to the upper-body solver.
            frame.Set(KinectJoint.HipLeft, new Vec3(-0.12f, 0.78f, 1.40f), true);
            frame.Set(KinectJoint.KneeLeft, new Vec3(-0.14f, 0.46f, 1.72f), true);
            frame.Set(KinectJoint.AnkleLeft, new Vec3(-0.14f, 0.12f, 1.80f), true);
            frame.Set(KinectJoint.FootLeft, new Vec3(-0.14f, 0.06f, 1.92f), true);
            frame.Set(KinectJoint.HipRight, new Vec3(0.12f, 0.78f, 1.40f), true);
            frame.Set(KinectJoint.KneeRight, new Vec3(0.14f, 0.46f, 1.72f), true);
            frame.Set(KinectJoint.AnkleRight, new Vec3(0.14f, 0.12f, 1.80f), true);
            frame.Set(KinectJoint.FootRight, new Vec3(0.14f, 0.06f, 1.92f), true);

            return frame;
        }

        /// <summary>
        /// Places both arm chains from a left-arm direction in character space. The right side
        /// is mirrored, so a symmetric pose is a single input.
        /// </summary>
        public static void PlaceArms(SkeletonFrameData frame, Vec3 leftDirection)
        {
            Vec3 left = leftDirection.Normalized();
            Vec3 right = new Vec3(-left.X, left.Y, left.Z);
            PlaceArm(frame, true, left, left);
            PlaceArm(frame, false, right, right);
        }

        /// <summary>
        /// Places one arm chain from explicit upper-arm and forearm directions, in character
        /// space. Each segment is placed by scaling the unit direction, so the direction
        /// measured from any parent joint to its child is exactly the one requested.
        /// </summary>
        public static void PlaceArm(SkeletonFrameData frame, bool isLeft,
            Vec3 upperDirection, Vec3 lowerDirection)
        {
            KinectJoint shoulder = isLeft ? KinectJoint.ShoulderLeft : KinectJoint.ShoulderRight;
            KinectJoint elbow = isLeft ? KinectJoint.ElbowLeft : KinectJoint.ElbowRight;
            KinectJoint wrist = isLeft ? KinectJoint.WristLeft : KinectJoint.WristRight;
            KinectJoint hand = isLeft ? KinectJoint.HandLeft : KinectJoint.HandRight;
            KinectJoint tip = isLeft ? KinectJoint.HandTipLeft : KinectJoint.HandTipRight;
            KinectJoint thumb = isLeft ? KinectJoint.ThumbLeft : KinectJoint.ThumbRight;

            Vec3 upper = upperDirection.Normalized();
            Vec3 lower = lowerDirection.Normalized();

            Vec3 shoulderPosition = frame.Get(shoulder);
            Vec3 elbowPosition = shoulderPosition + upper * UpperArmLength;
            frame.Set(elbow, elbowPosition, true);

            Vec3 wristPosition = elbowPosition + lower * LowerArmLength;
            frame.Set(wrist, wristPosition, true);

            // The hand and fingertip continue along the forearm, so the hand bone direction
            // matches the lower arm. Kinect's own Hand joint sits almost on the wrist.
            Vec3 handPosition = wristPosition + lower * HandLength;
            frame.Set(hand, handPosition, true);
            frame.Set(tip, handPosition + lower * 0.04f, true);
            frame.Set(thumb, wristPosition + lower * (LowerArmLength * 0.45f)
                + new Vec3(0f, 0f, isLeft ? 0.035f : -0.035f), true);
        }

        /// <summary>
        /// Kinect camera space to character space: negate Z, exactly as
        /// <c>KinectBodySource.ToCharacterSpace</c> does.
        ///
        /// This must stay identical to the adapter. When the two drifted apart in the past, the
        /// suite agreed with a mirrored adapter and could not see the bug: raising an arm
        /// measured as lowering it while every assertion stayed green.
        ///
        /// Only Z is negated. The user faces the sensor, so the character's forward is towards
        /// decreasing camera-space Z.
        /// </summary>
        public static Vec3 ToCharacterSpace(Vec3 kinectSpace)
        {
            return new Vec3(kinectSpace.X, kinectSpace.Y, -kinectSpace.Z);
        }

        /// <summary>Applies a rotation about the world origin, then a translation.</summary>
        public static SkeletonFrameData Transform(SkeletonFrameData source, Quat rotation, Vec3 translation)
        {
            var result = new SkeletonFrameData
            {
                BodyTracked = source.BodyTracked,
                TrackingId = source.TrackingId,
                Time = source.Time
            };

            for (int i = 0; i < SkeletonFrameData.JointCount; i++)
            {
                result.Positions[i] = (rotation * source.Positions[i]) + translation;
                result.Tracked[i] = source.Tracked[i];
            }
            return result;
        }

        /// <summary>Copies a frame so individual joints can be moved.</summary>
        public static SkeletonFrameData Clone(SkeletonFrameData source)
        {
            return Transform(source, Quat.Identity, Vec3.Zero);
        }

        /// <summary>Reads the normalised direction from one joint to another.</summary>
        public static Vec3 Direction(SkeletonFrameData frame, KinectJoint from, KinectJoint to)
        {
            return (frame.Get(to) - frame.Get(from)).Normalized();
        }
    }
}
