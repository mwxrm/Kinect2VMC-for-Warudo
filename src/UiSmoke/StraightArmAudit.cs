using System;
using System.Collections.Generic;
using Kinect2VMC.Core;

namespace Kinect2VMC.UiSmoke
{
    /// <summary>
    /// Checks the one thing the user's report points at and that no existing test covered:
    /// whether a STRAIGHT arm stays straight.
    ///
    /// Reaching forward with the elbow extended gives the upper arm, forearm and hand the SAME
    /// direction. The solver is expected to reproduce each measured direction exactly, so the
    /// three avatar bones must also share one direction -- any constant angle between consecutive
    /// bones means the chain is being solved wrongly, not that the rig differs.
    ///
    /// This runs purely in character space, so it isolates the solver from the adapter.
    /// </summary>
    internal static class StraightArmAudit
    {
        public static int Run()
        {
            int failures = 0;
            Console.WriteLine();
            Console.WriteLine("Straight-arm audit (does a straight arm stay straight?)");

            var solver = new UpperBodySolver { Smoothing = 0f, SendHead = false };

            // Directions to try, in character space. +Z is forward (towards the camera), -X is the
            // character's own left, +Y is up.
            var targets = new List<KeyValuePair<string, Vec3>>
            {
                new KeyValuePair<string, Vec3>("forward", new Vec3(0f, 0f, 1f)),
                new KeyValuePair<string, Vec3>("forward-up", new Vec3(0f, 0.5f, 0.866f)),
                new KeyValuePair<string, Vec3>("left", new Vec3(-1f, 0f, 0f)),
                new KeyValuePair<string, Vec3>("down", new Vec3(0f, -1f, 0f)),
                new KeyValuePair<string, Vec3>("forward-left", new Vec3(-0.6f, 0f, 0.8f))
            };

            foreach (var entry in targets)
            {
                Vec3 direction = entry.Value.Normalized();

                // A straight left arm pointing in `direction`: shoulder, elbow, wrist and hand all
                // lie on one ray.
                SkeletonFrameData frame = StraightLeftArm(direction);

                var pose = new SkeletonPose();
                if (!solver.Solve(frame, pose))
                {
                    Console.WriteLine("  [FAIL] " + entry.Key.PadRight(14) + " frame did not solve");
                    failures++;
                    continue;
                }

                // The avatar's own rest directions for the left arm, as the solver sees them.
                Vec3 restUpperArm = solver.ArmRestDirectionLeft;
                Vec3 restForearm = solver.ArmRestDirectionLeft;

                // Compose each bone's LOCAL rotation with its ancestors to get its WORLD rotation:
                // VMC carries local rotations, so comparing the raw emitted value against a
                // measured direction would be comparing two different things.
                Vec3 avatarUpperArm = (WorldRotation(pose, VmcBones.LeftUpperArm) * restUpperArm).Normalized();
                Vec3 avatarForearm = (WorldRotation(pose, VmcBones.LeftLowerArm) * restForearm).Normalized();

                float measuredAngle = AngleDegrees(avatarUpperArm, avatarForearm);

                // The solver reports WORLD rotations, so consecutive bones sharing a world rotation
                // means the avatar arm is straight. A large angle here is a solver defect.
                bool straight = measuredAngle < 1.0f;

                // Also verify each bone lands on the measured direction at all.
                Vec3 measuredDirection = (frame.Get(KinectJoint.ElbowLeft)
                    - frame.Get(KinectJoint.ShoulderLeft)).Normalized();
                float upperError = AngleDegrees(avatarUpperArm, measuredDirection);
                float forearmError = AngleDegrees(avatarForearm, measuredDirection);

                if (!straight || upperError > 1.0f || forearmError > 1.0f) { failures++; }

                Console.WriteLine("  " + (straight && upperError <= 1f && forearmError <= 1f ? "[ok]  " : "[FAIL]")
                    + " " + entry.Key.PadRight(14)
                    + " 上臂-小臂夹角 " + measuredAngle.ToString("F2").PadLeft(6) + "°"
                    + "   上臂误差 " + upperError.ToString("F2").PadLeft(6) + "°"
                    + "   小臂误差 " + forearmError.ToString("F2").PadLeft(6) + "°");
            }

            Console.WriteLine();
            Console.WriteLine("  说明：若夹角接近 0，解算器是自洽的，问题在角色的绑定姿势；");
            Console.WriteLine("        若夹角明显不为 0，则是解算器缺陷。");

            return failures;
        }

        /// <summary>A left arm with no elbow bend, pointing along <paramref name="direction"/>.</summary>
        private static SkeletonFrameData StraightLeftArm(Vec3 direction)
        {
            var frame = new SkeletonFrameData { BodyTracked = true, TrackingId = 1, Time = 0f };

            frame.Set(KinectJoint.SpineBase, new Vec3(0f, 0.82f, -1.42f), true);
            frame.Set(KinectJoint.SpineMid, new Vec3(0f, 0.98f, -1.40f), true);
            frame.Set(KinectJoint.SpineShoulder, new Vec3(0f, 1.14f, -1.39f), true);
            frame.Set(KinectJoint.Neck, new Vec3(0f, 1.22f, -1.39f), true);
            frame.Set(KinectJoint.Head, new Vec3(0f, 1.35f, -1.38f), true);

            // The character's own left is at negative X in character space.
            Vec3 shoulder = new Vec3(-0.20f, 1.12f, -1.39f);
            frame.Set(KinectJoint.ShoulderLeft, shoulder, true);
            frame.Set(KinectJoint.ShoulderRight, new Vec3(0.20f, 1.12f, -1.39f), true);

            Vec3 unit = direction.Normalized();
            Vec3 elbow = shoulder + unit * 0.28f;
            Vec3 wrist = elbow + unit * 0.25f;
            Vec3 hand = wrist + unit * 0.09f;

            frame.Set(KinectJoint.ElbowLeft, elbow, true);
            frame.Set(KinectJoint.WristLeft, wrist, true);
            frame.Set(KinectJoint.HandLeft, hand, true);
            frame.Set(KinectJoint.HandTipLeft, hand + unit * 0.04f, true);
            frame.Set(KinectJoint.ThumbLeft, hand, true);

            // Keep the right arm at a plausible hanging rest so the torso frame is stable.
            frame.Set(KinectJoint.ElbowRight, new Vec3(0.40f, 0.92f, -1.39f), true);
            frame.Set(KinectJoint.WristRight, new Vec3(0.58f, 0.74f, -1.39f), true);
            frame.Set(KinectJoint.HandRight, new Vec3(0.64f, 0.68f, -1.39f), true);
            frame.Set(KinectJoint.HandTipRight, new Vec3(0.68f, 0.64f, -1.39f), true);
            frame.Set(KinectJoint.ThumbRight, new Vec3(0.62f, 0.66f, -1.36f), true);

            frame.Set(KinectJoint.HipLeft, new Vec3(-0.12f, 0.78f, -1.40f), true);
            frame.Set(KinectJoint.HipRight, new Vec3(0.12f, 0.78f, -1.40f), true);

            return frame;
        }

        /// <summary>The parent chain the solver uses, mirroring UpperBodySolver's definitions.</summary>
        private static readonly Dictionary<string, string> Parents =
            new Dictionary<string, string>(StringComparer.Ordinal)
        {
            { VmcBones.Spine, null },
            { VmcBones.Chest, VmcBones.Spine },
            { VmcBones.UpperChest, VmcBones.Chest },
            { VmcBones.Neck, VmcBones.UpperChest },
            { VmcBones.Head, VmcBones.Neck },
            { VmcBones.LeftShoulder, VmcBones.UpperChest },
            { VmcBones.LeftUpperArm, VmcBones.LeftShoulder },
            { VmcBones.LeftLowerArm, VmcBones.LeftUpperArm },
            { VmcBones.LeftHand, VmcBones.LeftLowerArm },
            { VmcBones.RightShoulder, VmcBones.UpperChest },
            { VmcBones.RightUpperArm, VmcBones.RightShoulder },
            { VmcBones.RightLowerArm, VmcBones.RightUpperArm },
            { VmcBones.RightHand, VmcBones.RightLowerArm }
        };

        /// <summary>
        /// A bone's world rotation: its own local rotation composed with every ancestor's.
        /// The solver emits local rotations because that is what VMC carries.
        /// </summary>
        internal static Quat WorldRotation(SkeletonPose pose, string bone)
        {
            Quat local;
            if (!pose.BoneRotations.TryGetValue(bone, out local)) { return Quat.Identity; }

            string parent = Parents[bone];
            if (parent == null) { return local; }
            if (!pose.BoneRotations.ContainsKey(parent)) { return local; }

            return WorldRotation(pose, parent) * local;
        }
        private static float AngleDegrees(Vec3 a, Vec3 b)
        {
            float dot = Vec3.Dot(a.Normalized(), b.Normalized());
            if (dot > 1f) { dot = 1f; }
            if (dot < -1f) { dot = -1f; }
            return (float)(Math.Acos(dot) * 57.29577951308232);
        }
    }
}
