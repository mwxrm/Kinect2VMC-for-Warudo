using System;
using Kinect2VMC.Core;
using Kinect2VMC.SelfTest;

namespace Kinect2VMC.UiSmoke
{
    /// <summary>
    /// Quantiifes the trade-off the user is hitting: which neutral pose the solver's reference
    /// directions come from decides which arm motion is geometrically clean and which one picks
    /// up a roll about a tilted axis.
    ///
    /// The solver reproduces each measured direction exactly either way; what differs is the
    /// TWIST the shortest arc happens to introduce. Since Kinect supplies joint positions only,
    /// a bone's roll about its own axis has no unique answer, so this cannot be "fixed" -- it can
    /// only be aimed at the motion the user actually performs.
    /// </summary>
    internal static class ReferencePoseStudy
    {
        public static int Run()
        {
            Console.WriteLine();
            Console.WriteLine("Neutral-pose trade-off (forearm twist by motion plane)");
            Console.WriteLine("  基准姿势        | 运动              | 小臂滚转");

            // Rest directions in character space: the arm points along -X (the character's own
            // left is at negative X), and a hanging arm is 45 degrees below horizontal.
            Vec3 tPoseRest = new Vec3(-1f, 0f, 0f).Normalized();
            Vec3 hangingRest = new Vec3(-0.707f, -0.707f, 0f).Normalized();

            // The two motions the user performs, as forearm directions in character space.
            // "forward" is +Z because the character faces the camera.
            Vec3 forward = new Vec3(-0.35f, 0f, 0.94f).Normalized();
            Vec3 swingDown = new Vec3(-0.2f, -0.98f, 0f).Normalized();

            Report("T字水平", "向前伸", tPoseRest, forward);
            Report("T字水平", "向下摆", tPoseRest, swingDown);
            Report("静息下垂", "向前伸", hangingRest, forward);
            Report("静息下垂", "向下摆", hangingRest, swingDown);

            Console.WriteLine();
            Console.WriteLine("  判读：滚转越小越干净。同一基准姿势不可能对两种运动都最小。");

            return 0;
        }

        private static void Report(string poseName, string motionName, Vec3 rest, Vec3 target)
        {
            // The shortest arc that carries rest onto target, as the solver computes it.
            float dot = Vec3.Dot(rest, target);
            if (dot > 1f) { dot = 1f; }
            if (dot < -1f) { dot = -1f; }

            Vec3 axis = Vec3.Cross(rest, target);
            if (axis.LengthSquared < 1e-9f)
            {
                Console.WriteLine("  " + poseName.PadRight(14) + " | " + motionName.PadRight(17) + " | 无旋转");
                return;
            }

            Quat swing = Quat.AxisAngle(axis, (float)Math.Acos(dot));

            // Roll about the bone's own axis shows up as a rotation of the bone's local reference
            // axis that is perpendicular to the bone direction.
            Vec3 localReference = Vec3.Cross(rest, Vec3.Up).Normalized();
            Vec3 rotatedReference = (swing * localReference).Normalized();

            // Remove the component along the bone so only the roll remains.
            Vec3 rollComponent = rotatedReference - target * Vec3.Dot(rotatedReference, target);
            float rollDegrees = 0f;
            if (rollComponent.LengthSquared > 1e-9f)
            {
                Vec3 a = rollComponent.Normalized();
                Vec3 b = Vec3.Cross(localReference, rest).Normalized();
                float c = Vec3.Dot(a, b);
                if (c > 1f) { c = 1f; }
                if (c < -1f) { c = -1f; }
                rollDegrees = (float)(Math.Acos(c) * 57.29577951308232);
            }

            Console.WriteLine("  " + poseName.PadRight(14) + " | " + motionName.PadRight(17) + " | "
                + rollDegrees.ToString("F1").PadLeft(6) + "°");
        }
    }
}
