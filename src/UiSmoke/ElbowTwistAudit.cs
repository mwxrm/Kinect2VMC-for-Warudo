using System;
using System.Collections.Generic;
using System.Text;
using Kinect2VMC.Core;
using Kinect2VMC.SelfTest;

namespace Kinect2VMC.UiSmoke
{
    /// <summary>
    /// Measures a risk that had only ever been argued about, never quantified: the solver picks
    /// the minimal swing for every bone independently, so the forearm's roll comes from the
    /// body's up axis rather than from the plane the elbow actually bends in.
    ///
    /// This harness poses an arm with a bent elbow, then reports the twist the solver applies
    /// between the upper arm and the forearm. A small, pose-independent number means the
    /// approximation is harmless; a large or wildly varying one means it would be visible on
    /// the avatar and belongs in the documented limitations.
    /// </summary>
    internal static class ElbowTwistAudit
    {
        public static int Run()
        {
            int failures = 0;
            Console.WriteLine();
            Console.WriteLine("Elbow twist audit (upper arm vs forearm)");

            // A straight-arm reference, so the twist introduced by a bend is isolated.
            double[] bends = { 0.0, 30.0, 60.0, 90.0 };

            // Upper-arm directions that sweep through the space a seated user actually uses:
            // hanging, out to the side, raised, and reaching forward.
            var upperDirections = new List<KeyValuePair<string, Vec3>>
            {
                new KeyValuePair<string, Vec3>("down-out",  new Vec3(0.5f, -0.866f, 0f)),
                new KeyValuePair<string, Vec3>("sideways",  new Vec3(-1f, 0f, 0f)),
                new KeyValuePair<string, Vec3>("up-out",    new Vec3(-0.5f, 0.866f, 0f)),
                new KeyValuePair<string, Vec3>("forward",   new Vec3(0f, 0f, 1f)),
                new KeyValuePair<string, Vec3>("up-forward", new Vec3(0f, 0.707f, 0.707f))
            };

            var solver = new UpperBodySolver { Smoothing = 0f };

            Console.WriteLine("  弯曲角 | 上臂朝向      | 前臂相对扭转角");
            Console.WriteLine("  -------+---------------+---------------");

            float worst = 0f;
            string worstCase = null;
            float maxSpread = 0f;
            string spreadCase = null;

            foreach (var upper in upperDirections)
            {
                float minTwist = float.MaxValue;
                float maxTwist = float.MinValue;

                foreach (double bendDegrees in bends)
                {
                    SkeletonFrameData frame = TestSkeleton.NeutralTorso();

                    Vec3 upperDir = upper.Value.Normalized();
                    // Bend the forearm within the plane spanned by the upper arm and world up,
                    // which is how a real elbow behaves.
                    Vec3 bendAxis = Vec3.Cross(upperDir, Vec3.Up);
                    if (bendAxis.LengthSquared < 1e-6f) { bendAxis = Vec3.Cross(upperDir, Vec3.Forward); }
                    bendAxis = bendAxis.Normalized();

                    Quat bend = Quat.AxisAngle(bendAxis, (float)(bendDegrees * Math.PI / 180.0));
                    Vec3 lowerDir = (bend * upperDir).Normalized();

                    TestSkeleton.PlaceArm(frame, true, upperDir, lowerDir);

                    var pose = new SkeletonPose();
                    if (!solver.Solve(frame, pose))
                    {
                        Console.WriteLine("  " + bendDegrees.ToString("F0").PadLeft(6) + " | "
                            + upper.Key.PadRight(13) + " | 求解失败");
                        failures++;
                        continue;
                    }

                    float twist = RelativeTwist(pose, VmcBones.LeftUpperArm, VmcBones.LeftLowerArm);

                    Console.WriteLine("  " + bendDegrees.ToString("F0").PadLeft(6) + " | "
                        + upper.Key.PadRight(13) + " | " + twist.ToString("F1").PadLeft(6) + "°");

                    if (twist > worst) { worst = twist; worstCase = upper.Key + " @ " + bendDegrees + "°"; }
                    if (twist < minTwist) { minTwist = twist; }
                    if (twist > maxTwist) { maxTwist = twist; }
                }

                float spread = maxTwist - minTwist;
                if (spread > maxSpread) { maxSpread = spread; spreadCase = upper.Key; }
            }

            Console.WriteLine();
            Console.WriteLine("  结论数据：");
            Console.WriteLine("    最大前臂相对扭转 : " + worst.ToString("F1") + "°  (" + worstCase + ")");
            Console.WriteLine("    同一上臂朝向下的扭转变化幅度 : " + maxSpread.ToString("F1") + "°  (" + spreadCase + ")");

            // A bend within one plane should not need any forearm twist at all. Anything beyond a
            // few degrees is the approximation showing through.
            if (worst > 25f)
            {
                Console.WriteLine("    >>> 扭转较大，需要在文档里如实写明（属于已知近似）");
            }
            else if (worst > 8f)
            {
                Console.WriteLine("    >>> 扭转轻微，通常不可见，但值得记录");
            }
            else
            {
                Console.WriteLine("    >>> 扭转很小，最小 swing 近似在该姿态范围内是安全的");
            }

            // The solver must still reproduce every measured direction exactly; a twist
            // measurement is only meaningful if the underlying directions are correct.
            failures += VerifyDirectionsStillExact(solver);
            failures += MeasureSmoothness(solver);

            return failures;
        }

        /// <summary>
        /// Sweeps the upper arm through the frontal plane and reports how the forearm twist
        /// changes. A smooth, small number means the approximation cannot produce a visible snap;
        /// a jump means the wrist would visibly pop between poses.
        /// </summary>
        private static int MeasureSmoothness(UpperBodySolver solver)
        {
            Console.WriteLine();
            Console.WriteLine("  平滑性：上臂在额状面内扫过 180°，每 15° 采一次前臂扭转");

            float previous = float.NaN;
            float largestStep = 0f;
            float largestStepAt = 0f;
            var samples = new List<float>();

            for (int degrees = 0; degrees <= 180; degrees += 15)
            {
                double radians = degrees * Math.PI / 180.0;
                // Up-out at 0 degrees, swinging through sideways to down-out at 180.
                Vec3 upperDir = new Vec3(-(float)Math.Cos(radians), (float)Math.Sin(radians), 0f);
                if (upperDir.LengthSquared < 1e-6f) { continue; }

                Vec3 bendAxis = Vec3.Cross(upperDir, Vec3.Up);
                if (bendAxis.LengthSquared < 1e-6f) { bendAxis = Vec3.Cross(upperDir, Vec3.Forward); }
                Quat bend = Quat.AxisAngle(bendAxis.Normalized(), 60f * (float)Math.PI / 180f);
                Vec3 lowerDir = (bend * upperDir).Normalized();

                SkeletonFrameData frame = TestSkeleton.NeutralTorso();
                TestSkeleton.PlaceArm(frame, true, upperDir, lowerDir);

                var pose = new SkeletonPose();
                if (!solver.Solve(frame, pose)) { continue; }

                float twist = RelativeTwist(pose, VmcBones.LeftUpperArm, VmcBones.LeftLowerArm);
                samples.Add(twist);

                if (!float.IsNaN(previous))
                {
                    float step = Math.Abs(twist - previous);
                    if (step > largestStep) { largestStep = step; largestStepAt = degrees; }
                }
                previous = twist;
            }

            Console.WriteLine("    样本数 " + samples.Count
                + "，最大相邻跳变 " + largestStep.ToString("F1") + "° （出现在 " + largestStepAt.ToString("F0") + "° 处）");

            if (samples.Count > 0)
            {
                float min = float.MaxValue, max = float.MinValue;
                foreach (float s in samples) { if (s < min) { min = s; } if (s > max) { max = s; } }
                Console.WriteLine("    扭转范围 " + min.ToString("F1") + "° ~ " + max.ToString("F1") + "°");
            }

            if (largestStep > 30f)
            {
                Console.WriteLine("    >>> 存在明显跳变，手腕可能在特定姿态下突然翻一下");
            }
            else
            {
                Console.WriteLine("    >>> 变化连续，不会出现突然翻腕");
            }

            // The solver's forearm reference is the body's up axis. That reference is degenerate
            // exactly when the forearm points along up, i.e. straight up in front of the body.
            // Probe that direction family so the documented limitation is measured, not assumed.
            Console.WriteLine();
            Console.WriteLine("  退化方向探针：前臂朝正上方附近（前臂轴与身体上方向近平行）");

            double[] tilts = { 0.0, 5.0, 15.0, 30.0, 45.0 };
            float previousTwist = float.NaN;
            float largestDegenerateStep = 0f;

            foreach (double tilt in tilts)
            {
                double t = tilt * Math.PI / 180.0;

                // Upper arm reaching forward and slightly down; forearm rotated up towards the
                // vertical by `tilt` degrees, so the bend axis degenerates as tilt reaches 0.
                Vec3 upperDir = new Vec3(0f, -0.3f, 0.954f).Normalized();

                // Build a forearm direction that approaches +Y as tilt shrinks.
                Vec3 nearVertical = new Vec3(0f, (float)Math.Cos(t), (float)Math.Sin(t));
                Vec3 lowerDir = nearVertical.Normalized();
                if (VectorAngleDegrees(upperDir, lowerDir) < 5.0)
                {
                    // Keep a plausible bend so the pose stays physical.
                    Quat adjust = Quat.AxisAngle(Vec3.Right, 20f * (float)Math.PI / 180f);
                    lowerDir = (adjust * upperDir).Normalized();
                }

                SkeletonFrameData frame = TestSkeleton.NeutralTorso();
                TestSkeleton.PlaceArm(frame, true, upperDir, lowerDir);

                var pose = new SkeletonPose();
                if (!solver.Solve(frame, pose))
                {
                    Console.WriteLine("    倾斜 " + tilt.ToString("F0").PadLeft(4) + "°：求解失败");
                    continue;
                }

                float twist = RelativeTwist(pose, VmcBones.LeftUpperArm, VmcBones.LeftLowerArm);
                Console.WriteLine("    倾斜 " + tilt.ToString("F0").PadLeft(4) + "°：扭转 "
                    + twist.ToString("F1").PadLeft(6) + "°");

                if (!float.IsNaN(previousTwist))
                {
                    float step = Math.Abs(twist - previousTwist);
                    if (step > largestDegenerateStep) { largestDegenerateStep = step; }
                }
                previousTwist = twist;
            }

            if (largestDegenerateStep > 20f)
            {
                Console.WriteLine("    >>> 该方向族内扭转有跳变；前臂接近正上方时手腕可能翻一下");
                Console.WriteLine("        这是已知近似：Kinect 只给关节位置，前臂滚转没有唯一解");
            }
            else
            {
                Console.WriteLine("    >>> 该方向族内扭转连续，退化情形未造成可见问题");
            }

            return 0;
        }

        private static double VectorAngleDegrees(Vec3 a, Vec3 b)
        {
            float dot = Vec3.Dot(a.Normalized(), b.Normalized());
            if (dot > 1f) { dot = 1f; }
            if (dot < -1f) { dot = -1f; }
            return Math.Acos(dot) * 57.29577951308232;
        }

        /// <summary>
        /// Twist of the forearm about its own axis, relative to the upper arm.
        ///
        /// Decomposition matters here: the total angle of (upper^-1 * lower) is dominated by the
        /// elbow bend itself, so using it would just report the bend back. Swing-twist
        /// decomposition isolates the part that rotates about the forearm's own direction, which
        /// is the quantity that shows up as an unnatural wrist roll on the avatar.
        /// </summary>
        private static float RelativeTwist(SkeletonPose pose, string upperBone, string lowerBone)
        {
            Quat upper = pose.BoneRotations[upperBone].Normalized();
            Quat lower = pose.BoneRotations[lowerBone].Normalized();

            Quat relative = (upper.Conjugate() * lower).Normalized();

            // The forearm's own axis is the image of local +Y under its world rotation.
            Vec3 axis = (lower * Vec3.Up).Normalized();
            if (axis.LengthSquared < 0.5f) { return 0f; }

            float projection = Vec3.Dot(new Vec3(relative.X, relative.Y, relative.Z), axis);

            // Twist quaternion about that axis; its rotation angle is the roll we care about.
            var twist = new Quat(axis.X * projection, axis.Y * projection, axis.Z * projection, relative.W);
            return RotationAngleDegrees(twist);
        }

        /// <summary>Rotation magnitude of a quaternion, in degrees.</summary>
        private static float RotationAngleDegrees(Quat q)
        {
            Quat normalized = q.Normalized();
            float w = Math.Abs(normalized.W);
            if (w > 1f) { w = 1f; }
            return (float)(2.0 * Math.Acos(w) * 57.29577951308232);
        }

        private static int VerifyDirectionsStillExact(UpperBodySolver solver)
        {
            int failures = 0;
            Console.WriteLine();
            Console.WriteLine("  （对照）弯曲姿态下方向仍须精确：");

            double[] bends = { 30.0, 90.0 };
            foreach (double bendDegrees in bends)
            {
                SkeletonFrameData frame = TestSkeleton.NeutralTorso();
                Vec3 upperDir = new Vec3(0.5f, -0.866f, 0f).Normalized();
                Vec3 bendAxis = Vec3.Cross(upperDir, Vec3.Up).Normalized();
                Quat bend = Quat.AxisAngle(bendAxis, (float)(bendDegrees * Math.PI / 180.0));
                Vec3 lowerDir = (bend * upperDir).Normalized();
                TestSkeleton.PlaceArm(frame, true, upperDir, lowerDir);

                var pose = new SkeletonPose();
                if (!solver.Solve(frame, pose)) { failures++; continue; }

                Vec3 restUpper = TestSkeleton.RestArmDirection();
                Vec3 measuredUpper = TestSkeleton.Direction(frame, KinectJoint.ShoulderLeft, KinectJoint.ElbowLeft);
                Vec3 measuredLower = TestSkeleton.Direction(frame, KinectJoint.ElbowLeft, KinectJoint.WristLeft);

                float upperError = (StraightArmAudit.WorldRotation(pose, VmcBones.LeftUpperArm) * restUpper - measuredUpper).Length;
                float lowerError = (StraightArmAudit.WorldRotation(pose, VmcBones.LeftLowerArm) * restUpper - measuredLower).Length;

                bool ok = upperError < 3e-3f && lowerError < 3e-3f;
                if (!ok) { failures++; }
                Console.WriteLine("    " + (ok ? "[ok]  " : "[FAIL]") + " 弯曲 " + bendDegrees.ToString("F0")
                    + "°：上臂误差 " + upperError.ToString("F5")
                    + "，前臂误差 " + lowerError.ToString("F5"));
            }

            return failures;
        }
    }
}
