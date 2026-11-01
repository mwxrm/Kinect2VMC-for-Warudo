using System;
using System.Collections.Generic;
using Kinect2VMC.Core;

namespace Kinect2VMC.UiSmoke
{
    /// <summary>
    /// Empirically determines which camera-space sign convention satisfies every direction
    /// invariant at once.
    ///
    /// Hand-deriving the mapping has now produced three wrong answers, each of which shipped
    /// because the offline suite never exercised the adapter's convention. Rather than reason
    /// about it a fourth time, this enumerates the candidate conventions and reports which ones
    /// hold for up, forward, right and the T-pose control simultaneously.
    /// </summary>
    internal static class ConventionSearch
    {
        public static int Run()
        {
            Console.WriteLine();
            Console.WriteLine("Camera-space convention search (which sign flip is consistent?)");
            Console.WriteLine("  X,Y,Z 表示是否取负    | 上抬 | 前伸 | T字 | 结论");

            string best = null;
            var results = new List<string>();

            foreach (int mask in new[] { 0, 1, 2, 4, 3, 5, 6, 7 })
            {
                bool negX = (mask & 1) != 0;
                bool negY = (mask & 2) != 0;
                bool negZ = (mask & 4) != 0;

                bool upOk = CheckUp(negX, negY, negZ);
                bool forwardOk = CheckForward(negX, negY, negZ);
                bool tPoseOk = CheckTPose(negX, negY, negZ);

                bool allOk = upOk && forwardOk && tPoseOk;
                string label = (negX ? "-X" : "+X") + " " + (negY ? "-Y" : "+Y") + " " + (negZ ? "-Z" : "+Z");
                string verdict = allOk ? "<<< 唯一自洽" : "";

                Console.WriteLine("  " + label.PadRight(20) + " | "
                    + (upOk ? " ok " : " -- ") + " | "
                    + (forwardOk ? " ok " : " -- ") + " | "
                    + (tPoseOk ? " ok " : " -- ") + " | " + verdict);

                if (allOk) { best = label; }
                results.Add(label + (allOk ? " PASS" : ""));
            }

            Console.WriteLine();
            if (best == null)
            {
                Console.WriteLine("  >>> 没有任何单一符号约定能同时满足三项，说明不变量本身定义有误");
            }
            else
            {
                Console.WriteLine("  >>> 应当采用：" + best);
            }

            // A single global flip of all three axes cannot change any *direction* comparison
            // result, so exactly one of each complementary pair can win. Report the pair to make
            // that explicit.
            Console.WriteLine();
            Console.WriteLine("  说明：把三个轴同时取负不会改变任何方向的相对关系，");
            Console.WriteLine("        因此 -X-Y-Z 与 +X+Y+Z 等价，必须靠左右手约定来打破平局：");
            Console.WriteLine("        角色自己的 +X 指向它的右侧，与人正对镜头时观察者的右侧一致，");
            Console.WriteLine("        故 X 不取负。");

            return 0;
        }

        private static Vec3 Map(Vec3 d, bool negX, bool negY, bool negZ)
        {
            return new Vec3(negX ? -d.X : d.X, negY ? -d.Y : d.Y, negZ ? -d.Z : d.Z);
        }

        /// <summary>The user's hand moves from shoulder height to well above it.</summary>
        private static bool CheckUp(bool negX, bool negY, bool negZ)
        {
            Vec3 shoulder = Map(new Vec3(0.2f, 1.1f, 1.4f), negX, negY, negZ);
            Vec3 hand = Map(new Vec3(0.4f, 1.9f, 1.4f), negX, negY, negZ);
            Vec3 d = (hand - shoulder).Normalized();
            return d.Y > 0.9f;   // character up is +Y
        }

        /// <summary>The user's hand moves towards the sensor (smaller camera Z).</summary>
        private static bool CheckForward(bool negX, bool negY, bool negZ)
        {
            Vec3 shoulder = Map(new Vec3(0.2f, 1.1f, 1.4f), negX, negY, negZ);
            Vec3 hand = Map(new Vec3(0.2f, 1.1f, 0.9f), negX, negY, negZ);
            Vec3 d = (hand - shoulder).Normalized();
            return d.Z > 0.9f;   // character forward is +Z (towards the camera)
        }

        /// <summary>
        /// The control that every wrong version so far also passed: a sideways arm must stay
        /// sideways, with no vertical or depth component.
        /// </summary>
        private static bool CheckTPose(bool negX, bool negY, bool negZ)
        {
            Vec3 shoulder = Map(new Vec3(0.2f, 1.1f, 1.4f), negX, negY, negZ);
            Vec3 hand = Map(new Vec3(0.85f, 1.1f, 1.4f), negX, negY, negZ);
            Vec3 d = (hand - shoulder).Normalized();
            return Math.Abs(d.X) > 0.9f && Math.Abs(d.Y) < 0.2f && Math.Abs(d.Z) < 0.2f;
        }
    }
}
