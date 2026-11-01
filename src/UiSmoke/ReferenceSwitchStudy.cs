using System;
using Kinect2VMC.Core;

namespace Kinect2VMC.UiSmoke
{
    /// <summary>
    /// Decision tool, not a test: measures how large the roll discontinuity would be if the
    /// solver switched its reference axis when the preferred one becomes parallel to the bone.
    ///
    /// The two candidate frames differ by a rotation of <c>restZ</c> about the bone axis. When the
    /// preferred axis is nearly parallel to the bone, that rotation approaches 90 degrees, so the
    /// switch would inject up to 90 degrees of roll in one frame -- far worse than the
    /// ill-conditioning it was meant to cure.
    /// </summary>
    internal static class ReferenceSwitchStudy
    {
        public static int Run()
        {
            Console.WriteLine();
            Console.WriteLine("Reference-axis switch study (would it be worth it?)");
            Console.WriteLine("  上臂朝向                         | 平行度 | 切换会引入的滚转跳变");

            var secondaries = new[]
            {
                new { Name = "down-out", Dir = new Vec3(0.5f, -0.866f, 0f) },
                new { Name = "sideways", Dir = new Vec3(-1f, 0f, 0f) },
                new { Name = "up-out", Dir = new Vec3(-0.5f, 0.866f, 0f) },
                new { Name = "up 45", Dir = new Vec3(0f, 0.707f, 0.707f) },
                new { Name = "near up", Dir = new Vec3(0f, 0.995f, 0.1f) },
                new { Name = "straight up", Dir = new Vec3(0f, 1f, 0f) }
            };

            foreach (var entry in secondaries)
            {
                Vec3 primary = entry.Dir.Normalized();

                // The frame the solver builds: local Y along the bone, local Z from the preferred
                // reference (the body's up axis), local X completing the triple.
                Vec3 preferred = Vec3.Up;
                float alignment = Math.Abs(Vec3.Dot(preferred, primary));

                Vec3 zPreferred = preferred - primary * Vec3.Dot(preferred, primary);
                zPreferred = zPreferred.LengthSquared < 1e-9f ? Vec3.Zero : zPreferred.Normalized();

                // The fallback frame, from a perpendicular reference.
                Vec3 fallback = Vec3.Backward;
                Vec3 zFallback = fallback - primary * Vec3.Dot(fallback, primary);
                zFallback = zFallback.LengthSquared < 1e-9f ? Vec3.Zero : zFallback.Normalized();

                string jump;
                if (zPreferred.LengthSquared < 0.5f || zFallback.LengthSquared < 0.5f)
                {
                    jump = "参考轴退化，无法计算";
                }
                else
                {
                    float dot = Vec3.Dot(zPreferred, zFallback);
                    if (dot > 1f) { dot = 1f; }
                    if (dot < -1f) { dot = -1f; }
                    jump = (Math.Acos(dot) * 57.29577951308232).ToString("F1") + "°";
                }

                Console.WriteLine("  " + entry.Name.PadRight(32) + " | "
                    + alignment.ToString("F3") + "  | " + jump);
            }

            Console.WriteLine();
            Console.WriteLine("  判读：");
            Console.WriteLine("    - 只有当两个参考轴几乎垂直时（右列为 90°），切换才便宜");
            Console.WriteLine("    - 而切换恰恰发生在两者几乎平行时（平行度接近 1，右列接近 0°）");
            Console.WriteLine("    - 若右列很大，说明切换会注入比病态本身更大的跳变，不该做");

            // Quantify the two frames for the specific degenerate pose the audit found.
            Vec3 degeneratePrimary = new Vec3(0f, 1f, 0f);
            Vec3 zUp = (Vec3.Up - degeneratePrimary * Vec3.Dot(Vec3.Up, degeneratePrimary));
            Vec3 zBack = (Vec3.Backward - degeneratePrimary * Vec3.Dot(Vec3.Backward, degeneratePrimary));
            Console.WriteLine();
            Console.WriteLine("  退化姿态（前臂竖直）下两个参考轴的正交性：");
            Console.WriteLine("    身体上方向投影长度 = " + zUp.Length.ToString("F4") + "  （0 表示完全退化）");
            Console.WriteLine("    背后方向投影长度   = " + zBack.Length.ToString("F4"));

            return 0;
        }
    }
}
