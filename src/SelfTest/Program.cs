using System;
using System.Collections.Generic;
using System.Text;
using Kinect2VMC.Core;

namespace Kinect2VMC.SelfTest
{
    /// <summary>
    /// Offline verification of the solver and the VMC encoder.
    ///
    /// Nothing here needs a Kinect: the joint frames are synthesised. That is deliberate,
    /// because the sensor can only be opened from an unrestricted process, while the maths
    /// is exactly the part that has to be trusted before it ever reaches the avatar.
    /// </summary>
    internal static class Program
    {
        private static int _passed;
        private static int _failed;
        private static readonly List<string> Failures = new List<string>();

        private static int Main(string[] args)
        {
            Console.OutputEncoding = Encoding.UTF8;
            _dumpPackets = args != null && Array.IndexOf(args, "--dump") >= 0;

            Console.WriteLine("Kinect2VMC offline self-test");
            Console.WriteLine("============================");

            TestVectorBasics();
            TestQuaternionBasics();
            TestRotationFrameConvention();

            TestRestPoseIsIdentity();
            TestBoneDirectionInvariant();
            TestBodyYawIsolation();
            TestTurningRootYaw();
            TestArmRaise();
            TestArmMirrorSymmetry();
            TestCalibration();
            TestSeatedScenarios();
            TestEmissionIsDeterministicAndUnique();

            TestMissingJointsFallBack();
            TestUntrackedAndDegenerateInput();
            TestHeadAndHipsOwnership();
            TestSmoothing();

            TestOscLayout();
            TestOscRoundTrip();
            TestVmcMessageShape();
            TestUdpDelivery();

            Console.WriteLine();
            Console.WriteLine("passed: " + _passed + "   failed: " + _failed);
            if (_failed > 0)
            {
                Console.WriteLine();
                Console.WriteLine("FAILURES:");
                foreach (string failure in Failures) { Console.WriteLine("  - " + failure); }
                return 1;
            }

            Console.WriteLine("ALL TESTS PASSED");
            return 0;
        }

        // ---------------------------------------------------------------- assertions

        private static void Check(bool condition, string name, string detail = null)
        {
            if (condition)
            {
                _passed++;
                Console.WriteLine("  [ok]   " + name);
            }
            else
            {
                _failed++;
                string message = name + (detail == null ? string.Empty : "  -> " + detail);
                Failures.Add(message);
                Console.WriteLine("  [FAIL] " + message);
            }
        }

        private static void CheckClose(float actual, float expected, float tolerance, string name)
        {
            Check(Math.Abs(actual - expected) <= tolerance, name,
                "expected " + expected.ToString("F5") + ", got " + actual.ToString("F5"));
        }

        private static void CheckVecClose(Vec3 actual, Vec3 expected, float tolerance, string name)
        {
            Check((actual - expected).Length <= tolerance, name, "expected " + expected + ", got " + actual);
        }

        private static void CheckQuatClose(Quat actual, Quat expected, float tolerance, string name)
        {
            // q and -q describe the same rotation.
            float alignment = Math.Abs(Quat.Dot(actual.Normalized(), expected.Normalized()));
            Check(1f - alignment <= tolerance, name, "expected " + expected + ", got " + actual);
        }

        /// <summary>Rotation magnitude of a unit quaternion, in degrees.</summary>
        private static float RotationAngleDegrees(Quat q)
        {
            float w = Math.Abs(q.Normalized().W);
            if (w > 1f) { w = 1f; }
            return (float)(2.0 * Math.Acos(w) * 57.29577951308232);
        }

        private static Vec3 RotationAxis(Quat q)
        {
            return new Vec3(q.X, q.Y, q.Z).Normalized();
        }

        // ---------------------------------------------------------------- maths layer

        private static void TestVectorBasics()
        {
            Console.WriteLine();
            Console.WriteLine("Vec3");
            CheckVecClose(Vec3.Cross(Vec3.Up, Vec3.Right), Vec3.Backward, 1e-5f, "up x right = backward");
            CheckVecClose(Vec3.Cross(Vec3.Right, Vec3.Up), Vec3.Forward, 1e-5f, "right x up = forward");
            CheckClose(Vec3.Dot(Vec3.Right, Vec3.Up), 0f, 1e-6f, "right . up = 0");
            CheckVecClose(new Vec3(3f, 4f, 0f).Normalized(), new Vec3(0.6f, 0.8f, 0f), 1e-5f, "normalize 3-4-5");
            CheckVecClose(new Vec3(10f, 0f, 0f).ClampedMagnitude(2f), new Vec3(2f, 0f, 0f), 1e-5f, "clamp magnitude");
            Check(!new Vec3(float.NaN, 0f, 0f).IsFinite(), "NaN is not finite");
        }

        private static void TestQuaternionBasics()
        {
            Console.WriteLine();
            Console.WriteLine("Quat");

            Quat yaw90 = Quat.AxisAngle(Vec3.Up, (float)(Math.PI / 2.0));
            CheckVecClose(yaw90 * Vec3.Forward, Vec3.Right, 1e-4f, "yaw +90 about up maps forward -> right");
            CheckVecClose(yaw90 * Vec3.Right, Vec3.Backward, 1e-4f, "yaw +90 maps right -> backward");

            Quat pitch90 = Quat.AxisAngle(Vec3.Right, (float)(Math.PI / 2.0));
            CheckVecClose(pitch90 * Vec3.Forward, Vec3.Down, 1e-4f, "pitch +90 about right maps forward -> down");

            Quat q = Quat.AxisAngle(new Vec3(1f, 2f, 3f), 0.7f);
            CheckVecClose(q.Conjugate() * (q * Vec3.Forward), Vec3.Forward, 1e-4f, "q^-1 * q * v = v");

            Quat a = Quat.AxisAngle(Vec3.Up, (float)(Math.PI / 2.0));
            CheckVecClose((a * a) * Vec3.Forward, Vec3.Backward, 1e-4f, "yaw 180 flips forward");

            CheckClose(Quat.AxisAngle(Vec3.Up, 0.3f).ToEulerDegrees().Y, 0.3f * 57.2957795f, 0.05f,
                "euler yaw round trip");

            // Slerp midpoint of two 90 degree yaws is a 45 degree yaw.
            Quat half = Quat.Slerp(Quat.Identity, yaw90, 0.5f);
            CheckClose(RotationAngleDegrees(half), 45f, 1e-3f, "slerp midpoint halves the angle");
            CheckVecClose(half * Vec3.Forward, new Vec3(0.7071f, 0f, 0.7071f), 1e-3f, "slerp midpoint direction");

            // Slerp must take the short way around when the quaternions are flipped.
            Quat negated = new Quat(-yaw90.X, -yaw90.Y, -yaw90.Z, -yaw90.W);
            CheckClose(RotationAngleDegrees(Quat.Slerp(Quat.Identity, negated, 0.5f)), 45f, 1e-3f,
                "slerp takes the short path for negated quaternions");
        }

        private static void TestRotationFrameConvention()
        {
            Console.WriteLine();
            Console.WriteLine("Rotation frame convention");

            // The solver builds every bone rotation from an orthonormal frame. If the cross
            // product convention and the trace formula disagree, every bone comes out
            // inverted, so pin the round trip down first.
            Vec3 x = Vec3.Right;
            Vec3 y = Vec3.Up;
            Vec3 z = Vec3.Cross(x, y);
            Quat q = UpperBodySolver.QuatFromAxes(x, y, z);
            CheckVecClose(q * Vec3.Right, x, 1e-4f, "QuatFromAxes reproduces local X");
            CheckVecClose(q * Vec3.Up, y, 1e-4f, "QuatFromAxes reproduces local Y");
            CheckVecClose(q * Vec3.Forward, z, 1e-4f, "QuatFromAxes reproduces local Z");

            for (int i = 0; i < 8; i++)
            {
                Quat rotation = Quat.AxisAngle(new Vec3(0.3f + i, 1f, -0.2f * i), 0.2f * i + 0.1f);
                Vec3 rx = rotation * Vec3.Right;
                Vec3 ry = rotation * Vec3.Up;
                Vec3 rz = rotation * Vec3.Forward;
                Quat recovered = UpperBodySolver.QuatFromAxes(rx, ry, rz);
                CheckQuatClose(recovered, rotation, 1e-3f, "QuatFromAxes recovers rotation #" + i);
            }
        }

        // ---------------------------------------------------------------- solver

        private static void TestRestPoseIsIdentity()
        {
            Console.WriteLine();
            Console.WriteLine("Rest pose");

            var solver = new UpperBodySolver { Smoothing = 0f, SendHead = true };
            var pose = new SkeletonPose();

            Check(solver.Solve(TestSkeleton.RestPose(), pose), "rest pose solves");

            if (_dumpPackets)
            {
                SkeletonFrameData probe = TestSkeleton.RestPose();
                Console.WriteLine("  --- solver diagnostics (rest pose) ---");
                Console.WriteLine("  ShoulderLeft  = " + probe.Get(KinectJoint.ShoulderLeft));
                Console.WriteLine("  ElbowLeft     = " + probe.Get(KinectJoint.ElbowLeft));
                Console.WriteLine("  WristLeft     = " + probe.Get(KinectJoint.WristLeft));
                Console.WriteLine("  SpineBase     = " + probe.Get(KinectJoint.SpineBase));
                Console.WriteLine("  SpineShoulder = " + probe.Get(KinectJoint.SpineShoulder));
                Console.WriteLine("  expected arm direction = " + TestSkeleton.RestArmDirection());
                foreach (var pair in solver.DiagnosticPrimary)
                {
                    Console.WriteLine("  " + pair.Key.PadRight(16)
                        + " primary=" + pair.Value
                        + " rest=" + solver.DiagnosticRestPrimary[pair.Key]
                       
                        + " result=" + solver.DiagnosticResult[pair.Key]);
                }
            }
            if (_dumpPackets)
            {
                Console.WriteLine("  up=" + solver.DiagnosticBodyUp
                    + " right=" + solver.DiagnosticBodyRight
                    + " forward=" + solver.DiagnosticBodyForward
                    + " frame=" + solver.DiagnosticBodyFrame);
                foreach (var pair in solver.DiagnosticPrimary)
                {
                    Console.WriteLine("  " + pair.Key.PadRight(16)
                        + " primary=" + pair.Value
                        + " rest=" + solver.DiagnosticRestPrimary[pair.Key]
                        + " result=" + solver.DiagnosticResult[pair.Key]);
                }
            }

            // 11 with SendHand off (the default): 9 upper-body bones plus Head and Neck.
            Check(pose.BoneRotations.Count == 13, "rest pose produces 13 bones",
                "got " + pose.BoneRotations.Count);
            Check(!solver.IsCalibrated, "solver starts uncalibrated");

            // The synthesised rest pose is by definition the solver's neutral pose, so every
            // bone must come out as the identity rotation.
            foreach (var pair in pose.BoneRotations)
            {
                CheckQuatClose(pair.Value, Quat.Identity, 2e-3f, "rest pose identity: " + pair.Key);
            }
        }

        private static void TestBoneDirectionInvariant()
        {
            Console.WriteLine();
            Console.WriteLine("Bone direction invariant");

            var solver = new UpperBodySolver { Smoothing = 0f };
            var pose = new SkeletonPose();

            // An asymmetric pose: left arm forward and up, right arm out to the side.
            SkeletonFrameData frame = TestSkeleton.NeutralTorso();
            TestSkeleton.PlaceArm(frame, true,
                new Vec3(0.5f, 0.5f, 0.7071f), new Vec3(0.2f, 0.9f, 0.3873f));
            TestSkeleton.PlaceArm(frame, false,
                new Vec3(-1f, 0f, 0f), new Vec3(-0.9f, 0.4359f, 0f));

            Check(solver.Solve(frame, pose), "posed skeleton solves");

            // Applying the produced world rotation to the rest-pose bone direction must
            // reproduce the measured direction. This is the core correctness property.
            CheckBoneDirection(pose, frame, VmcBones.LeftUpperArm,
                TestSkeleton.RestArmDirection(), KinectJoint.ShoulderLeft, KinectJoint.ElbowLeft);
            CheckBoneDirection(pose, frame, VmcBones.LeftLowerArm,
                TestSkeleton.RestArmDirection(), KinectJoint.ElbowLeft, KinectJoint.WristLeft);
            CheckBoneDirection(pose, frame, VmcBones.RightUpperArm,
                new Vec3(-TestSkeleton.RestArmDirection().X,
                    TestSkeleton.RestArmDirection().Y, TestSkeleton.RestArmDirection().Z),
                KinectJoint.ShoulderRight, KinectJoint.ElbowRight);
            CheckBoneDirection(pose, frame, VmcBones.RightLowerArm,
                new Vec3(-TestSkeleton.RestArmDirection().X,
                    TestSkeleton.RestArmDirection().Y, TestSkeleton.RestArmDirection().Z),
                KinectJoint.ElbowRight, KinectJoint.WristRight);
            CheckBoneDirection(pose, frame, VmcBones.Spine,
                Vec3.Up, KinectJoint.SpineBase, KinectJoint.SpineMid);
            CheckBoneDirection(pose, frame, VmcBones.Chest,
                Vec3.Up, KinectJoint.SpineMid, KinectJoint.SpineShoulder);
            CheckBoneDirection(pose, frame, VmcBones.UpperChest,
                Vec3.Up, KinectJoint.SpineShoulder, KinectJoint.Neck);
        }

        /// <summary>
        /// The VMC bone rotation for <paramref name="bone"/>, composed with its ancestors.
        ///
        /// The solver emits LOCAL rotations (each bone relative to its parent), which is what
        /// /VMC/Ext/Bone/Pos carries. A bone's WORLD rotation is its local rotation multiplied by
        /// its parent's world rotation, so the assertions below have to compose the chain before
        /// they can compare against a measured direction.
        /// </summary>
        private static Quat WorldRotation(SkeletonPose pose, string bone)
        {
            Quat local;
            if (!pose.BoneRotations.TryGetValue(bone, out local)) { return Quat.Identity; }

            string parent = SolverParents[bone];
            if (parent == null) { return local; }
            if (!pose.BoneRotations.ContainsKey(parent)) { return local; }

            return WorldRotation(pose, parent) * local;
        }

        /// <summary>The parent chain the solver uses, mirroring UpperBodySolver's definitions.</summary>
        private static readonly Dictionary<string, string> SolverParents =
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

        private static void CheckBoneDirection(SkeletonPose pose, SkeletonFrameData frame, string bone,
            Vec3 restDirection, KinectJoint from, KinectJoint to)
        {
            if (!pose.BoneRotations.ContainsKey(bone))
            {
                Check(false, "bone present: " + bone, "missing from pose");
                return;
            }

            Vec3 expected = TestSkeleton.Direction(frame, from, to);
            Vec3 actual = WorldRotation(pose, bone) * restDirection.Normalized();
            CheckVecClose(actual, expected, 3e-3f, "direction ok: " + bone);
        }

        private static void TestBodyYawIsolation()
        {
            Console.WriteLine();
            Console.WriteLine("Rigid-motion behaviour");

            var solver = new UpperBodySolver { Smoothing = 0f };
            var pose = new SkeletonPose();

            // A rigid TRANSLATION of the whole body must produce no bone rotation at all:
            // every bone direction is unchanged and only the root position moves.
            SkeletonFrameData moved = TestSkeleton.Transform(
                TestSkeleton.RestPose(), Quat.Identity, new Vec3(0.15f, -0.05f, 0.2f));

            Check(solver.Solve(moved, pose), "translated body solves");
            foreach (var pair in pose.BoneRotations)
            {
                CheckQuatClose(pair.Value, Quat.Identity, 2e-3f, "translation is bone-neutral: " + pair.Key);
            }
            CheckVecClose(pose.RootPosition, moved.Get(KinectJoint.SpineBase), 1e-5f,
                "root position follows the translation");

            // A rigid YAW of the whole body is deliberately NOT bone-neutral in this solver.
            // Each bone is measured against its neutral direction in character space, so an arm
            // genuinely changes its world direction when its owner turns on the spot. That
            // behaviour is pinned down here rather than asserted away, so a future change of
            // convention is caught instead of silently shipped.
            //
            // With the arms hanging (the avatar bind pose) the arm is nearly parallel to the turn
            // axis, so a yaw moves it far less than the yaw angle itself. The assertion therefore
            // checks the swing that the arm's own direction ACTUALLY undergoes, not the yaw angle:
            // hard-coding the yaw angle here would only re-encode the old T-pose assumption.
            float radians = 35f * 0.0174532925f;
            SkeletonFrameData turned = TestSkeleton.Transform(
                TestSkeleton.RestPose(), Quat.AxisAngle(Vec3.Up, radians), Vec3.Zero);

            var turnedPose = new SkeletonPose();
            Check(solver.Solve(turned, turnedPose), "turned body solves");

            // The spine lies along the turn axis, so it cannot move.
            CheckQuatClose(turnedPose.BoneRotations[VmcBones.Spine], Quat.Identity, 3e-3f,
                "a pure yaw leaves the spine unrotated");

            // The arm must report the swing its own direction actually undergoes.
            Quat arm = WorldRotation(turnedPose, VmcBones.LeftUpperArm);
            Vec3 restArm = TestSkeleton.RestArmDirection().Normalized();
            Vec3 turnedArm = TestSkeleton.Direction(turned, KinectJoint.ShoulderLeft, KinectJoint.ElbowLeft);

            float expectedSwing = 0f;
            {
                float dot = Vec3.Dot(restArm, turnedArm.Normalized());
                if (dot > 1f) { dot = 1f; }
                if (dot < -1f) { dot = -1f; }
                expectedSwing = (float)(Math.Acos(dot) * 57.29577951308232);
            }

            Check(expectedSwing > 1f,
                "the yaw actually moves the arm",
                expectedSwing.ToString("F2") + " degrees");

            CheckClose(RotationAngleDegrees(arm), expectedSwing, 1.5f,
                "a pure yaw swings the arm by the angle its direction really changes");

            // The swing axis is not necessarily vertical; it is
            // perpendicular to both the rest and the turned direction. Assert that instead of the
            // old "about the vertical axis", which only held for a horizontal arm.
            Vec3 expectedAxis = Vec3.Cross(restArm, turnedArm.Normalized());
            if (expectedAxis.LengthSquared > 1e-8f)
            {
                expectedAxis = expectedAxis.Normalized();
                Check(Math.Abs(Vec3.Dot(RotationAxis(arm), expectedAxis)) > 0.999f,
                    "the arm swing is about the axis perpendicular to both directions",
                    "axis = " + RotationAxis(arm) + " expected " + expectedAxis);
            }

            // And the result must still land the bone on its measured direction.
            CheckVecClose(arm * TestSkeleton.RestArmDirection(),
                TestSkeleton.Direction(turned, KinectJoint.ShoulderLeft, KinectJoint.ElbowLeft),
                3e-3f, "the swung arm still matches the measured direction");
        }

        /// <summary>
        /// 转身（根偏航）的行为。这里钉死两个实测踩过的坑：
        ///
        /// ① 基准必须"用户坐定后自动建立"。曾经把自动建立删掉、只留手点校准，结果
        ///    不点校准就完全不转身，而且没有任何报错 —— 现象和"通路断了"一模一样。
        /// ② 基准不能取自单帧采样。肩线基线只有 40 cm 上下，几厘米的关节噪声就会
        ///    固化成几度的永久朝向偏移（用户看到的是"正对镜头，模型却朝一侧偏、腰扭"）。
        /// </summary>
        private static void TestTurningRootYaw()
        {
            Console.WriteLine();
            Console.WriteLine("Turning (root yaw)");

            // ① 坐定约 1 秒后自动建立基准，不需要手点校准。
            {
                var solver = new UpperBodySolver { SendRootRotation = true };
                float establishedAt = -1f;

                for (int i = 0; i < 90; i++)
                {
                    SkeletonFrameData frame = TestSkeleton.RestPose();
                    frame.Time = i / 30f;

                    var pose = new SkeletonPose();
                    if (!solver.Solve(frame, pose)) { continue; }
                    if (pose.HasRootRotation) { establishedAt = frame.Time; break; }
                }

                Check(establishedAt > 0f, "坐定后自动建立转身基准（不需要手点校准）",
                    establishedAt < 0f
                        ? "90 帧内始终没有建立"
                        : ("用时 " + establishedAt.ToString("F2") + " 秒"));
                CheckClose(establishedAt, 1f, 0.2f, "基准在坐定约 1 秒后建立，而不是第一帧");
            }

            // ② 转身要产生正确的根偏航：角度对、绕竖直轴、方向与转身一致。
            {
                var solver = new UpperBodySolver { SendRootRotation = true };
                SkeletonPose settled = null;

                for (int i = 0; i <= 40; i++)
                {
                    SkeletonFrameData frame = TestSkeleton.RestPose();
                    frame.Time = i / 30f;
                    settled = new SkeletonPose();
                    solver.Solve(frame, settled);
                }

                Check(settled.HasRootRotation, "基准建立后开始发送根偏航");

                float radians = 30f * 0.0174532925f;
                SkeletonFrameData turned = TestSkeleton.Transform(
                    TestSkeleton.RestPose(), Quat.AxisAngle(Vec3.Up, radians), Vec3.Zero);

                // 肩线是平滑过的，转身之后要几帧才走完，所以连发 20 帧再看结果。
                bool solvedEveryFrame = true;
                var turnedPose = new SkeletonPose();
                for (int i = 0; i < 20; i++)
                {
                    turned.Time = 41f / 30f + i / 30f;
                    turnedPose = new SkeletonPose();
                    if (!solver.Solve(turned, turnedPose)) { solvedEveryFrame = false; }
                }

                Check(solvedEveryFrame, "转身后每帧仍能解算");
                Check(turnedPose.HasRootRotation, "转身后发送根偏航");

                CheckClose(RotationAngleDegrees(turnedPose.RootRotation), 30f, 1.5f,
                    "根偏航的角度等于实际转身角");

                Vec3 axis = RotationAxis(turnedPose.RootRotation);
                Check(axis.Y > 0.999f, "根偏航绕竖直轴，方向与转身一致",
                    "axis = " + axis);
            }

            // ③ 肩线噪声不得变成永久的朝向偏移。
            {
                var solver = new UpperBodySolver { SendRootRotation = true };
                var pose = new SkeletonPose();
                float worst = 0f;

                for (int i = 0; i < 60; i++)
                {
                    SkeletonFrameData frame = TestSkeleton.RestPose();
                    frame.Time = i / 30f;

                    // 4 cm 的肩线抖动（Kinect 关节噪声的量级），隔帧反向 —— 这是指数平滑
                    // 最吃亏的输入，也是"单帧采样当基准"最容易暴露的输入。
                    float jitter = (i % 2 == 0) ? 0.04f : -0.04f;
                    Vec3 shoulder = frame.Get(KinectJoint.ShoulderLeft);
                    frame.Set(KinectJoint.ShoulderLeft,
                        new Vec3(shoulder.X, shoulder.Y, shoulder.Z + jitter), true);

                    pose = new SkeletonPose();
                    solver.Solve(frame, pose);
                    if (pose.HasRootRotation)
                    {
                        worst = Math.Max(worst, RotationAngleDegrees(pose.RootRotation));
                    }
                }

                Check(pose.HasRootRotation, "肩线抖动时依然能建立基准并发送根偏航");
                Check(worst < 2.5f, "肩线噪声不会固化成永久朝向偏移",
                    "最大偏航 " + worst.ToString("F2") + " 度");
            }
        }

        private static void TestArmRaise()
        {
            Console.WriteLine();
            Console.WriteLine("Arm raise");

            var solver = new UpperBodySolver { Smoothing = 0f };
            var pose = new SkeletonPose();

            // Rotate the left arm 40 degrees about the body's backward axis, away from the
            // built-in A-pose rest direction. The right arm is cloned from the rest pose so
            // it stays neutral.
            Quat expected = Quat.AxisAngle(Vec3.Backward, 40f * 0.0174532925f);
            Vec3 raised = (expected * TestSkeleton.RestArmDirection()).Normalized();

            SkeletonFrameData frame = TestSkeleton.RestPose();
            TestSkeleton.PlaceArm(frame, true, raised, raised);

            Check(solver.Solve(frame, pose), "raised arm solves");

            Quat upperArm = pose.BoneRotations[VmcBones.LeftUpperArm];
            CheckClose(RotationAngleDegrees(upperArm), 40f, 0.5f, "upper arm rotation magnitude is 40 degrees");
            Check(Math.Abs(Vec3.Dot(RotationAxis(upperArm), Vec3.Backward)) > 0.999f,
                "upper arm rolls about the backward axis", "axis = " + RotationAxis(upperArm));
            CheckVecClose(upperArm * TestSkeleton.RestArmDirection(), raised, 3e-3f,
                "upper arm lands on the raised direction");

            // The forearm follows in this straight-arm pose. Because VMC carries LOCAL rotations,
            // a forearm that continues along the upper arm contributes no rotation of its own: it
            // simply inherits the parent's. This is the property that keeps a straight arm
            // straight, and it is the fix for the reported "forearm folds 90 degrees" defect.
            Quat lowerArm = pose.BoneRotations[VmcBones.LeftLowerArm];
            CheckQuatClose(lowerArm, Quat.Identity, 3e-3f,
                "a straight forearm adds no local rotation of its own");
            CheckClose(RotationAngleDegrees(WorldRotation(pose, VmcBones.LeftLowerArm)), 40f, 0.5f,
                "the forearm's WORLD rotation still matches the upper arm's 40 degrees");

            // The right arm stayed put, so it must remain at identity.
            CheckQuatClose(pose.BoneRotations[VmcBones.RightUpperArm], Quat.Identity, 3e-3f,
                "untouched right arm stays at rest");

            // Sanity: the raised pose really is above the rest pose.
            Check(raised.Y > TestSkeleton.RestArmDirection().Y, "raised direction is higher than rest");
        }

        private static void TestArmMirrorSymmetry()
        {
            Console.WriteLine();
            Console.WriteLine("Arm mirror symmetry");

            var solver = new UpperBodySolver { Smoothing = 0f };
            var pose = new SkeletonPose();

            // Raise both arms by 40 degrees, symmetrically. PlaceArms mirrors the left
            // direction onto X, so this is the natural symmetric pose.
            Quat expected = Quat.AxisAngle(Vec3.Backward, 40f * 0.0174532925f);
            Vec3 raised = (expected * TestSkeleton.RestArmDirection()).Normalized();

            SkeletonFrameData frame = TestSkeleton.NeutralTorso();
            TestSkeleton.PlaceArms(frame, raised);

            Check(solver.Solve(frame, pose), "mirrored arms solve");

            Quat left = pose.BoneRotations[VmcBones.LeftUpperArm];
            Quat right = pose.BoneRotations[VmcBones.RightUpperArm];

            CheckClose(RotationAngleDegrees(left), 40f, 0.5f, "left arm rotation magnitude is 40 degrees");
            CheckClose(RotationAngleDegrees(right), 40f, 0.5f, "right arm rotation magnitude is 40 degrees");

            Check(Math.Abs(Vec3.Dot(RotationAxis(left), Vec3.Backward)) > 0.999f,
                "left arm rolls about backward", "axis = " + RotationAxis(left));
            Check(Math.Abs(Vec3.Dot(RotationAxis(right), Vec3.Backward)) > 0.999f,
                "right arm rolls about backward", "axis = " + RotationAxis(right));

            // Opposite roll signs are what removes the 180 degree twist a shared reference
            // axis would otherwise introduce on one side.
            float leftSigned = left.W * Vec3.Dot(RotationAxis(left), Vec3.Backward);
            float rightSigned = right.W * Vec3.Dot(RotationAxis(right), Vec3.Backward);
            Check(Math.Sign(leftSigned) != Math.Sign(rightSigned), "left and right rolls have opposite sign",
                "left=" + leftSigned.ToString("F4") + " right=" + rightSigned.ToString("F4"));

            // A symmetric raise must not introduce yaw on either arm.
            Check(Math.Abs(left.ToEulerDegrees().Y) < 1f, "left arm has no spurious yaw",
                "euler = " + left.ToEulerDegrees());
            Check(Math.Abs(right.ToEulerDegrees().Y) < 1f, "right arm has no spurious yaw",
                "euler = " + right.ToEulerDegrees());

            // Both arms must reach their mirrored targets. The rotations are world-space, so
            // the right arm's resulting direction is the X-mirror of the left arm's.
            CheckVecClose(left * TestSkeleton.RestArmDirection(), raised, 3e-3f, "left arm lands on target");
            CheckVecClose(right * new Vec3(-TestSkeleton.RestArmDirection().X,
                TestSkeleton.RestArmDirection().Y, TestSkeleton.RestArmDirection().Z),
                new Vec3(-raised.X, raised.Y, raised.Z), 3e-3f,
                "right arm lands on mirrored target");
        }

        private static void TestCalibration()
        {
            Console.WriteLine();
            Console.WriteLine("Calibration");

            var solver = new UpperBodySolver { Smoothing = 0f };

            // A user who sits with the arms straight out in a T-pose, which is NOT the
            // built-in default rest direction.
            SkeletonFrameData tPose = TestSkeleton.NeutralTorso();
            TestSkeleton.PlaceArms(tPose, new Vec3(1f, 0f, 0f));

            var beforePose = new SkeletonPose();
            solver.Solve(tPose, beforePose);
            float beforeAngle = RotationAngleDegrees(beforePose.BoneRotations[VmcBones.LeftUpperArm]);
            Check(beforeAngle > 30f, "uncalibrated T-pose reports a large arm rotation",
                "got " + beforeAngle.ToString("F1") + " degrees");

            Check(solver.Calibrate(tPose), "calibration succeeds");
            Check(solver.IsCalibrated, "solver reports calibrated");

            // Solving the very same pose again must now be the neutral pose.
            var afterPose = new SkeletonPose();
            solver.Solve(tPose, afterPose);
            foreach (var pair in afterPose.BoneRotations)
            {
                CheckQuatClose(pair.Value, Quat.Identity, 3e-3f, "calibrated neutral is identity: " + pair.Key);
            }

            // And motion away from the calibrated pose is still measured correctly.
            Quat raise = Quat.AxisAngle(Vec3.Backward, 25f * 0.0174532925f);
            Vec3 raised = (raise * new Vec3(1f, 0f, 0f)).Normalized();
            SkeletonFrameData moved = TestSkeleton.NeutralTorso();
            TestSkeleton.PlaceArms(moved, raised);

            var movedPose = new SkeletonPose();
            solver.Solve(moved, movedPose);
            CheckClose(RotationAngleDegrees(movedPose.BoneRotations[VmcBones.LeftUpperArm]), 25f, 0.5f,
                "calibrated motion is measured from the calibrated pose");
            CheckVecClose(movedPose.BoneRotations[VmcBones.LeftUpperArm] * new Vec3(1f, 0f, 0f),
                raised, 3e-3f, "calibrated motion lands on the measured direction");

            // Clearing calibration restores the built-in defaults.
            solver.ClearCalibration();
            Check(!solver.IsCalibrated, "clear calibration resets the flag");

            // Calibration must refuse a frame with no usable torso.
            var empty = new SkeletonFrameData();
            empty.Clear();
            Check(!solver.Calibrate(empty), "calibration refuses an untracked frame");
        }

        // ---------------------------------------------------------------- robustness

        /// <summary>
        /// Walks through the poses a seated streamer actually produces, each time calibrating
        /// from a realistic slouched rest pose first, and checks that the transmitted bone
        /// directions land where the joints say they should.
        ///
        /// This is the closest offline approximation of the real end-to-end behaviour: it is
        /// the same sequence the application performs (calibrate once, then solve every frame).
        /// </summary>
        private static void TestSeatedScenarios()
        {
            Console.WriteLine();
            Console.WriteLine("Seated scenarios (calibrate, then move)");

            var solver = new UpperBodySolver { Smoothing = 0f };

            // A realistic seated neutral pose: the arms rest lowered rather than horizontal,
            // leaning on the desk a little. This is NOT the solver's built-in default, which is
            // exactly why calibration exists.
            Vec3 relaxedArm = new Vec3(0.5f, -0.866f, 0f);   // 60 degrees below horizontal
            SkeletonFrameData neutral = TestSkeleton.NeutralTorso();
            TestSkeleton.PlaceArms(neutral, relaxedArm);

            Check(solver.Calibrate(neutral), "seated neutral pose calibrates");
            Check(solver.IsCalibrated, "solver reports calibrated");

            var heldPose = new SkeletonPose();
            Check(solver.Solve(neutral, heldPose), "calibrated pose solves");
            foreach (var pair in heldPose.BoneRotations)
            {
                CheckQuatClose(pair.Value, Quat.Identity, 3e-3f,
                    "calibrated seated pose is the identity: " + pair.Key);
            }

            // Scenario 1: raise one hand to wave. The arm must reach the measured direction and
            // the other arm must stay put.
            var wave = TestSkeleton.NeutralTorso();
            TestSkeleton.PlaceArm(wave, true, new Vec3(0.6f, 0.6f, 0.529f), new Vec3(0.2f, 0.95f, 0.24f));
            TestSkeleton.PlaceArm(wave, false, new Vec3(-relaxedArm.X, relaxedArm.Y, relaxedArm.Z),
                new Vec3(-relaxedArm.X, relaxedArm.Y, relaxedArm.Z));

            var wavePose = new SkeletonPose();
            Check(solver.Solve(wave, wavePose), "wave pose solves");
            CheckVecClose(wavePose.BoneRotations[VmcBones.LeftUpperArm] * relaxedArm,
                TestSkeleton.Direction(wave, KinectJoint.ShoulderLeft, KinectJoint.ElbowLeft),
                5e-3f, "waving arm reaches the measured direction");
            CheckQuatClose(wavePose.BoneRotations[VmcBones.RightUpperArm], Quat.Identity, 3e-3f,
                "the other arm stays at the calibrated neutral");

            // Scenario 2: lean forward to the desk. The spine chain must bend, and it must bend
            // in the direction of the lean rather than sideways.
            SkeletonFrameData lean = TestSkeleton.NeutralTorso();
            TestSkeleton.PlaceArms(lean, relaxedArm);
            Quat leanRotation = Quat.AxisAngle(Vec3.Right, 12f * 0.0174532925f);
            // Rotate the upper torso about the hip so the spine genuinely tilts forward.
            Vec3 pivot = lean.Get(KinectJoint.SpineBase);
            RotateChainAbove(lean, KinectJoint.SpineBase, pivot, leanRotation);

            var leanPose = new SkeletonPose();
            Check(solver.Solve(lean, leanPose), "lean pose solves");

            Quat spine = leanPose.BoneRotations[VmcBones.Spine];
            Check(RotationAngleDegrees(spine) > 1f, "leaning forward bends the spine",
                "spine rotation = " + RotationAngleDegrees(spine).ToString("F2") + " degrees");
            Check(Math.Abs(Vec3.Dot(RotationAxis(spine), Vec3.Right)) > 0.98f,
                "the spine bends about the sideways axis, not a diagonal",
                "axis = " + RotationAxis(spine));
            CheckVecClose(spine * Vec3.Up,
                TestSkeleton.Direction(lean, KinectJoint.SpineBase, KinectJoint.SpineMid),
                5e-3f, "the bent spine matches the measured direction");

            // Scenario 3: stand up and step back. A pure translation must produce no rotation.
            SkeletonFrameData stepped = TestSkeleton.Transform(lean, Quat.Identity, new Vec3(0f, 0.35f, 0.3f));
            var steppedPose = new SkeletonPose();
            Check(solver.Solve(stepped, steppedPose), "stepped-back pose solves");
            CheckQuatClose(steppedPose.BoneRotations[VmcBones.Spine], spine, 3e-3f,
                "translation does not change any bone rotation");

            // The root must still report the movement, in metres, in character space.
            CheckClose(steppedPose.RootPosition.Z, stepped.Get(KinectJoint.SpineBase).Z, 1e-5f,
                "root Z follows the step back");
            CheckClose(steppedPose.RootPosition.Y, stepped.Get(KinectJoint.SpineBase).Y, 1e-5f,
                "root Y follows the step up");
        }

        /// <summary>Rotates every joint of a chain above a pivot, about that pivot.</summary>
        private static void RotateChainAbove(SkeletonFrameData frame, KinectJoint pivotJoint,
            Vec3 pivot, Quat rotation)
        {
            // SpineBase is the root of the upper body in this model, so everything except the
            // legs and hips rotates with it.
            var moving = new[]
            {
                KinectJoint.SpineBase, KinectJoint.SpineMid, KinectJoint.SpineShoulder,
                KinectJoint.Neck, KinectJoint.Head,
                KinectJoint.ShoulderLeft, KinectJoint.ElbowLeft, KinectJoint.WristLeft,
                KinectJoint.HandLeft, KinectJoint.HandTipLeft, KinectJoint.ThumbLeft,
                KinectJoint.ShoulderRight, KinectJoint.ElbowRight, KinectJoint.WristRight,
                KinectJoint.HandRight, KinectJoint.HandTipRight, KinectJoint.ThumbRight
            };

            for (int i = 0; i < moving.Length; i++)
            {
                int index = (int)moving[i];
                Vec3 p = frame.Positions[index];
                frame.Positions[index] = (rotation * (p - pivot)) + pivot;
            }
        }

        /// <summary>
        /// Two properties the sender depends on: solving the same input twice gives bit-identical
        /// results (no hidden state or randomness), and the emitted bone set contains no
        /// duplicates or lower-body names.
        /// </summary>
        private static void TestEmissionIsDeterministicAndUnique()
        {
            Console.WriteLine();
            Console.WriteLine("Emission determinism and uniqueness");

            // Determinism: a fresh solver must reproduce the same rotations exactly.
            // SendHand is OFF by default now -- the hand is driven from the camera -- so the
            // checks below, which are about the ADVERTISED upper-body set, switch it on.
            var first = new UpperBodySolver { Smoothing = 0f };
            var second = new UpperBodySolver { Smoothing = 0f };
            var poseA = new SkeletonPose();
            var poseB = new SkeletonPose();

            SkeletonFrameData frame = TestSkeleton.RestPose();
            first.Solve(frame, poseA);
            second.Solve(frame, poseB);

            Check(poseA.BoneRotations.Count == poseB.BoneRotations.Count,
                "two independent solvers emit the same bone count");
            foreach (var pair in poseA.BoneRotations)
            {
                Quat other;
                if (!poseB.BoneRotations.TryGetValue(pair.Key, out other))
                {
                    Check(false, "bone present in both runs: " + pair.Key);
                    continue;
                }
                Check(pair.Value.Equals(other), "bit-identical rotation for " + pair.Key,
                    pair.Value + " vs " + other);
            }

            // Uniqueness: names are emitted once each, all upper body, and the count is exact.
            Check(VmcBones.DefaultUpperBody.Length == poseA.BoneRotations.Count,
                "emitted count matches the advertised upper-body set",
                poseA.BoneRotations.Count + " vs " + VmcBones.DefaultUpperBody.Length);

            var seen = new HashSet<string>(StringComparer.Ordinal);
            foreach (string bone in VmcBones.DefaultUpperBody)
            {
                Check(seen.Add(bone), "no duplicate bone in the advertised set: " + bone);
                Check(poseA.BoneRotations.ContainsKey(bone), "advertised bone is emitted: " + bone);
            }

            // The wire payload must contain every emitted bone exactly once per send.
            using (var sender = new VmcSender())
            {
                sender.Configure("127.0.0.1", 1);
                var received = new List<string>();
                int length;

                foreach (var pair in poseA.BoneRotations)
                {
                    byte[] packet = sender.EncodeBone(pair.Key, pair.Value, out length);
                    OscMessage message;
                    if (!OscMessageReader.TryParse(packet, length, out message)) { continue; }

                    string name = message.Arguments[0].StringValue;
                    Check(!received.Contains(name), "encoded packet carries a unique bone: " + name);
                    received.Add(name);
                    CheckClose(message.Arguments[7].FloatValue, pair.Value.W, 1e-9f,
                        "encoded quaternion w matches the solved value for " + name);
                }

                Check(received.Count == poseA.BoneRotations.Count,
                    "every solved bone was encoded", received.Count + " of " + poseA.BoneRotations.Count);
            }
        }

        private static void TestMissingJointsFallBack()
        {
            Console.WriteLine();
            Console.WriteLine("Joint loss handling");

            var solver = new UpperBodySolver { Smoothing = 0f };
            var pose = new SkeletonPose();

            Check(solver.Solve(TestSkeleton.RestPose(), pose), "priming frame solves");

            // Lose the whole left arm below the shoulder. The solver must hold the last
            // known positions rather than snapping the arm back to the rest pose, which is
            // what a user sees as a violent twitch every time Kinect loses a wrist.
            SkeletonFrameData occluded = TestSkeleton.RestPose();
            occluded.Set(KinectJoint.ElbowLeft, Vec3.Zero, false);
            occluded.Set(KinectJoint.WristLeft, Vec3.Zero, false);
            occluded.Set(KinectJoint.HandLeft, Vec3.Zero, false);
            occluded.Set(KinectJoint.HandTipLeft, Vec3.Zero, false);

            var occludedPose = new SkeletonPose();
            Check(solver.Solve(occluded, occludedPose), "solves with the left arm occluded");
            Check(occludedPose.BoneRotations.ContainsKey(VmcBones.LeftUpperArm),
                "occluded arm still produces bones");
            CheckQuatClose(occludedPose.BoneRotations[VmcBones.LeftUpperArm], Quat.Identity, 3e-3f,
                "occluded arm holds its last known orientation");
            Check(occludedPose.BoneRotations.ContainsKey(VmcBones.RightUpperArm),
                "the other arm is unaffected");

            // A frame missing the torso must be rejected outright rather than guessed at.
            SkeletonFrameData noTorso = TestSkeleton.RestPose();
            noTorso.Set(KinectJoint.SpineShoulder, Vec3.Zero, false);
            noTorso.Set(KinectJoint.ShoulderLeft, Vec3.Zero, false);
            var noTorsoPose = new SkeletonPose();
            Check(!solver.Solve(noTorso, noTorsoPose), "frame without a stable torso is rejected");
        }

        private static void TestUntrackedAndDegenerateInput()
        {
            Console.WriteLine();
            Console.WriteLine("Degenerate input");

            var solver = new UpperBodySolver { Smoothing = 0f };

            var empty = new SkeletonFrameData();
            empty.Clear();
            var emptyPose = new SkeletonPose();
            Check(!solver.Solve(empty, emptyPose), "untracked body is rejected");
            Check(emptyPose.BoneRotations.Count == 0, "rejected frame produces no bones");

            // Every joint at the origin: the frames are degenerate but nothing may be NaN.
            var degenerate = new SkeletonFrameData { BodyTracked = true };
            for (int i = 0; i < SkeletonFrameData.JointCount; i++)
            {
                degenerate.Positions[i] = Vec3.Zero;
                degenerate.Tracked[i] = true;
            }
            var degeneratePose = new SkeletonPose();
            solver.Solve(degenerate, degeneratePose);
            Check(true, "degenerate frame does not throw");
            foreach (var pair in degeneratePose.BoneRotations)
            {
                Check(pair.Value.IsFinite(), "degenerate output finite: " + pair.Key);
            }

            // Non-finite joint data must not leak into the output.
            SkeletonFrameData nanFrame = TestSkeleton.RestPose();
            nanFrame.Set(KinectJoint.ElbowRight, new Vec3(float.NaN, 1f, 1f), true);
            var nanPose = new SkeletonPose();
            solver.Solve(nanFrame, nanPose);
            foreach (var pair in nanPose.BoneRotations)
            {
                Check(pair.Value.IsFinite(), "NaN input does not leak into: " + pair.Key);
            }
        }

        private static void TestHeadAndHipsOwnership()
        {
            Console.WriteLine();
            Console.WriteLine("Head and hips ownership");

            SkeletonFrameData frame = TestSkeleton.RestPose();

            var withHead = new UpperBodySolver { Smoothing = 0f, SendHead = true };
            var poseWithHead = new SkeletonPose();
            withHead.Solve(frame, poseWithHead);
            Check(poseWithHead.BoneRotations.ContainsKey(VmcBones.Head), "SendHead=true emits Head");
            Check(poseWithHead.BoneRotations.ContainsKey(VmcBones.Neck), "SendHead=true emits Neck");

            var withoutHead = new UpperBodySolver { Smoothing = 0f, SendHead = false };
            var poseWithoutHead = new SkeletonPose();
            withoutHead.Solve(frame, poseWithoutHead);
            Check(!poseWithoutHead.BoneRotations.ContainsKey(VmcBones.Head), "SendHead=false omits Head");
            Check(!poseWithoutHead.BoneRotations.ContainsKey(VmcBones.Neck), "SendHead=false omits Neck");
            Check(!poseWithoutHead.BoneRotations.ContainsKey(VmcBones.Hips),
                "hips are not emitted, so the idle animation keeps the pelvis");
            Check(poseWithoutHead.BoneRotations.ContainsKey(VmcBones.Spine), "spine is still emitted");
            Check(poseWithoutHead.BoneRotations.Count == 11, "upper-body-only pose has 11 bones",
                "got " + poseWithoutHead.BoneRotations.Count);

            // Nothing below the waist may ever be emitted: that is the whole point of the
            // seated, upper-body-only mode.
            foreach (var pair in poseWithoutHead.BoneRotations)
            {
                Check(pair.Key.IndexOf("Leg", StringComparison.Ordinal) < 0
                    && pair.Key.IndexOf("Foot", StringComparison.Ordinal) < 0
                    && pair.Key.IndexOf("Toes", StringComparison.Ordinal) < 0,
                    "no lower-body bone emitted: " + pair.Key);
            }
        }

        private static void TestSmoothing()
        {
            Console.WriteLine();
            Console.WriteLine("Smoothing");

            var smoothed = new UpperBodySolver { Smoothing = 0.8f };
            var first = new SkeletonPose();
            smoothed.Solve(TestSkeleton.RestPose(), first);
            Quat before = first.BoneRotations[VmcBones.LeftUpperArm];

            // A sudden 40 degree change must be damped, not applied in full.
            Quat expected = Quat.AxisAngle(Vec3.Backward, 40f * 0.0174532925f);
            Vec3 raised = (expected * TestSkeleton.RestArmDirection()).Normalized();
            SkeletonFrameData jerked = TestSkeleton.NeutralTorso();
            TestSkeleton.PlaceArms(jerked, raised);

            var second = new SkeletonPose();
            smoothed.Solve(jerked, second);
            Quat after = second.BoneRotations[VmcBones.LeftUpperArm];

            float moved = RotationAngleDegrees(before * after.Conjugate());
            Check(moved > 1f && moved < 39f, "smoothing damps a 40 degree jump",
                "moved " + moved.ToString("F1") + " degrees in one frame");

            // With smoothing off, the full change lands immediately.
            var raw = new UpperBodySolver { Smoothing = 0f };
            var rawPose = new SkeletonPose();
            raw.Solve(jerked, rawPose);
            CheckClose(RotationAngleDegrees(rawPose.BoneRotations[VmcBones.LeftUpperArm]), 40f, 0.5f,
                "smoothing off applies the full change");

            // Repeated frames converge towards the target instead of stalling.
            var converge = new UpperBodySolver { Smoothing = 0.7f };
            for (int i = 0; i < 40; i++)
            {
                var step = new SkeletonPose();
                converge.Solve(jerked, step);
                if (i == 39)
                {
                    CheckClose(RotationAngleDegrees(step.BoneRotations[VmcBones.LeftUpperArm]), 40f, 1f,
                        "smoothing converges on a held pose");
                }
            }
        }

        // ---------------------------------------------------------------- protocol

        private static void TestOscLayout()
        {
            Console.WriteLine();
            Console.WriteLine("OSC wire layout");

            var writer = new OscMessageWriter(64);
            writer.Begin("/VMC/Ext/Bone/Pos");
            writer.AddString("LeftUpperArm");
            writer.AddFloat(0f);
            writer.AddFloat(0f);
            writer.AddFloat(0f);
            writer.AddFloat(-2.25f);
            writer.AddFloat(0.5f);
            writer.AddFloat(0.25f);
            writer.AddFloat(0.75f);
            int length = writer.End();
            byte[] packet = writer.Buffer;

            Check(length % 4 == 0, "packet length is 4-byte aligned", "length = " + length);

            // OSC requires every string to be null-terminated inside its padding.
            int offset = 0;
            string address;
            Check(TryReadString(packet, length, ref offset, out address), "address is null terminated");
            Check(address == "/VMC/Ext/Bone/Pos", "address round trips", address);
            Check(packet[offset - 1] == 0, "address terminates with a zero byte");
            Check(offset % 4 == 0, "address is padded to a 4-byte boundary", "offset = " + offset);

            string tags;
            Check(TryReadString(packet, length, ref offset, out tags), "type tags are null terminated");
            Check(tags == ",sfffffff", "type tags describe one string and seven floats", tags);
            Check(offset % 4 == 0, "type tags are padded to a 4-byte boundary", "offset = " + offset);

            string bone;
            Check(TryReadString(packet, length, ref offset, out bone), "bone name is null terminated");
            Check(bone == "LeftUpperArm", "bone name round trips", bone);
            Check(packet[offset - 1] == 0, "bone name terminates with a zero byte");

            if (_dumpPackets)
            {
                DumpPacket(packet, length, "Bone/Pos: LeftUpperArm, first float -2.25");
                Console.WriteLine("  offset after all strings = " + offset + ", packet length = " + length);
            }

            // First float is -2.25, which is the bone-name string followed by four floats in a
            // real bone message; this test writes its own sequence, so the layout checks below
            // follow the writer's own argument order.
            CheckClose(ReadBigEndianFloat(packet, offset), 0f, 1e-6f, "first float is 0");
            CheckClose(ReadBigEndianFloat(packet, offset + 4), 0f, 1e-6f, "second float is 0");
            CheckClose(ReadBigEndianFloat(packet, offset + 8), 0f, 1e-6f, "third float is 0");
            CheckClose(ReadBigEndianFloat(packet, offset + 12), -2.25f, 1e-6f, "-2.25 decodes big-endian");
            CheckClose(ReadBigEndianFloat(packet, offset + 16), 0.5f, 1e-6f, "fifth float is 0.5");
            CheckClose(ReadBigEndianFloat(packet, offset + 20), 0.25f, 1e-6f, "sixth float is 0.25");
            CheckClose(ReadBigEndianFloat(packet, offset + 24), 0.75f, 1e-6f, "seventh float is 0.75");

            // -2.25 is 0xC0100000 big-endian; the little-endian reading of the same bytes is
            // not the value we wrote, which proves the byte order is big-endian.
            Check(packet[offset + 12] == 0xC0 && packet[offset + 13] == 0x10,
                "-2.25 sign/exponent bytes are 0xC0 0x10",
                string.Format("got 0x{0:X2} 0x{1:X2}", packet[offset + 12], packet[offset + 13]));
            CheckClose(ReadBigEndianFloat(packet, offset + 12), -2.25f, 1e-6f,
                "-2.25 round trips through the big-endian reader");
            Check(offset + 28 == length, "the packet ends exactly after the seventh float",
                "expected " + (offset + 28) + ", length " + length);

            // Integer arguments are big-endian two's complement.
            writer.Begin("/VMC/Ext/OK");
            writer.AddInt(1);
            writer.AddInt(3);
            writer.AddInt(258);   // 0x00000102, catches byte-swapping
            writer.AddInt(1);
            int okLength = writer.End();
            byte[] okPacket = writer.Buffer;

            int okOffset = 0;
            string okAddress, okTags;
            TryReadString(okPacket, okLength, ref okOffset, out okAddress);
            TryReadString(okPacket, okLength, ref okOffset, out okTags);
            Check(okTags == ",iiii", "OK type tags are four ints", okTags);

            if (_dumpPackets) { DumpPacket(okPacket, okLength, "OK: 1, 3, 258, 1"); }

            Check(ReadBigEndianInt(okPacket, okOffset) == 1, "first int decodes as 1");
            Check(ReadBigEndianInt(okPacket, okOffset + 8) == 258, "258 decodes big-endian",
                "got " + ReadBigEndianInt(okPacket, okOffset + 8));
            Check(ReadLittleEndianInt(okPacket, okOffset + 8) != 258,
                "the same four bytes read little-endian would be wrong");
        }

        private static bool _dumpPackets;

        private static void DumpPacket(byte[] packet, int length, string title)
        {
            Console.WriteLine("  --- packet dump: " + title + " (" + length + " bytes) ---");
            var line = new StringBuilder();
            for (int i = 0; i < length; i++)
            {
                if (i % 16 == 0)
                {
                    if (line.Length > 0) { Console.WriteLine("  " + line); }
                    line = new StringBuilder();
                    line.Append(i.ToString("D3")).Append(": ");
                }
                line.Append(packet[i].ToString("X2")).Append(' ');
            }
            if (line.Length > 0) { Console.WriteLine("  " + line); }
        }

        private static float ReadBigEndianFloat(byte[] buffer, int offset)
        {
            byte[] raw = { buffer[offset + 3], buffer[offset + 2], buffer[offset + 1], buffer[offset] };
            return BitConverter.ToSingle(raw, 0);
        }

        private static float ReadLittleEndianFloat(byte[] buffer, int offset)
        {
            byte[] raw = { buffer[offset], buffer[offset + 1], buffer[offset + 2], buffer[offset + 3] };
            return BitConverter.ToSingle(raw, 0);
        }

        private static int ReadBigEndianInt(byte[] buffer, int offset)
        {
            return (buffer[offset] << 24) | (buffer[offset + 1] << 16)
                 | (buffer[offset + 2] << 8) | buffer[offset + 3];
        }

        private static int ReadLittleEndianInt(byte[] buffer, int offset)
        {
            return (buffer[offset + 3] << 24) | (buffer[offset + 2] << 16)
                 | (buffer[offset + 1] << 8) | buffer[offset];
        }

        private static bool TryReadString(byte[] buffer, int length, ref int offset, out string value)
        {
            value = null;
            int start = offset;
            int end = start;
            while (end < length && buffer[end] != 0) { end++; }
            if (end >= length) { return false; }
            value = Encoding.UTF8.GetString(buffer, start, end - start);
            int consumed = end - start + 1;
            offset = start + consumed + ((4 - (consumed % 4)) % 4);
            return true;
        }

        private static void TestOscRoundTrip()
        {
            Console.WriteLine();
            Console.WriteLine("OSC round trip");

            var writer = new OscMessageWriter();
            writer.Begin("/VMC/Ext/Bone/Pos");
            writer.AddString("RightLowerArm");
            writer.AddFloat(0f);
            writer.AddFloat(0f);
            writer.AddFloat(0f);
            writer.AddFloat(-0.123456f);
            writer.AddFloat(0.654321f);
            writer.AddFloat(0.5f);
            writer.AddFloat(0.987654f);
            int length = writer.End();

            OscMessage parsed;
            Check(OscMessageReader.TryParse(writer.Buffer, length, out parsed), "packet parses");
            if (parsed == null) { return; }

            Check(parsed.Address == "/VMC/Ext/Bone/Pos", "address round trips", parsed.Address);
            Check(parsed.Arguments.Length == 8, "8 arguments round trip", "got " + parsed.Arguments.Length);
            Check(parsed.Arguments[0].Type == 's' && parsed.Arguments[0].StringValue == "RightLowerArm",
                "bone name round trips", parsed.Arguments[0].ToString());
            CheckClose(parsed.Arguments[4].FloatValue, -0.123456f, 1e-6f, "float round trips exactly");
            CheckClose(parsed.Arguments[7].FloatValue, 0.987654f, 1e-6f, "quaternion w round trips");

            // A range of float magnitudes, including values whose byte pattern would reveal
            // a byte-order mistake.
            float[] samples = { 0f, 1f, -1f, 0.5f, -0.5f, 123.456f, -0.000123f, 1e-6f, 1234.5f };
            for (int i = 0; i < samples.Length; i++)
            {
                writer.Begin("/VMC/Ext/T");
                writer.AddFloat(samples[i]);
                int sampleLength = writer.End();
                OscMessage sample;
                Check(OscMessageReader.TryParse(writer.Buffer, sampleLength, out sample),
                    "sample float parses: " + samples[i]);
                if (sample != null)
                {
                    CheckClose(sample.Arguments[0].FloatValue, samples[i], 1e-12f,
                        "sample float round trips: " + samples[i]);
                }
            }

            // Strings of every alignment class must survive padding.
            string[] names = { "", "a", "ab", "abc", "abcd", "abcde", "LeftUpperArm", "UpperChest" };
            for (int i = 0; i < names.Length; i++)
            {
                writer.Begin("/VMC/Ext/Bone/Pos");
                writer.AddString(names[i]);
                writer.AddFloat(1f);
                writer.AddFloat(2f);
                writer.AddFloat(3f);
                writer.AddFloat(0f);
                writer.AddFloat(0f);
                writer.AddFloat(0f);
                writer.AddFloat(1f);
                int nameLength = writer.End();
                OscMessage nameMessage;
                Check(OscMessageReader.TryParse(writer.Buffer, nameLength, out nameMessage),
                    "name packet parses: '" + names[i] + "'");
                if (nameMessage != null)
                {
                    Check(nameMessage.Arguments[0].StringValue == names[i],
                        "name round trips: '" + names[i] + "'",
                        "got '" + nameMessage.Arguments[0].StringValue + "'");
                    CheckClose(nameMessage.Arguments[1].FloatValue, 1f, 1e-9f,
                        "float after name is intact: '" + names[i] + "'");
                }
            }

            // Garbage must be rejected rather than throwing.
            OscMessage junk;
            Check(!OscMessageReader.TryParse(new byte[] { 1, 2, 3, 4, 5, 6, 7, 8 }, 8, out junk), "garbage rejected");
            Check(!OscMessageReader.TryParse(new byte[0], 0, out junk), "empty packet rejected");

            // A packet that declares one float but stops before the payload is truncated.
            Check(!OscMessageReader.TryParse(
                new byte[] { (byte)'/', 0, 0, 0, (byte)',', (byte)'f', 0, 0, 0, 0, 0 }, 11, out junk),
                "truncated float payload rejected");

            // An address that is not an OSC path, and a tag string that does not start with
            // a comma, are both malformed.
            Check(!OscMessageReader.TryParse(
                new byte[] { (byte)'x', 0, 0, 0, (byte)',', 0, 0, 0 }, 8, out junk),
                "address without a leading slash rejected");
            Check(!OscMessageReader.TryParse(
                new byte[] { (byte)'/', 0, 0, 0, (byte)'f', 0, 0, 0 }, 8, out junk),
                "type tags without a leading comma rejected");
        }

        private static void TestVmcMessageShape()
        {
            Console.WriteLine();
            Console.WriteLine("VMC message shape");

            using (var sender = new VmcSender())
            {
                int length;
                byte[] packet = sender.EncodeBone(VmcBones.Chest, Quat.Identity, out length);

                OscMessage parsed;
                Check(OscMessageReader.TryParse(packet, length, out parsed), "encoded bone parses");
                if (parsed != null)
                {
                    Check(parsed.Address == "/VMC/Ext/Bone/Pos", "bone address is correct", parsed.Address);
                    Check(parsed.Arguments[0].StringValue == VmcBones.Chest, "bone name is 'Chest'");

                    // Positions must be zero: Warudo overrides local rotations and does not
                    // solve rotations from positions, so sending positions can only hurt.
                    Check(parsed.Arguments[1].FloatValue == 0f
                        && parsed.Arguments[2].FloatValue == 0f
                        && parsed.Arguments[3].FloatValue == 0f,
                        "bone positions are zero (rotation-only mode)");
                    Check(parsed.Arguments[7].FloatValue == 1f, "identity quaternion has w = 1");
                }

                var statusPose = new SkeletonPose { Time = 3.5f, Tracking = true };
                packet = sender.EncodeStatus(statusPose, out length);
                Check(OscMessageReader.TryParse(packet, length, out parsed), "encoded status parses");
                if (parsed != null)
                {
                    Check(parsed.Address == "/VMC/Ext/OK", "status address is /VMC/Ext/OK", parsed.Address);
                    Check(parsed.Arguments.Length == 4, "status carries the VMC 2.7 argument count");
                    Check(parsed.Arguments[0].IntValue == 1, "loaded flag is 1");
                    Check(parsed.Arguments[1].IntValue == 3, "calibration state is Calibrated");
                    Check(parsed.Arguments[2].IntValue == 0, "calibration mode is Normal");
                    Check(parsed.Arguments[3].IntValue == 1, "tracking status is OK");
                }
            }

            // Only the bones the receiver can actually use may be advertised.
            Check(VmcBones.DefaultUpperBody.Length == 11, "default upper-body set has 11 bones");
            foreach (string bone in VmcBones.DefaultUpperBody)
            {
                Check(VmcBones.IsUpperBody(bone), "advertised bone is upper body: " + bone);
            }
            Check(!VmcBones.IsUpperBody("LeftUpperLeg"), "legs are not upper body");
            Check(!VmcBones.IsUpperBody("Hips"), "hips are not part of the emitted set");
        }

        private static void TestUdpDelivery()
        {
            Console.WriteLine();
            Console.WriteLine("UDP delivery");

            var receiver = new System.Net.Sockets.UdpClient(
                new System.Net.IPEndPoint(System.Net.IPAddress.Loopback, 0));
            int port = ((System.Net.IPEndPoint)receiver.Client.LocalEndPoint).Port;

            try
            {
                var sender = new VmcSender();
                sender.Configure("127.0.0.1", port);
                sender.SendRootPosition = true;

                var pose = new SkeletonPose { Time = 1f, Tracking = true };
                pose.SetBone(VmcBones.Spine, Quat.AxisAngle(Vec3.Up, 0.25f));
                pose.SetBone(VmcBones.LeftUpperArm, Quat.AxisAngle(Vec3.Backward, -0.5f));
                pose.HasRootPosition = true;
                pose.RootPosition = new Vec3(0.05f, 0.1f, 0.15f);

                Check(sender.Send(pose, true), "send reports success");

                List<OscMessage> received = Drain(receiver, 1200);
                Check(Contains(received, "/VMC/Ext/OK"), "status message delivered");
                Check(Contains(received, "/VMC/Ext/T"), "time message delivered");
                Check(Contains(received, "/VMC/Ext/Root/Pos"), "root position delivered");
                Check(CountOf(received, "/VMC/Ext/Bone/Pos") == 2, "both bone messages delivered",
                    "got " + CountOf(received, "/VMC/Ext/Bone/Pos"));
                Check(sender.PacketsSent >= 5, "packet counter advanced", "count = " + sender.PacketsSent);
                Check(sender.BytesSent > 0, "byte counter advanced");
                Check(sender.LastError == null, "no send error", sender.LastError);

                // The root message must carry the position we asked for.
                OscMessage root = Find(received, "/VMC/Ext/Root/Pos");
                if (root != null)
                {
                    Check(root.Arguments[0].StringValue == "root", "root message names the root");
                    CheckClose(root.Arguments[1].FloatValue, 0.05f, 1e-6f, "root x delivered");
                    CheckClose(root.Arguments[2].FloatValue, 0.1f, 1e-6f, "root y delivered");
                    CheckClose(root.Arguments[3].FloatValue, 0.15f, 1e-6f, "root z delivered");
                    Check(root.Arguments.Length == 14, "root carries the full VMC 2.1 argument set",
                        "got " + root.Arguments.Length);
                    Check(root.Arguments[7].Type == 'f' && root.Arguments[13].Type == 'f',
                        "root scale and offset fields are floats");
                }
                else
                {
                    Check(false, "root message parsed");
                }

                // The switch must be respected.
                sender.SendRootPosition = false;
                sender.Send(pose, true);
                List<OscMessage> after = Drain(receiver, 600);
                Check(!Contains(after, "/VMC/Ext/Root/Pos"), "root position suppressed when disabled");
                Check(Contains(after, "/VMC/Ext/Bone/Pos"), "bone messages still sent");

                // Reconfiguring to a different port must take effect.
                var receiver2 = new System.Net.Sockets.UdpClient(
                    new System.Net.IPEndPoint(System.Net.IPAddress.Loopback, 0));
                try
                {
                    int port2 = ((System.Net.IPEndPoint)receiver2.Client.LocalEndPoint).Port;
                    sender.Configure("127.0.0.1", port2);
                    sender.Send(pose, true);
                    List<OscMessage> moved = Drain(receiver2, 800);
                    Check(Contains(moved, "/VMC/Ext/Bone/Pos"), "reconfigured port receives traffic");

                    List<OscMessage> stale = Drain(receiver, 300);
                    Check(!Contains(stale, "/VMC/Ext/Bone/Pos"), "old port stops receiving traffic");
                }
                finally
                {
                    receiver2.Close();
                }

                sender.Dispose();
                Check(true, "sender disposes without throwing");
            }
            finally
            {
                receiver.Close();
            }
        }

        private static List<OscMessage> Drain(System.Net.Sockets.UdpClient receiver, int milliseconds)
        {
            var messages = new List<OscMessage>();
            receiver.Client.ReceiveTimeout = 120;
            DateTime deadline = DateTime.UtcNow.AddMilliseconds(milliseconds);
            while (DateTime.UtcNow < deadline)
            {
                try
                {
                    var endpoint = new System.Net.IPEndPoint(System.Net.IPAddress.Any, 0);
                    byte[] data = receiver.Receive(ref endpoint);
                    OscMessage message;
                    if (OscMessageReader.TryParse(data, data.Length, out message)) { messages.Add(message); }
                }
                catch (System.Net.Sockets.SocketException)
                {
                    // Timeout: keep waiting until the deadline, but stop early once the
                    // traffic has clearly stopped arriving.
                    if (messages.Count > 0) { break; }
                }
            }
            return messages;
        }

        private static bool Contains(List<OscMessage> messages, string address)
        {
            return Find(messages, address) != null;
        }

        private static OscMessage Find(List<OscMessage> messages, string address)
        {
            for (int i = 0; i < messages.Count; i++)
            {
                if (messages[i].Address == address) { return messages[i]; }
            }
            return null;
        }

        private static int CountOf(List<OscMessage> messages, string address)
        {
            int count = 0;
            for (int i = 0; i < messages.Count; i++)
            {
                if (messages[i].Address == address) { count++; }
            }
            return count;
        }
    }
}
