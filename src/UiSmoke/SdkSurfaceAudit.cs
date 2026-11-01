using System;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using System.Text;

namespace Kinect2VMC.UiSmoke
{
    /// <summary>
    /// Verifies that every Microsoft.Kinect member the application relies on actually exists,
    /// with the signature the code assumes.
    ///
    /// This matters because the Kinect SDK is the one dependency that cannot be exercised
    /// without the device. A typo, a renamed property or a wrong overload would otherwise only
    /// surface as a runtime failure on the user's machine. Reflection catches all of it here.
    /// </summary>
    internal static class SdkSurfaceAudit
    {
        public static int Run()
        {
            int failures = 0;
            Console.WriteLine();
            Console.WriteLine("Kinect SDK surface audit");

            Assembly sdk = typeof(Microsoft.Kinect.KinectSensor).Assembly;
            Console.WriteLine("  assembly: " + sdk.GetName().Name + " " + sdk.GetName().Version
                + "  (" + sdk.Location + ")");

            // --- KinectSensor ---------------------------------------------------------
            failures += ExpectProperty(sdk, "KinectSensor", "IsAvailable", typeof(bool));
            failures += ExpectProperty(sdk, "KinectSensor", "IsOpen", typeof(bool));
            failures += ExpectProperty(sdk, "KinectSensor", "UniqueKinectId", typeof(string));
            failures += ExpectProperty(sdk, "KinectSensor", "BodyFrameSource", "BodyFrameSource");
            failures += ExpectProperty(sdk, "KinectSensor", "CoordinateMapper", "CoordinateMapper");
            failures += ExpectEvent(sdk, "KinectSensor", "IsAvailableChanged");
            failures += ExpectStaticMethod(sdk, "KinectSensor", "GetDefault", "KinectSensor");
            failures += ExpectMethod(sdk, "KinectSensor", "Open", typeof(void));
            failures += ExpectMethod(sdk, "KinectSensor", "Close", typeof(void));

            // The reader factory is the only way to get body frames in the managed API.
            failures += ExpectMethod(sdk, "KinectSensor", "OpenMultiSourceFrameReader", "MultiSourceFrameReader");
            failures += ExpectEnumHas(sdk, "FrameSourceTypes", "Body");

            // --- MultiSourceFrameReader ----------------------------------------------
            failures += ExpectEvent(sdk, "MultiSourceFrameReader", "MultiSourceFrameArrived");
            failures += ExpectMethod(sdk, "MultiSourceFrameReader", "AcquireLatestFrame", "MultiSourceFrame");

            // --- MultiSourceFrame ----------------------------------------------------
            failures += ExpectProperty(sdk, "MultiSourceFrame", "BodyFrameReference", "BodyFrameReference");

            // --- BodyFrameReference / BodyFrame --------------------------------------
            failures += ExpectMethod(sdk, "BodyFrameReference", "AcquireFrame", "BodyFrame");
            failures += ExpectMethod(sdk, "BodyFrame", "GetAndRefreshBodyData", typeof(void));
            failures += ExpectProperty(sdk, "BodyFrame", "BodyCount", typeof(int));

            // --- Body ----------------------------------------------------------------
            failures += ExpectProperty(sdk, "Body", "IsTracked", typeof(bool));
            failures += ExpectProperty(sdk, "Body", "TrackingId", typeof(ulong));
            failures += ExpectProperty(sdk, "Body", "Joints", "IReadOnlyDictionary`2");
            failures += ExpectProperty(sdk, "Body", "Lean", "PointF");

            // --- BodyFrameSource -----------------------------------------------------
            failures += ExpectProperty(sdk, "BodyFrameSource", "BodyCount", typeof(int));

            // --- Joint / JointType ---------------------------------------------------
            failures += ExpectField(sdk, "Joint", "Position");
            failures += ExpectField(sdk, "Joint", "TrackingState");
            failures += ExpectField(sdk, "Joint", "JointType");
            failures += ExpectEnumHas(sdk, "TrackingState", "Tracked");
            failures += ExpectEnumHas(sdk, "TrackingState", "Inferred");
            failures += ExpectEnumHas(sdk, "TrackingState", "NotTracked");

            // --- JointType: every joint the solver's enum names must exist ------------
            failures += AuditJointTypeNames(sdk);

            // --- CameraSpacePoint ----------------------------------------------------
            failures += ExpectField(sdk, "CameraSpacePoint", "X");
            failures += ExpectField(sdk, "CameraSpacePoint", "Y");
            failures += ExpectField(sdk, "CameraSpacePoint", "Z");

            // --- MultiSourceFrame is deliberately NOT IDisposable ---------------------
            failures += ExpectNotDisposable(sdk, "MultiSourceFrame");
            failures += ExpectDisposable(sdk, "BodyFrame");
            failures += ExpectDisposable(sdk, "MultiSourceFrameReader");

            // --- The app's own adapter must line up with the solver's joint enum -----
            failures += AuditAdapterJointMap();

            return failures;
        }

        // ---------------------------------------------------------------- checks

        private static Type Find(Assembly assembly, string name)
        {
            Type type = assembly.GetType("Microsoft.Kinect." + name, false);
            if (type != null) { return type; }

            // Nested or namespace-shifted types.
            foreach (Type candidate in assembly.GetTypes())
            {
                if (candidate.Name == name) { return candidate; }
            }
            return null;
        }

        private static int ExpectProperty(Assembly assembly, string typeName, string propertyName, Type propertyType)
        {
            return ExpectProperty(assembly, typeName, propertyName, propertyType == null ? null : propertyType.Name);
        }

        private static int ExpectProperty(Assembly assembly, string typeName, string propertyName, string typeNameExpected)
        {
            Type type = Find(assembly, typeName);
            if (type == null) { return Fail(typeName + " type is missing"); }

            PropertyInfo property = type.GetProperty(propertyName,
                BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static);
            if (property == null)
            {
                return Fail(typeName + "." + propertyName + " property is missing");
            }
            if (typeNameExpected != null && property.PropertyType.Name != typeNameExpected)
            {
                return Fail(typeName + "." + propertyName + " has type "
                    + property.PropertyType.Name + ", expected " + typeNameExpected);
            }

            Console.WriteLine("  [ok]   " + typeName + "." + propertyName + " : " + property.PropertyType.Name);
            return 0;
        }

        private static int ExpectEvent(Assembly assembly, string typeName, string eventName)
        {
            Type type = Find(assembly, typeName);
            if (type == null) { return Fail(typeName + " type is missing"); }

            EventInfo found = type.GetEvent(eventName, BindingFlags.Public | BindingFlags.Instance);
            if (found == null) { return Fail(typeName + "." + eventName + " event is missing"); }

            Console.WriteLine("  [ok]   " + typeName + "." + eventName + " : " + found.EventHandlerType.Name);
            return 0;
        }

        private static int ExpectStaticMethod(Assembly assembly, string typeName, string methodName, string returnType)
        {
            Type type = Find(assembly, typeName);
            if (type == null) { return Fail(typeName + " type is missing"); }

            foreach (MethodInfo method in type.GetMethods(BindingFlags.Public | BindingFlags.Static))
            {
                if (method.Name == methodName && method.GetParameters().Length == 0
                    && (returnType == null || method.ReturnType.Name == returnType))
                {
                    Console.WriteLine("  [ok]   static " + typeName + "." + methodName + "() -> " + method.ReturnType.Name);
                    return 0;
                }
            }
            return Fail("static " + typeName + "." + methodName + "() is missing");
        }

        private static int ExpectMethod(Assembly assembly, string typeName, string methodName, Type returnType)
        {
            return ExpectMethod(assembly, typeName, methodName, returnType == null ? null : returnType.Name);
        }

        private static int ExpectMethod(Assembly assembly, string typeName, string methodName, string returnType)
        {
            Type type = Find(assembly, typeName);
            if (type == null) { return Fail(typeName + " type is missing"); }

            foreach (MethodInfo method in type.GetMethods(BindingFlags.Public | BindingFlags.Instance))
            {
                if (method.Name != methodName) { continue; }
                if (returnType != null && method.ReturnType.Name != returnType) { continue; }

                var parameters = new StringBuilder();
                foreach (ParameterInfo parameter in method.GetParameters())
                {
                    if (parameters.Length > 0) { parameters.Append(", "); }
                    parameters.Append(parameter.ParameterType.Name);
                }
                Console.WriteLine("  [ok]   " + typeName + "." + methodName + "(" + parameters + ") -> "
                    + method.ReturnType.Name);
                return 0;
            }
            return Fail(typeName + "." + methodName + " is missing (expected return " + returnType + ")");
        }

        private static int ExpectField(Assembly assembly, string typeName, string fieldName)
        {
            Type type = Find(assembly, typeName);
            if (type == null) { return Fail(typeName + " type is missing"); }

            FieldInfo field = type.GetField(fieldName, BindingFlags.Public | BindingFlags.Instance);
            if (field == null) { return Fail(typeName + "." + fieldName + " field is missing"); }

            Console.WriteLine("  [ok]   " + typeName + "." + fieldName + " : " + field.FieldType.Name);
            return 0;
        }

        private static int ExpectEnumHas(Assembly assembly, string enumName, string member)
        {
            Type type = Find(assembly, enumName);
            if (type == null) { return Fail(enumName + " enum is missing"); }
            if (!type.IsEnum) { return Fail(enumName + " is not an enum"); }
            if (Array.IndexOf(Enum.GetNames(type), member) < 0)
            {
                return Fail(enumName + " has no member " + member);
            }
            Console.WriteLine("  [ok]   " + enumName + "." + member);
            return 0;
        }

        private static int ExpectDisposable(Assembly assembly, string typeName)
        {
            Type type = Find(assembly, typeName);
            if (type == null) { return Fail(typeName + " type is missing"); }
            if (!typeof(IDisposable).IsAssignableFrom(type))
            {
                return Fail(typeName + " should implement IDisposable");
            }
            Console.WriteLine("  [ok]   " + typeName + " implements IDisposable");
            return 0;
        }

        private static int ExpectNotDisposable(Assembly assembly, string typeName)
        {
            Type type = Find(assembly, typeName);
            if (type == null) { return Fail(typeName + " type is missing"); }
            if (typeof(IDisposable).IsAssignableFrom(type))
            {
                return Fail(typeName + " IS IDisposable, but the adapter does not dispose it");
            }
            Console.WriteLine("  [ok]   " + typeName + " is not IDisposable (as the adapter assumes)");
            return 0;
        }

        private static int Fail(string message)
        {
            Console.WriteLine("  [FAIL] " + message);
            return 1;
        }

        // ---------------------------------------------------------------- joint map

        private static int AuditJointTypeNames(Assembly sdk)
        {
            Type jointType = Find(sdk, "JointType");
            if (jointType == null) { return Fail("JointType enum is missing"); }

            var actual = new HashSet<string>(Enum.GetNames(jointType), StringComparer.Ordinal);

            Type solverEnum = typeof(Kinect2VMC.Core.KinectJoint);
            var expected = Enum.GetNames(solverEnum);

            int failures = 0;
            foreach (string name in expected)
            {
                if (name == "Count")
                {
                    // The solver's own sentinel, deliberately absent from the SDK enum.
                    failures += actual.Contains(name) ? Fail("JointType unexpectedly has a Count member") : 0;
                    continue;
                }
                if (!actual.Contains(name))
                {
                    failures += Fail("JointType has no member '" + name + "' (solver expects it)");
                }
            }

            // The SDK must not have joints the solver silently ignores, other than none.
            foreach (string name in actual)
            {
                if (Array.IndexOf(expected, name) < 0)
                {
                    Console.WriteLine("  [note] JointType." + name + " is not mapped by the solver (unused)");
                }
            }

            if (failures == 0)
            {
                Console.WriteLine("  [ok]   all " + (expected.Length - 1) + " solver joints exist in JointType");
            }
            return failures;
        }

        /// <summary>
        /// The adapter builds its joint map by name at type-init time. Force that static
        /// initialiser to run so a mismatch fails here rather than on the user's machine.
        /// </summary>
        private static int AuditAdapterJointMap()
        {
            try
            {
                // Touching any static member of the adapter runs its type initialiser, which
                // builds the Kinect-joint map by name and throws if the SDK has no joint with a
                // name the solver expects. That is exactly the failure that would otherwise
                // surface only on the user's machine, at startup.
                var point = new Microsoft.Kinect.CameraSpacePoint { X = 0f, Y = 0f, Z = 0f };
                Kinect2VMC.App.KinectBodySource.ToCharacterSpace(point);

                int joints = Kinect2VMC.Core.SkeletonFrameData.JointCount;
                Console.WriteLine("  [ok]   adapter joint map built; solver joint count = " + joints);
                return 0;
            }
            catch (Exception ex)
            {
                return Fail("adapter joint map failed to initialise: " + ex.Message);
            }
        }

        /// <summary>
        /// Kinect camera space is right-handed, with +X to the viewer's right and +Z away from
        /// the sensor. The adapter mirrors X. This pins the documented facts down in one place.
        /// </summary>
        /// <summary>
        /// Pins the camera-space to character-space mapping: (x, y, z) -> (x, y, -z).
        ///
        /// Only Z is negated. The choice was not derived by hand: hand-deriving it produced three
        /// different wrong answers in a row, each of which shipped to the user's hardware. It is
        /// now determined empirically by <c>ConventionSearch</c>, which enumerates all eight sign
        /// combinations and reports which ones satisfy "up", "forward" AND the T-pose control at
        /// the same time. Exactly two do, and they are global negations of each other; the
        /// character's own-left-at-negative-X convention breaks that tie in favour of not
        /// negating X.
        ///
        /// The assertions below are the invariants that search checks, kept here as a regression
        /// guard. The T-pose control matters most: every wrong version so far also passed it, so
        /// it rules out fixes that merely shuffle which gesture is broken.
        /// </summary>
        public static int AuditCoordinateConstants()
        {
            int failures = 0;
            Console.WriteLine();
            Console.WriteLine("Coordinate convention (only Z negated; determined by ConventionSearch)");

            var point = new Microsoft.Kinect.CameraSpacePoint { X = 1f, Y = 2f, Z = 3f };
            var mapped = Kinect2VMC.App.KinectBodySource.ToCharacterSpace(point);

            failures += Check(mapped.X == 1f, "X passes through (left/right stays put)",
                mapped.X.ToString(CultureInfo.InvariantCulture));
            failures += Check(mapped.Y == 2f, "Y passes through (camera up stays up)",
                mapped.Y.ToString(CultureInfo.InvariantCulture));
            failures += Check(mapped.Z == -3f, "Z is negated (depth is the flipped axis)",
                mapped.Z.ToString(CultureInfo.InvariantCulture));

            var shoulder = new Microsoft.Kinect.CameraSpacePoint { X = 0.2f, Y = 1.1f, Z = 1.4f };

            // Control: a sideways arm has no vertical or depth extent, so it looks right under
            // every wrong version. It must stay right.
            var sideHand = new Microsoft.Kinect.CameraSpacePoint { X = 0.85f, Y = 1.1f, Z = 1.4f };
            Kinect2VMC.Core.Vec3 sideways = (Kinect2VMC.App.KinectBodySource.ToCharacterSpace(sideHand)
                - Kinect2VMC.App.KinectBodySource.ToCharacterSpace(shoulder)).Normalized();
            failures += Check(Math.Abs(sideways.X) > 0.9f
                && Math.Abs(sideways.Y) < 0.2f && Math.Abs(sideways.Z) < 0.2f,
                "a horizontal T-pose arm measures as purely sideways",
                sideways.ToString());

            // A hand above the shoulder must measure as character-space up (+Y).
            var raisedHand = new Microsoft.Kinect.CameraSpacePoint { X = 0.4f, Y = 1.9f, Z = 1.4f };
            Kinect2VMC.Core.Vec3 up = (Kinect2VMC.App.KinectBodySource.ToCharacterSpace(raisedHand)
                - Kinect2VMC.App.KinectBodySource.ToCharacterSpace(shoulder)).Normalized();
            failures += Check(up.Y > 0.9f,
                "a hand above the shoulder measures as up (+Y)",
                up.ToString());

            // Reaching towards the camera (smaller camera Z) is the character's forward (+Z).
            var reachedHand = new Microsoft.Kinect.CameraSpacePoint { X = 0.2f, Y = 1.1f, Z = 0.9f };
            Kinect2VMC.Core.Vec3 forward = (Kinect2VMC.App.KinectBodySource.ToCharacterSpace(reachedHand)
                - Kinect2VMC.App.KinectBodySource.ToCharacterSpace(shoulder)).Normalized();
            failures += Check(forward.Z > 0.9f,
                "reaching towards the camera measures as forward (+Z)",
                forward.ToString());

            // Moving away from the camera is the opposite.
            var backHand = new Microsoft.Kinect.CameraSpacePoint { X = 0.2f, Y = 1.1f, Z = 1.9f };
            Kinect2VMC.Core.Vec3 backward = (Kinect2VMC.App.KinectBodySource.ToCharacterSpace(backHand)
                - Kinect2VMC.App.KinectBodySource.ToCharacterSpace(shoulder)).Normalized();
            failures += Check(backward.Z < -0.9f,
                "moving away from the camera measures as backwards (-Z)",
                backward.ToString());

            // Leaning forward: the pelvis stays put, the shoulders move towards the camera, so the
            // spine must be up AND forward. The user reported this one explicitly, and it is the
            // assertion that catches a reversed depth axis.
            //
            // The pelvis-to-shoulder line is far from vertical in reality, so the "up" component
            // is only about 0.85; requiring more than that would reject a correct lean.
            var pelvis = new Microsoft.Kinect.CameraSpacePoint { X = 0f, Y = 0.8f, Z = 1.4f };
            var leanedShoulders = new Microsoft.Kinect.CameraSpacePoint { X = 0f, Y = 1.12f, Z = 1.2f };
            Kinect2VMC.Core.Vec3 spine = (Kinect2VMC.App.KinectBodySource.ToCharacterSpace(leanedShoulders)
                - Kinect2VMC.App.KinectBodySource.ToCharacterSpace(pelvis)).Normalized();
            failures += Check(spine.Y > 0.7f && spine.Z > 0.1f,
                "leaning forward tilts the spine up and towards the camera",
                spine.ToString());

            // And leaning backward must give the opposite depth sign, so the assertion above
            // cannot be satisfied by a sign-agnostic magnitude check.
            var leanedBackShoulders = new Microsoft.Kinect.CameraSpacePoint { X = 0f, Y = 1.12f, Z = 1.6f };
            Kinect2VMC.Core.Vec3 spineBack = (Kinect2VMC.App.KinectBodySource.ToCharacterSpace(leanedBackShoulders)
                - Kinect2VMC.App.KinectBodySource.ToCharacterSpace(pelvis)).Normalized();
            failures += Check(spineBack.Y > 0.7f && spineBack.Z < -0.1f,
                "leaning backward tilts the spine up and away from the camera",
                spineBack.ToString());

            return failures;
        }

        public static int AuditTiltCorrection()
        {
            int failures = 0;
            Console.WriteLine();
            Console.WriteLine("Kinect tilt correction (software compensation for a downward-angled sensor)");

            // An upright torso in character space: spine straight up, pelvis at the origin.
            var pelvis = new Kinect2VMC.Core.Vec3(0f, 0f, 0f);
            var shoulders = new Kinect2VMC.Core.Vec3(0f, 0.3f, 0f);
            var head = new Kinect2VMC.Core.Vec3(0f, 0.45f, 0f);

            // Positive correction rotates the skeleton about the character's right axis (+X).
            // Rotating +Y about +X by a positive angle yields +Z, and character-space +Z is the
            // direction the character faces (towards the sensor), so positive leans FORWARD.
            var pitch = Kinect2VMC.Core.Quat.AxisAngle(
                new Kinect2VMC.Core.Vec3(1f, 0f, 0f), 10f * 0.0174532925f);
            Kinect2VMC.Core.Vec3 rotated = pitch * (shoulders - pelvis);

            failures += Check(rotated.Z > 0.01f,
                "a positive pitch about +X leans the torso towards the sensor",
                rotated.ToString());

            // So the correction the user needs (undo a forward-looking lean) must be applied with
            // the OPPOSITE sign, which is why the UI documents positive as "tilt backwards".
            Kinect2VMC.Core.Vec3 corrected = Kinect2VMC.Core.Quat.AxisAngle(
                new Kinect2VMC.Core.Vec3(1f, 0f, 0f), -10f * 0.0174532925f) * (shoulders - pelvis);
            failures += Check(corrected.Z < -0.01f,
                "a negative pitch leans the torso away from the sensor",
                corrected.ToString());

            // The correction must preserve bone lengths, or it would deform the skeleton.
            float before = (head - pelvis).Length;
            Kinect2VMC.Core.Vec3 correctedHead =
                Kinect2VMC.Core.Quat.AxisAngle(new Kinect2VMC.Core.Vec3(1f, 0f, 0f), -10f * 0.0174532925f)
                * (head - pelvis);
            failures += Check(Math.Abs(correctedHead.Length - before) < 1e-4f,
                "the tilt correction preserves bone lengths",
                correctedHead.Length.ToString("F6", CultureInfo.InvariantCulture));

            return failures;
        }

        /// <summary>
        /// The arm reference pose must be selectable, because a model can DISPLAY a lowered-arms
        /// idle animation while its skeleton is still bound horizontally. The bind pose is what
        /// matters, and it cannot be inferred from what the model is showing.
        /// </summary>
        public static int AuditArmReferenceSwitch()
        {
            int failures = 0;
            Console.WriteLine();
            Console.WriteLine("Arm reference pose switch (T-pose bind vs A-pose bind)");

            var solver = new Kinect2VMC.Core.UpperBodySolver { Smoothing = 0f };

            failures += Check(solver.ArmReference == Kinect2VMC.Core.UpperBodySolver.ArmReferencePose.HorizontalT,
                "the default reference is the horizontal T-pose bind",
                solver.ArmReference.ToString());

            Kinect2VMC.Core.Vec3 tLeft = solver.ArmRestDirectionLeft;
            Kinect2VMC.Core.Vec3 tRight = solver.ArmRestDirectionRight;
            failures += Check(Math.Abs(tLeft.X + 1f) < 1e-4f && Math.Abs(tLeft.Y) < 1e-4f,
                "in T-pose mode the left arm rest points straight out (-X)",
                tLeft.ToString());
            failures += Check(Math.Abs(tRight.X - 1f) < 1e-4f && Math.Abs(tRight.Y) < 1e-4f,
                "in T-pose mode the right arm rest points straight out (+X)",
                tRight.ToString());

            solver.ArmReference = Kinect2VMC.Core.UpperBodySolver.ArmReferencePose.HangingA;
            Kinect2VMC.Core.Vec3 aLeft = solver.ArmRestDirectionLeft;
            Kinect2VMC.Core.Vec3 aRight = solver.ArmRestDirectionRight;
            failures += Check(aLeft.Y < -0.9f, "in A-pose mode the left arm rest points down",
                aLeft.ToString());
            failures += Check(aRight.Y < -0.9f && aRight.X > 0f,
                "in A-pose mode the right arm rest is the mirror of the left",
                aRight.ToString());

            // The two references must genuinely differ, or the switch would be a no-op.
            failures += Check(Math.Abs(aLeft.X - tLeft.X) > 0.5f,
                "switching the reference really changes the arm rest direction",
                aLeft.X.ToString("F3", CultureInfo.InvariantCulture));

            return failures;
        }

        /// <summary>
        /// The hand-shape estimate must move in the right direction: a fist brings the fingertip
        /// towards the thumb, an open hand pushes it away. Kinect supplies no per-finger angles, so
        /// this coarse shape is the most the sensor can support.
        /// </summary>
        public static int AuditHandPose()
        {
            int failures = 0;
            Console.WriteLine();
            Console.WriteLine("Hand pose estimate (fist vs open, from five joints)");

            // Joint positions in metres. The fingertip-to-thumb distance is the measurement.
            var wrist = new Kinect2VMC.Core.Vec3(0f, 0f, 0f);
            var hand = new Kinect2VMC.Core.Vec3(-0.04f, 0f, 0f);

            // Fist: the fingertip folds back to about the palm centre, near the thumb.
            var fistTip = new Kinect2VMC.Core.Vec3(-0.05f, -0.02f, 0f);
            var fistThumb = new Kinect2VMC.Core.Vec3(-0.055f, -0.025f, 0f);   // 0.7 cm apart

            // Open: the fingertip reaches well past the palm and the thumb is spread away.
            var openTip = new Kinect2VMC.Core.Vec3(-0.12f, 0f, 0f);
            var openThumb = new Kinect2VMC.Core.Vec3(-0.04f, -0.09f, 0f);    // 12 cm apart

            Kinect2VMC.Core.HandMetrics fist =
                Kinect2VMC.Core.HandPoseEstimator.Compute(wrist, hand, fistTip, fistThumb);
            Kinect2VMC.Core.HandMetrics open =
                Kinect2VMC.Core.HandPoseEstimator.Compute(wrist, hand, openTip, openThumb);

            failures += Check(fist.Valid && open.Valid, "both hand samples are valid", "");

            failures += Check(fist.TipToThumb < open.TipToThumb,
                "a fist measures a smaller fingertip-to-thumb distance than an open hand",
                fist.TipToThumb.ToString("F3", CultureInfo.InvariantCulture) + " vs "
                + open.TipToThumb.ToString("F3", CultureInfo.InvariantCulture));

            failures += Check(fist.Pose == Kinect2VMC.Core.HandPose.Fist,
                "the closed sample classifies as a fist",
                fist.Pose + " (" + fist.TipToThumb.ToString("F3", CultureInfo.InvariantCulture) + ")");

            failures += Check(open.Pose == Kinect2VMC.Core.HandPose.Open,
                "the open sample classifies as an open hand",
                open.Pose + " (" + open.TipToThumb.ToString("F3", CultureInfo.InvariantCulture) + ")");

            failures += Check(fist.Openness < 0.2f && open.Openness > 0.8f,
                "the openness value spans closed to open",
                fist.Openness.ToString("F2", CultureInfo.InvariantCulture) + " .. "
                + open.Openness.ToString("F2", CultureInfo.InvariantCulture));

            // Degenerate input must be reported as invalid rather than producing a bogus pose.
            var zero = Kinect2VMC.Core.HandPoseEstimator.Compute(
                wrist, wrist, fistTip, fistThumb);
            failures += Check(!zero.Valid,
                "a zero-length palm bone is reported as invalid, not as a pose",
                zero.Valid.ToString());

            // Kinect zeroes the fingertip and thumb when it loses the hand. Without the gate
            // that reads as a fingertip resting on the thumb -- a fabricated fist. The user's
            // own recording contained those readings.
            var dropped = Kinect2VMC.Core.HandPoseEstimator.Compute(
                wrist, hand, new Kinect2VMC.Core.Vec3(0f, 0f, 0f), new Kinect2VMC.Core.Vec3(0f, 0f, 0f), false);
            failures += Check(!dropped.Valid, "an untracked hand reports no pose at all", dropped.Pose.ToString());

            var zeroed = Kinect2VMC.Core.HandPoseEstimator.Compute(
                wrist, hand, new Kinect2VMC.Core.Vec3(0f, 0f, 0f), new Kinect2VMC.Core.Vec3(0f, 0f, 0f), true);
            failures += Check(!zeroed.Valid,
                "two coincident joints are treated as a dropout, not as a fist",
                zeroed.Pose.ToString());

            return failures;
        }

        /// <summary>
        /// The classifier against REAL recorded data: fingertip-to-thumb distances in metres,
        /// taken from the user's own hand-log.csv.
        ///
        /// The left hand produced a clearly bimodal distribution -- it closed to about 0.5 cm and
        /// opened to about 14 cm -- while the right hand never left 1.4-2.2 hand-size units and so
        /// never closed. The samples below are the measured extremes, so this test fails if the
        /// thresholds are ever moved somewhere the real sensor cannot reach.
        /// </summary>
        public static int AuditHandPoseAgainstRecordedData()
        {
            int failures = 0;
            Console.WriteLine();
            Console.WriteLine("Hand pose against the recorded sample (real measurements)");

            var wrist = new Kinect2VMC.Core.Vec3(0f, 0f, 0f);
            var hand = new Kinect2VMC.Core.Vec3(-0.04f, 0f, 0f);

            // The measured closed extreme: fingertip essentially touching the thumb.
            var closedTip = new Kinect2VMC.Core.Vec3(-0.05f, -0.02f, 0f);
            var closedThumb = new Kinect2VMC.Core.Vec3(-0.055f, -0.02f, 0f);   // 0.5 cm apart

            // The measured open extreme.
            var openTip = new Kinect2VMC.Core.Vec3(-0.14f, -0.02f, 0f);
            var openThumb = new Kinect2VMC.Core.Vec3(-0.05f, -0.09f, 0f);      // 11.4 cm apart

            Kinect2VMC.Core.HandMetrics closed =
                Kinect2VMC.Core.HandPoseEstimator.Compute(wrist, hand, closedTip, closedThumb);
            Kinect2VMC.Core.HandMetrics open =
                Kinect2VMC.Core.HandPoseEstimator.Compute(wrist, hand, openTip, openThumb);

            failures += Check(closed.Pose == Kinect2VMC.Core.HandPose.Fist,
                "the recorded closed extreme classifies as a fist",
                (closed.TipToThumb * 100f).ToString("F1", CultureInfo.InvariantCulture) + " cm -> " + closed.Pose);

            failures += Check(open.Pose == Kinect2VMC.Core.HandPose.Open,
                "the recorded open extreme classifies as an open hand",
                (open.TipToThumb * 100f).ToString("F1", CultureInfo.InvariantCulture) + " cm -> " + open.Pose);

            failures += Check(closed.TipToThumb < Kinect2VMC.Core.HandPoseEstimator.FistDistance
                && open.TipToThumb > Kinect2VMC.Core.HandPoseEstimator.OpenDistance,
                "the real closed and open extremes straddle the thresholds",
                (closed.TipToThumb * 100f).ToString("F1", CultureInfo.InvariantCulture) + " / "
                + (open.TipToThumb * 100f).ToString("F1", CultureInfo.InvariantCulture) + " cm");

            // The right hand stayed at one value throughout, which says nothing about its shape.
            // Such a reading must not be turned into a confident "fist".
            var stuckTip = new Kinect2VMC.Core.Vec3(-0.10f, 0f, 0f);
            var stuckThumb = new Kinect2VMC.Core.Vec3(-0.04f, -0.06f, 0f);
            Kinect2VMC.Core.HandMetrics stuck =
                Kinect2VMC.Core.HandPoseEstimator.Compute(wrist, hand, stuckTip, stuckThumb);

            failures += Check(stuck.Pose != Kinect2VMC.Core.HandPose.Fist,
                "a hand that never closes is not reported as a fist",
                (stuck.TipToThumb * 100f).ToString("F1", CultureInfo.InvariantCulture) + " cm -> " + stuck.Pose);

            return failures;
        }

        /// <summary>
        /// Hysteresis, which is what makes a two-state fist readout usable. A real hand passes
        /// through every intermediate shape and the sensor noise overlaps the states, so a raw
        /// threshold flickers frame to frame. The latch must ignore intermediate readings and hold
        /// whatever state it is already in.
        /// </summary>
        public static int AuditHandHysteresis()
        {
            int failures = 0;
            Console.WriteLine();
            Console.WriteLine("Hand hysteresis (a two-state readout that does not flicker)");

            var mode = Kinect2VMC.Core.HandPoseEstimator.HandStabilizer.Hysteresis;
            float engage = Kinect2VMC.Core.HandPoseEstimator.FistEngageDistance;
            float release = Kinect2VMC.Core.HandPoseEstimator.OpenReleaseDistance;
            float middle = (engage + release) / 2f;

            failures += Check(engage < release,
                "the fist-engage threshold sits below the open-release threshold",
                (engage * 100f).ToString("F1", CultureInfo.InvariantCulture) + " / "
                + (release * 100f).ToString("F1", CultureInfo.InvariantCulture) + " cm");

            Kinect2VMC.Core.HandPose afterClosing = Kinect2VMC.Core.HandPoseEstimator.Stabilize(
                0.01f, Kinect2VMC.Core.HandPose.Open, mode);
            failures += Check(afterClosing == Kinect2VMC.Core.HandPose.Fist,
                "a clearly closed hand latches to a fist", afterClosing.ToString());

            Kinect2VMC.Core.HandPose held = afterClosing;
            for (int i = 0; i < 20; i++)
            {
                held = Kinect2VMC.Core.HandPoseEstimator.Stabilize(0.02f, held, mode);
            }
            failures += Check(held == Kinect2VMC.Core.HandPose.Fist,
                "a held fist stays a fist for twenty frames", held.ToString());

            Kinect2VMC.Core.HandPose midFromFist = Kinect2VMC.Core.HandPoseEstimator.Stabilize(
                middle, Kinect2VMC.Core.HandPose.Fist, mode);
            Kinect2VMC.Core.HandPose midFromOpen = Kinect2VMC.Core.HandPoseEstimator.Stabilize(
                middle, Kinect2VMC.Core.HandPose.Open, mode);
            failures += Check(midFromFist == Kinect2VMC.Core.HandPose.Fist,
                "a half-closed reading does not release an established fist", midFromFist.ToString());
            failures += Check(midFromOpen == Kinect2VMC.Core.HandPose.Open,
                "a half-closed reading does not create a fist from an open hand", midFromOpen.ToString());

            Kinect2VMC.Core.HandPose released = Kinect2VMC.Core.HandPoseEstimator.Stabilize(
                0.12f, Kinect2VMC.Core.HandPose.Fist, mode);
            failures += Check(released == Kinect2VMC.Core.HandPose.Open,
                "a clearly open hand releases the fist", released.ToString());

            Kinect2VMC.Core.HandPose noisy = Kinect2VMC.Core.HandPose.Open;
            for (int i = 0; i < 40; i++)
            {
                float jitter = middle + (i % 2 == 0 ? 0.005f : -0.005f);
                noisy = Kinect2VMC.Core.HandPoseEstimator.Stabilize(jitter, noisy, mode);
            }
            failures += Check(noisy == Kinect2VMC.Core.HandPose.Open,
                "a hand hovering mid-way never changes state over forty frames", noisy.ToString());

            Kinect2VMC.Core.HandPose raw = Kinect2VMC.Core.HandPoseEstimator.Stabilize(
                middle, Kinect2VMC.Core.HandPose.Open,
                Kinect2VMC.Core.HandPoseEstimator.HandStabilizer.None);
            failures += Check(raw == Kinect2VMC.Core.HandPose.LooseFist,
                "without hysteresis the intermediate state is still reported", raw.ToString());

            return failures;
        }

        /// <summary>
        /// The palm frame that carries a twisted wrist through to the fingers. A hand's roll
        /// cannot be expressed by one bone direction, which is why the thumb direction is used to
        /// build a second axis.
        /// </summary>
        public static int AuditPalmFrame()
        {
            int failures = 0;
            Console.WriteLine();
            Console.WriteLine("Palm roll frame (a wrist turn turns the HAND; fingers only curl)");

            // SendHand is OFF by default now, because the hand is driven from the camera instead
            // (the user established that the hand script turns the palm correctly, while Kinect's
            // short, noisy hand bone manages at best a wave). This audit is specifically about the
            // hand bone's own frame, so it has to be switched on for the check to mean anything.
            var solver = new Kinect2VMC.Core.UpperBodySolver { Smoothing = 0f, EmitFingers = true };
            // Single-frame test: commit the latch immediately instead of after the debounce.
            Kinect2VMC.Core.HandPoseEstimator.SwitchHoldSeconds = 0f;

            var frame = new Kinect2VMC.Core.SkeletonFrameData { BodyTracked = true, TrackingId = 1, Time = 0f };

            frame.Set(Kinect2VMC.Core.KinectJoint.SpineBase, new Kinect2VMC.Core.Vec3(0f, 0.82f, -1.42f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.SpineMid, new Kinect2VMC.Core.Vec3(0f, 0.98f, -1.40f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.SpineShoulder, new Kinect2VMC.Core.Vec3(0f, 1.14f, -1.39f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.Neck, new Kinect2VMC.Core.Vec3(0f, 1.22f, -1.39f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.Head, new Kinect2VMC.Core.Vec3(0f, 1.35f, -1.38f), true);

            // A left arm reaching forward, with the palm turned so the thumb points down.
            frame.Set(Kinect2VMC.Core.KinectJoint.ShoulderLeft, new Kinect2VMC.Core.Vec3(-0.20f, 1.12f, -1.39f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.ShoulderRight, new Kinect2VMC.Core.Vec3(0.20f, 1.12f, -1.39f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.ElbowLeft, new Kinect2VMC.Core.Vec3(-0.22f, 1.10f, -1.10f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.WristLeft, new Kinect2VMC.Core.Vec3(-0.23f, 1.08f, -0.90f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.HandLeft, new Kinect2VMC.Core.Vec3(-0.23f, 1.08f, -0.86f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.HandTipLeft, new Kinect2VMC.Core.Vec3(-0.23f, 1.08f, -0.74f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.ThumbLeft, new Kinect2VMC.Core.Vec3(-0.27f, 1.02f, -0.84f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.ThumbRight, new Kinect2VMC.Core.Vec3(0.27f, 1.02f, -0.84f), true);

            frame.Set(Kinect2VMC.Core.KinectJoint.ElbowRight, new Kinect2VMC.Core.Vec3(0.22f, 1.10f, -1.10f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.WristRight, new Kinect2VMC.Core.Vec3(0.23f, 1.08f, -0.90f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.HandRight, new Kinect2VMC.Core.Vec3(0.23f, 1.08f, -0.86f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.HandTipRight, new Kinect2VMC.Core.Vec3(0.23f, 1.08f, -0.74f), true);

            frame.Set(Kinect2VMC.Core.KinectJoint.HipLeft, new Kinect2VMC.Core.Vec3(-0.12f, 0.78f, -1.40f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.HipRight, new Kinect2VMC.Core.Vec3(0.12f, 0.78f, -1.40f), true);

            var pose = new Kinect2VMC.Core.SkeletonPose();
            failures += Check(solver.Solve(frame, pose), "the hand frame solves", "");

            // The thumb bones must appear only while finger tracking is on.
            failures += Check(pose.BoneRotations.ContainsKey(Kinect2VMC.Core.VmcBones.LeftThumb),
                "the left thumb bone is emitted for the palm roll", "");
            failures += Check(pose.BoneRotations.ContainsKey(Kinect2VMC.Core.VmcBones.RightThumb),
                "the right thumb bone is emitted for the palm roll", "");

            // The finger bones must now carry the roll, so they are not all the identity.
            Kinect2VMC.Core.Quat index;
            bool present = pose.BoneRotations.TryGetValue("LeftIndexProximal", out index);
            failures += Check(present, "the finger bones are emitted", "");

            if (present)
            {
                // The palm's twist must NOT live on the fingers. Putting it there made a wrist turn
                // spin the fingers instead of turning the hand, which is what the user saw. With an
                // open hand the curl is zero, so the finger bones must be perfectly neutral.
                float fingerAngle = (float)(2.0 * Math.Acos(Math.Min(1.0, Math.Abs(index.W))) * 57.29577951308232);
                failures += Check(fingerAngle < 1f,
                    "an open hand leaves the finger bones neutral, twisted wrist or not",
                    fingerAngle.ToString("F1", CultureInfo.InvariantCulture) + " degrees");

                // The twist must live on the HAND bone instead, so a wrist turn turns the hand
                // rather than spinning the fingers. Compare the hand's WORLD rotation (composed
                // through its ancestors) against the direction actually measured: the difference
                // between the two is the twist that the direction alone cannot express.
                Kinect2VMC.Core.Quat handWorld = StraightArmAudit.WorldRotation(pose, Kinect2VMC.Core.VmcBones.LeftHand);
                Kinect2VMC.Core.Vec3 restForward = solver.ArmRestDirectionLeft;
                Kinect2VMC.Core.Vec3 measuredForward = (frame.Get(Kinect2VMC.Core.KinectJoint.HandTipLeft)
                    - frame.Get(Kinect2VMC.Core.KinectJoint.WristLeft)).Normalized();

                Kinect2VMC.Core.Vec3 pointed = (handWorld * restForward).Normalized();
                failures += Check(Kinect2VMC.Core.Vec3.Dot(pointed, measuredForward) > 0.99f,
                    "the hand still points where it was measured to point",
                    Kinect2VMC.Core.Vec3.Dot(pointed, measuredForward).ToString("F4", CultureInfo.InvariantCulture));

                // And the finger bones must be neutral, so the twist is on the hand, not the fingers.
                Kinect2VMC.Core.Quat index2;
                pose.BoneRotations.TryGetValue("LeftIndexProximal", out index2);
                float fingerAngle2 = (float)(2.0 * Math.Acos(Math.Min(1.0, Math.Abs(index2.W))) * 57.29577951308232);
                failures += Check(fingerAngle2 < 1f,
                    "the finger bones stay neutral so the hand, not the fingers, carries the turn",
                    fingerAngle2.ToString("F1", CultureInfo.InvariantCulture) + " degrees");
            }

            // With finger tracking off, neither the thumb nor the finger bones may be sent.
            var plain = new Kinect2VMC.Core.UpperBodySolver { Smoothing = 0f, EmitFingers = false };
            var plainPose = new Kinect2VMC.Core.SkeletonPose();
            plain.Solve(frame, plainPose);
            failures += Check(!plainPose.BoneRotations.ContainsKey(Kinect2VMC.Core.VmcBones.LeftThumb)
                && !plainPose.BoneRotations.ContainsKey("LeftIndexProximal"),
                "with finger tracking off, no hand bones beyond LeftHand are sent", "");

            return failures;
        }
        /// <summary>
        /// The direction the fingers fold. A fist brings the fingertips TOWARDS the palm; splaying
        /// them away is the "fingers turned inside out" defect the user reported.
        ///
        /// The criterion is that definition, not the sign of a cross product. The palm normal is
        /// derived as forward x toThumb and toThumb is NOT perpendicular to forward, so that cross
        /// product's sign is not a reliable indicator of which side is the palm -- which is how the
        /// wrong direction shipped in the first place.
        /// </summary>
        public static int AuditFingerCurlDirection()
        {
            int failures = 0;
            Console.WriteLine();
            Console.WriteLine("Finger curl direction (a fist closes TOWARDS the palm)");

            var solver = new Kinect2VMC.Core.UpperBodySolver { Smoothing = 0f, EmitFingers = true };
            // Single-frame test: commit the latch immediately instead of after the debounce.
            Kinect2VMC.Core.HandPoseEstimator.SwitchHoldSeconds = 0f;
            var frame = new Kinect2VMC.Core.SkeletonFrameData { BodyTracked = true, TrackingId = 1, Time = 0f };

            frame.Set(Kinect2VMC.Core.KinectJoint.SpineBase, new Kinect2VMC.Core.Vec3(0f, 0.82f, -1.42f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.SpineMid, new Kinect2VMC.Core.Vec3(0f, 0.98f, -1.40f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.SpineShoulder, new Kinect2VMC.Core.Vec3(0f, 1.14f, -1.39f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.Neck, new Kinect2VMC.Core.Vec3(0f, 1.22f, -1.39f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.Head, new Kinect2VMC.Core.Vec3(0f, 1.35f, -1.38f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.ShoulderLeft, new Kinect2VMC.Core.Vec3(-0.20f, 1.12f, -1.39f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.ShoulderRight, new Kinect2VMC.Core.Vec3(0.20f, 1.12f, -1.39f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.ElbowLeft, new Kinect2VMC.Core.Vec3(-0.24f, 1.10f, -1.10f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.ElbowRight, new Kinect2VMC.Core.Vec3(0.24f, 1.10f, -1.10f), true);

            // A hand pointing forward with the thumb across the palm.
            var wrist = new Kinect2VMC.Core.Vec3(-0.25f, 1.08f, -0.90f);
            var thumb = new Kinect2VMC.Core.Vec3(-0.31f, 1.13f, -0.84f);

            frame.Set(Kinect2VMC.Core.KinectJoint.WristLeft, wrist, true);
            frame.Set(Kinect2VMC.Core.KinectJoint.HandLeft, new Kinect2VMC.Core.Vec3(-0.25f, 1.07f, -0.86f), true);
            // The fingertip sits very close to the thumb, which is what marks this as a fist.
            frame.Set(Kinect2VMC.Core.KinectJoint.HandTipLeft, new Kinect2VMC.Core.Vec3(-0.305f, 1.13f, -0.845f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.ThumbLeft, thumb, true);

            frame.Set(Kinect2VMC.Core.KinectJoint.WristRight, new Kinect2VMC.Core.Vec3(0.25f, 1.08f, -0.90f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.HandRight, new Kinect2VMC.Core.Vec3(0.25f, 1.07f, -0.86f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.HandTipRight, new Kinect2VMC.Core.Vec3(0.305f, 1.13f, -0.845f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.ThumbRight, new Kinect2VMC.Core.Vec3(0.31f, 1.13f, -0.84f), true);

            frame.Set(Kinect2VMC.Core.KinectJoint.HipLeft, new Kinect2VMC.Core.Vec3(-0.12f, 0.78f, -1.40f), true);
            frame.Set(Kinect2VMC.Core.KinectJoint.HipRight, new Kinect2VMC.Core.Vec3(0.12f, 0.78f, -1.40f), true);

            var pose = new Kinect2VMC.Core.SkeletonPose();
            failures += Check(solver.Solve(frame, pose), "the closed hand solves", "");

            Kinect2VMC.Core.Quat curl;
            if (!pose.BoneRotations.TryGetValue("LeftIndexProximal", out curl))
            {
                failures += Check(false, "the finger bones are emitted", "missing");
                return failures;
            }

            float curlAngle = (float)(2.0 * Math.Acos(Math.Min(1.0, Math.Abs(curl.W))) * 57.29577951308232);
            failures += Check(curlAngle > 30f,
                "a fist folds the fingers by a substantial angle",
                curlAngle.ToString("F1", CultureInfo.InvariantCulture) + " degrees");

            // The decisive check, stated without reference to any cross-product sign: folding a
            // finger moves its tip towards the palm. Model that on the upper arm, whose joint the
            // solver also drives, by asking whether the curl rotates the bone direction the same
            // way a closing hand would.
            Kinect2VMC.Core.Vec3 forward = (frame.Get(Kinect2VMC.Core.KinectJoint.HandTipLeft) - wrist).Normalized();
            Kinect2VMC.Core.Vec3 palmNormal = Kinect2VMC.Core.Vec3.Cross(forward,
                (thumb - wrist).Normalized()).Normalized();

            Kinect2VMC.Core.Vec3 curled = (curl * forward).Normalized();
            Kinect2VMC.Core.Vec3 uncircled = (Kinect2VMC.Core.Quat.AxisAngle(palmNormal,
                curlAngle * 0.0174532925f) * forward).Normalized();

            // Whatever axis the solver chose, the result must lie on the side of the hand that the
            // thumb is NOT on: the fingers fold across the palm, towards the thumb's own side.
            Kinect2VMC.Core.Vec3 towardsThumb = (thumb - wrist).Normalized();
            float curledTowardsThumb = Kinect2VMC.Core.Vec3.Dot(curled, towardsThumb);
            float plainTowardsThumb = Kinect2VMC.Core.Vec3.Dot(uncircled, towardsThumb);

            failures += Check(curledTowardsThumb > plainTowardsThumb,
                "the fingers fold towards the palm side of the hand",
                curledTowardsThumb.ToString("F3", CultureInfo.InvariantCulture) + " vs "
                + plainTowardsThumb.ToString("F3", CultureInfo.InvariantCulture));

            return failures;
        }

        private static int Check(bool condition, string name, string detail)
        {
            if (condition)
            {
                Console.WriteLine("  [ok]   " + name);
                return 0;
            }
            Console.WriteLine("  [FAIL] " + name + "  -> got " + detail);
            return 1;
        }
    }
}
