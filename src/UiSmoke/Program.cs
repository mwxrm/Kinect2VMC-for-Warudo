using System;
using System.Reflection;
using System.Text;
using System.Threading;
using System.Windows.Forms;
using Kinect2VMC.App;
using Kinect2VMC.Core;
using Kinect2VMC.SelfTest;

namespace Kinect2VMC.UiSmoke
{
    /// <summary>
    /// Starts the real main window and lets its message loop run for a moment, then closes it.
    ///
    /// This catches the class of failure a compile cannot: a control that throws while being
    /// created, a layout that loops forever, a settings file that cannot be parsed, or a
    /// shutdown path that deadlocks. It deliberately does NOT start tracking, so no Kinect
    /// device is required and the pipeline stays idle.
    /// </summary>
    internal static class Program
    {
        private static int _failed;

        [STAThread]
        private static int Main(string[] args)
        {
            Console.OutputEncoding = Encoding.UTF8;

            // A developer convenience: print a diagnostic report without opening the window.
            if (args != null && Array.IndexOf(args, "--report") >= 0)
            {
                return PrintSampleReport();
            }

            Console.WriteLine("Kinect2VMC UI smoke test");
            Console.WriteLine("========================");

            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);

            // 1. Settings round trip, including a malformed file.
            TestSettingsRoundTrip();

            // 2. Confirm every Kinect SDK member the adapter calls actually exists, with the
            //    signature the code assumes. This is the only dependency that cannot be
            //    exercised without the device, so a typo would otherwise reach the user.
            int sdkFailures = SdkSurfaceAudit.Run();
            sdkFailures += SdkSurfaceAudit.AuditCoordinateConstants();

            sdkFailures += SdkSurfaceAudit.AuditTiltCorrection();
            sdkFailures += SdkSurfaceAudit.AuditArmReferenceSwitch();
            sdkFailures += SdkSurfaceAudit.AuditHandPose();
            sdkFailures += SdkSurfaceAudit.AuditHandPoseAgainstRecordedData();
            sdkFailures += SdkSurfaceAudit.AuditHandHysteresis();
            sdkFailures += SdkSurfaceAudit.AuditPalmFrame();
            sdkFailures += SdkSurfaceAudit.AuditFingerCurlDirection();
            if (sdkFailures > 0) { _failed += sdkFailures; }

            // 3. Construct the real window and pump its message loop briefly.
            TestMainFormConstructsAndCloses();

            // 4. Drive the real pipeline with synthetic frames and inspect the VMC it emits.
            //    This is the end-to-end coverage a device-free environment can still provide.
            int pipelineFailures = PipelineHarness.Run();
            if (pipelineFailures > 0) { _failed += pipelineFailures; }

            // 5. Build the window once more with a pipeline that has already been exercised, to
            //    make sure nothing in the pipeline leaves the UI in a bad state.
            TestMainFormAfterPipelineUse();

            // 6. The diagnostic report is the artefact the user sends back when the first real
            //    run misbehaves, so every section must actually be produced.
            TestDiagnosticsReport();

            // 7. Quantify the solver's one known approximation: the forearm's roll comes from the
            //    body's up axis rather than from the plane the elbow bends in. The result decides
            //    what the documentation must admit.
            ConventionSearch.Run();
            ReferencePoseStudy.Run();
            int straightFailures = StraightArmAudit.Run();
            if (straightFailures > 0) { _failed += straightFailures; }

            int elbowFailures = ElbowTwistAudit.Run();
            if (elbowFailures > 0) { _failed += elbowFailures; }

            // 8. Record why the obvious "fix" for that approximation is not applied: switching the
            //    reference axis costs more roll than the ill-conditioning it would cure.
            ReferenceSwitchStudy.Run();

            Console.WriteLine();
            if (_failed > 0)
            {
                Console.WriteLine("UI SMOKE TEST FAILED: " + _failed + " problem(s)");
                return 1;
            }
            Console.WriteLine("UI SMOKE TEST PASSED");
            return 0;
        }

        private static void Check(bool condition, string name, string detail = null)
        {
            if (condition)
            {
                Console.WriteLine("  [ok]   " + name);
            }
            else
            {
                _failed++;
                Console.WriteLine("  [FAIL] " + name + (detail == null ? "" : "  -> " + detail));
            }
        }

        private static void TestSettingsRoundTrip()
        {
            Console.WriteLine();
            Console.WriteLine("Settings");

            var original = new AppSettings
            {
                TargetAddress = "127.0.0.1",
                TargetPort = 40000,
                Smoothing = 0.62f,
                SendHead = true,
                SendRootPosition = false,
                MovementScale = 1.4f,
                MovementLimit = 0.35f,
                Mirror = true
            };

            var loaded = new AppSettings();
            loaded.LoadFromKeyValueText(original.ToKeyValueText());

            Check(loaded.TargetAddress == original.TargetAddress, "address round trips", loaded.TargetAddress);
            Check(loaded.TargetPort == original.TargetPort, "port round trips", loaded.TargetPort.ToString());
            Check(Math.Abs(loaded.Smoothing - original.Smoothing) < 1e-6f, "smoothing round trips");
            Check(loaded.SendHead == original.SendHead, "head flag round trips");
            Check(loaded.SendRootPosition == original.SendRootPosition, "root flag round trips");
            Check(Math.Abs(loaded.MovementScale - original.MovementScale) < 1e-6f, "movement scale round trips");
            Check(Math.Abs(loaded.MovementLimit - original.MovementLimit) < 1e-6f, "movement limit round trips");
            Check(loaded.Mirror == original.Mirror, "mirror flag round trips");

            // Malformed input must not throw and must not corrupt the defaults into nonsense.
            var malformed = new AppSettings();
            malformed.LoadFromKeyValueText(
                "not a pair\r\nTargetPort=abc\r\nSmoothing=-5\r\nMovementLimit=999\r\nTargetAddress=\r\n\r\n# comment\r\n");
            Check(malformed.TargetPort == 39539, "bad port falls back to 39539", malformed.TargetPort.ToString());
            Check(malformed.Smoothing >= 0f, "negative smoothing is clamped", malformed.Smoothing.ToString("F2"));
            Check(malformed.MovementLimit <= 3f, "absurd movement limit is clamped",
                malformed.MovementLimit.ToString("F2"));
            Check(malformed.TargetAddress.Length > 0, "empty address falls back to a usable one");

            // A hand-edited file with spaces and "true"/"false" text should still be read.
            var friendly = new AppSettings();
            friendly.LoadFromKeyValueText("SendHead = true\r\nMirror = false\r\n");
            Check(friendly.SendHead, "text 'true' is accepted for a flag");
            Check(!friendly.Mirror, "text 'false' is accepted for a flag");

            // The documented defaults must match the code. The manual tells the user what to
            // expect on a fresh install, so a silent change here would make the documentation lie.
            var fresh = new AppSettings();
            Check(fresh.TargetAddress == "127.0.0.1", "documented default address", fresh.TargetAddress);
            Check(fresh.TargetPort == 39539, "documented default port", fresh.TargetPort.ToString());
            Check(Math.Abs(fresh.Smoothing - 0.45f) < 1e-6f, "documented default smoothing",
                fresh.Smoothing.ToString("F2"));
            Check(!fresh.SendHead, "the head is off by default (VSeeFace owns it)");
            Check(fresh.SendRootPosition, "root position is on by default");
            Check(Math.Abs(fresh.MovementLimit - 0.5f) < 1e-6f, "documented default movement limit",
                fresh.MovementLimit.ToString("F2"));
            Check(!fresh.Mirror, "mirroring is off by default");

            // And the default file must be parseable by the same reader that loads it.
            var reparsed = new AppSettings();
            reparsed.LoadFromKeyValueText(fresh.ToKeyValueText());
            Check(reparsed.TargetPort == fresh.TargetPort
                && Math.Abs(reparsed.Smoothing - fresh.Smoothing) < 1e-6f
                && reparsed.SendHead == fresh.SendHead
                && reparsed.SendRootPosition == fresh.SendRootPosition
                && reparsed.Mirror == fresh.Mirror,
                "the default settings file round trips through the real loader");
        }

        private static void TestMainFormConstructsAndCloses()
        {
            Console.WriteLine();
            Console.WriteLine("Main window");

            MainForm form = null;
            Exception constructionError = null;

            try
            {
                form = new MainForm();
            }
            catch (Exception ex)
            {
                constructionError = ex;
            }

            Check(constructionError == null, "MainForm constructs without throwing",
                constructionError == null ? null : constructionError.GetType().Name + ": " + constructionError.Message);

            if (form == null) { return; }

            Check(form.Controls.Count > 0, "the window has content");
            Check(form.Text.Length > 0, "the window has a caption", form.Text);

            // Count the interactive controls, which also proves the layout code ran to the end.
            int buttons = CountControls(form, typeof(Button));
            int labels = CountControls(form, typeof(Label));
            int checkBoxes = CountControls(form, typeof(CheckBox));
            int numeric = CountControls(form, typeof(NumericUpDown));
            int trackBars = CountControls(form, typeof(TrackBar));

            Check(buttons >= 4, "expected at least four buttons", buttons.ToString());
            Check(labels >= 8, "expected the status labels to exist", labels.ToString());
            Check(checkBoxes >= 3, "expected the three option check boxes", checkBoxes.ToString());
            Check(numeric >= 1, "expected the numeric inputs", numeric.ToString());
            Check(trackBars >= 1, "expected the smoothing slider", trackBars.ToString());

            // Read the built-in timer field to confirm the refresh loop was wired up.
            FieldInfo timerField = typeof(MainForm).GetField("_refreshTimer",
                BindingFlags.NonPublic | BindingFlags.Instance);
            Check(timerField != null, "refresh timer field is present");

            Exception runError = null;
            var closer = new System.Windows.Forms.Timer { Interval = 700 };
            closer.Tick += (s, e) =>
            {
                closer.Stop();
                try { form.Close(); }
                catch (Exception ex) { runError = ex; }
            };
            closer.Start();

            try
            {
                // Pumping the real message loop exercises the timer callback that reads the
                // pipeline state and repaints the preview.
                Application.Run(form);
            }
            catch (Exception ex)
            {
                runError = ex;
            }
            finally
            {
                closer.Dispose();
            }

            Check(runError == null, "the message loop runs and the window closes cleanly",
                runError == null ? null : runError.GetType().Name + ": " + runError.Message);
            Check(form.IsDisposed, "the window was disposed on close");

            if (timerField != null)
            {
                var timer = timerField.GetValue(form) as System.Windows.Forms.Timer;
                Check(timer == null || !timer.Enabled, "the refresh timer was stopped on close");
            }
        }

        private static void TestMainFormAfterPipelineUse()
        {
            Console.WriteLine();
            Console.WriteLine("Main window after pipeline use");

            // A settings file may now exist from an earlier run; the window must load it without
            // complaint and still construct.
            Exception error = null;
            MainForm form = null;
            try
            {
                form = new MainForm();
            }
            catch (Exception ex)
            {
                error = ex;
            }

            Report(error == null, "MainForm still constructs with an existing settings file",
                error == null ? null : error.GetType().Name + ": " + error.Message);

            if (form == null) { return; }

            // The refresh callback must survive an idle pipeline, which is the state a user sees
            // immediately after launching.
            var closer = new System.Windows.Forms.Timer { Interval = 500 };
            closer.Tick += (s, e) =>
            {
                closer.Stop();
                form.Close();
            };
            closer.Start();

            Exception runError = null;
            try
            {
                Application.Run(form);
            }
            catch (Exception ex)
            {
                runError = ex;
            }
            finally
            {
                closer.Dispose();
            }

            // MainForm persists settings beside the running executable as a normal side effect;
            // remove it so the build output stays free of test litter.
            try
            {
                string litter = System.IO.Path.Combine(
                    System.IO.Path.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().Location),
                    "settings.txt");
                if (System.IO.File.Exists(litter)) { System.IO.File.Delete(litter); }
            }
            catch { /* tidiness only */ }
            Report(runError == null, "the window survives a refresh cycle with an idle pipeline",
                runError == null ? null : runError.GetType().Name + ": " + runError.Message);
        }

        /// <summary>
        /// Prints a diagnostic report to the console without a window or a Kinect, so the
        /// report's contents can be reviewed during development.
        /// </summary>
        private static int PrintSampleReport()
        {
            var settings = new AppSettings();
            var source = new FakeBodySource { HoldFrame = TestSkeleton.RestPose() };

            using (var pipeline = new TrackingPipeline(settings, source))
            {
                pipeline.Tick();

                string text;
                string path = DiagnosticsReport.Write(settings, pipeline, out text);
                Console.WriteLine(text);
                Console.WriteLine("(also written to " + path + ")");

                try { System.IO.File.Delete(path); } catch { }
            }

            return 0;
        }

        /// <summary>
        /// The diagnostic report is what the user sends back when the first real run misbehaves,
        /// so every section must actually be produced and must stay free of personal data.
        /// </summary>
        private static void TestDiagnosticsReport()
        {
            Console.WriteLine();
            Console.WriteLine("Diagnostic report");

            var settings = new AppSettings
            {
                TargetAddress = "127.0.0.1",
                TargetPort = 39539,
                Smoothing = 0.45f,
                SendHead = false,
                SendRootPosition = true,
                MovementLimit = 0.5f,
                Mirror = false
            };

            var source = new FakeBodySource { HoldFrame = TestSkeleton.RestPose() };
            using (var pipeline = new TrackingPipeline(settings, source))
            {
                pipeline.Tick();

                string text = null;
                string path = null;
                Exception error = null;

                try
                {
                    path = DiagnosticsReport.Write(settings, pipeline, out text);
                }
                catch (Exception ex)
                {
                    error = ex;
                }

                Report(error == null, "the report is generated without throwing",
                    error == null ? null : error.GetType().Name + ": " + error.Message);

                if (text == null) { return; }

                Report(text.Length > 800, "the report has substantial content",
                    text.Length + " characters");

                // Every section the user is told to look at must be present.
                string[] required =
                {
                    "[1] 运行环境",
                    "[2] Kinect SDK 程序集",
                    "[3] USB 设备节点",
                    "[4] Kinect 运行时服务与管道",
                    "[5] 传感器状态",
                    "[6] 输出配置",
                    "[7] 运行统计",
                    "[8] 请把这个文件发回给开发者"
                };
                foreach (string section in required)
                {
                    Report(text.Contains(section), "report contains section '" + section + "'");
                }

                // The values the developer needs to reproduce the setup.
                Report(text.Contains("VID_045E&PID_02C4"), "report names the expected hardware id");
                Report(text.Contains("KinectMonitor"), "report records the service state");
                Report(text.Contains("39539"), "report records the VMC port");
                Report(text.Contains("11"), "report records what the bone count should be");

                // The report must not leak anything personal: no user name, no machine name.
                string user = Environment.UserName;
                bool leakedUser = !string.IsNullOrEmpty(user) && text.Contains(user);
                Report(!leakedUser, "report does not contain the account name",
                    leakedUser ? "found '" + user + "'" : null);

                if (path != null)
                {
                    Report(System.IO.File.Exists(path), "the report file exists on disk", path);
                    Report(path.EndsWith(".txt", StringComparison.OrdinalIgnoreCase),
                        "the report is written as a text file", path);

                    // Leave no litter behind in the build output.
                    try { System.IO.File.Delete(path); } catch { }
                }
            }
        }

        /// <summary>Records one smoke-test result and keeps the failure count.</summary>
        private static void Report(bool condition, string name, string detail = null)
        {
            if (condition)
            {
                Console.WriteLine("  [ok]   " + name);
            }
            else
            {
                _failed++;
                Console.WriteLine("  [FAIL] " + name + (detail == null ? "" : "  -> " + detail));
            }
        }

        private static int CountControls(Control root, Type type)
        {
            int count = 0;
            foreach (Control child in root.Controls)
            {
                if (type.IsInstanceOfType(child)) { count++; }
                count += CountControls(child, type);
            }
            return count;
        }
    }
}
