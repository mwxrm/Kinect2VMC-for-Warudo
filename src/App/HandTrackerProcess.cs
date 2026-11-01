using System;
using System.Diagnostics;
using System.IO;
using System.Text;

namespace Kinect2VMC.App
{
    /// <summary>
    /// Launches and supervises the Python hand tracker as a child process.
    ///
    /// Why a child process rather than a library
    /// -----------------------------------------
    /// The hand tracker needs MediaPipe, whose Python wheel carries its own native runtime. Hosting
    /// that inside this .NET Framework 4.8 process would mean matching a second runtime's threading
    /// and lifetime rules to WinForms, for no benefit: this program only needs to start it, point it
    /// at the right VMC port, and stop it again.
    ///
    /// Two things this deliberately gets right, because both cost real debugging time:
    ///
    ///   * The interpreter is FOUND, not assumed. `python3.exe` in the per-user WindowsApps folder
    ///     is a Microsoft Store stub that exits without doing anything, so "python is on PATH" is
    ///     not enough to conclude that Python is installed.
    ///   * The script is located relative to THIS executable, so the dist folder keeps working when
    ///     it is copied elsewhere.
    /// </summary>
    public sealed class HandTrackerProcess : IDisposable
    {
        private Process _process;

        /// <summary>True while the child process is alive.</summary>
        public bool IsRunning
        {
            get
            {
                try { return _process != null && !_process.HasExited; }
                catch { return false; }
            }
        }

        /// <summary>
        /// Default UDP port the tracker sends palm orientations back on.
        ///
        /// Declared here so the number the launcher passes and the number the receiver
        /// listens on cannot drift apart.
        /// </summary>
        public const int DefaultFramePort = 39542;

        /// <summary>Where the last launch found things, for the UI to report.</summary>
        public string LastMessage { get; private set; }

        /// <summary>
        /// Looks for a usable Python interpreter.
        ///
        /// Order matters. A directory that also contains a WindowsApps stub is checked for the real
        /// file, and only paths that actually exist on disk are returned -- running the stub would
        /// look like success while doing nothing at all.
        /// </summary>
        public static string FindPython()
        {
            // An explicit setting wins, so a user with an unusual install can point at it.
            string configured = Environment.GetEnvironmentVariable("KINECT2VMC_PYTHON");
            if (!string.IsNullOrEmpty(configured) && File.Exists(configured)) { return configured; }

            var candidates = new System.Collections.Generic.List<string>();

            // Common per-user and per-machine install roots, newest first.
            string localAppData = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData);
            string programFiles = Environment.GetFolderPath(Environment.SpecialFolder.ProgramFiles);

            AddVersioned(candidates, Path.Combine(localAppData, "Programs", "Python"));
            AddVersioned(candidates, programFiles);

            // Then whatever PATH offers, skipping the Store stub's folder.
            string path = Environment.GetEnvironmentVariable("PATH") ?? string.Empty;
            foreach (string dir in path.Split(';'))
            {
                if (dir.Length == 0) { continue; }
                if (dir.IndexOf("WindowsApps", StringComparison.OrdinalIgnoreCase) >= 0) { continue; }
                candidates.Add(Path.Combine(dir, "python.exe"));
            }

            foreach (string candidate in candidates)
            {
                if (!string.IsNullOrEmpty(candidate) && File.Exists(candidate)) { return candidate; }
            }

            return null;
        }

        private static void AddVersioned(System.Collections.Generic.List<string> into, string root)
        {
            try
            {
                if (!Directory.Exists(root)) { return; }

                var found = new System.Collections.Generic.List<string>();
                foreach (string dir in Directory.GetDirectories(root, "python*"))
                {
                    string exe = Path.Combine(dir, "python.exe");
                    if (File.Exists(exe)) { found.Add(exe); }
                }

                // Highest version first, so a machine with several installs uses the newest.
                found.Sort();
                found.Reverse();
                into.AddRange(found);
            }
            catch
            {
                // A search path that cannot be read is simply skipped.
            }
        }

        /// <summary>Finds hand_tracker.py beside this executable, or in a tools subfolder.</summary>
        public static string FindScript()
        {
            string here = Path.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().Location);
            string[] probes =
            {
                Path.Combine(here, "hand_tracker.py"),
                Path.Combine(here, "tools", "hand_tracker.py"),
                Path.Combine(here, "..", "tools", "hand_tracker.py"),
            };

            foreach (string probe in probes)
            {
                string full = Path.GetFullPath(probe);
                if (File.Exists(full)) { return full; }
            }

            return null;
        }

        /// <summary>
        /// Starts the tracker. Returns false and sets LastMessage when it cannot.
        /// </summary>
        /// <param name="vmcPort">Port this program is sending VMC to; the script sends there too.</param>
        /// <param name="camera">Camera index.</param>
        /// <param name="preview">shown, black or hidden.</param>
        /// <param name="posesFile">Pose data the finger blend reads.</param>
        public bool Start(int vmcPort, int camera, string preview, string posesFile,
            int relayPort = 0)
        {
            if (IsRunning) { return true; }

            string python = FindPython();
            if (python == null)
            {
                LastMessage = "找不到 Python。请安装 Python 3，或设置环境变量 KINECT2VMC_PYTHON 指向 python.exe。";
                return false;
            }

            string script = FindScript();
            if (script == null)
            {
                LastMessage = "找不到 hand_tracker.py。请把它放在本程序旁边，或放到 tools 子文件夹。";
                return false;
            }

            string scriptDir = Path.GetDirectoryName(script);
            string poses = posesFile;
            if (string.IsNullOrEmpty(poses)) { poses = Path.Combine(scriptDir, "poses.json"); }

            var arguments = new StringBuilder();
            arguments.Append("-u \"").Append(script).Append("\"");
            arguments.Append(" --port ").Append(vmcPort.ToString(System.Globalization.CultureInfo.InvariantCulture));
            arguments.Append(" --preview ").Append(preview);

            if (relayPort > 0)
            {
                // Images come from the Kinect through this program, rather than from a webcam the
                // user may not have or may already be using for face capture.
                arguments.Append(" --relay ").Append(relayPort.ToString(
                    System.Globalization.CultureInfo.InvariantCulture));

                // Kinect's colour image is NOT mirrored, so the frame behaves like a photograph of
                // the user. Saying so is what keeps the hands on the correct sides: the "leftmost
                // detection is the user's right hand" rule only holds for a mirrored webcam image,
                // and applying it to a Kinect frame swapped the two hands.
                arguments.Append(" --no-mirror");

                // Where the tracker sends the measured palm orientation back to. The receiver in
                // this program listens on the same number, so the two always agree.
                arguments.Append(" --frame-port ").Append(DefaultFramePort.ToString(
                    System.Globalization.CultureInfo.InvariantCulture));
            }
            else
            {
                arguments.Append(" --camera ").Append(camera.ToString(
                    System.Globalization.CultureInfo.InvariantCulture));
            }
            if (File.Exists(poses))
            {
                arguments.Append(" --poses \"").Append(poses).Append("\"");
            }

            var info = new ProcessStartInfo
            {
                FileName = python,
                Arguments = arguments.ToString(),
                WorkingDirectory = scriptDir,
                UseShellExecute = false,
                // No console window of its own: the preview window is the only thing this should
                // ever put on screen, and the parent's console would flash otherwise.
                CreateNoWindow = true,
            };

            try
            {
                _process = Process.Start(info);
                if (_process == null)
                {
                    LastMessage = "启动 " + python + " 失败。";
                    return false;
                }

                LastMessage = "已启动手部识别（" + Path.GetFileName(python) + "，" +
                    (preview == "hidden" ? "无预览窗口" : preview == "black" ? "纯黑窗口" : "显示预览") + "）";
                return true;
            }
            catch (Exception ex)
            {
                LastMessage = "启动手部识别失败: " + ex.Message;
                _process = null;
                return false;
            }
        }

        /// <summary>Asks the child to stop. Safe to call when nothing is running.</summary>
        public void Stop()
        {
            try
            {
                if (_process != null && !_process.HasExited)
                {
                    _process.Kill();
                    _process.WaitForExit(2000);
                }
            }
            catch
            {
                // Already gone, or not ours to kill. Nothing useful to do either way.
            }
            finally
            {
                LastMessage = "手部识别已停止";
            }
        }

        public void Dispose()
        {
            Stop();
            if (_process != null) { _process.Dispose(); _process = null; }
        }
    }
}
