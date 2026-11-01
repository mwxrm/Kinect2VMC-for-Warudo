using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.ServiceProcess;
using System.Text;
using System.Windows.Forms;
using Kinect2VMC.Core;
using Microsoft.Win32;

namespace Kinect2VMC.App
{
    /// <summary>
    /// Produces a plain-text diagnostic report describing everything needed to tell why the
    /// program is not working.
    ///
    /// This exists because the Kinect can only be opened from an ordinary desktop session, so
    /// the first real run always happens on the user's machine. Without a report, a failure
    /// like "it says Kinect is not connected" is unactionable; with one, the device node, the
    /// service state, the runtime pipes and the SDK location are all visible at once.
    /// </summary>
    public static class DiagnosticsReport
    {
        /// <summary>
        /// Builds the report text and writes it next to the executable.
        /// Returns the full path of the file that was written.
        /// </summary>
        public static string Write(AppSettings settings, TrackingPipeline pipeline, out string text)
        {
            text = Build(settings, pipeline);

            string directory = Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location);
            if (string.IsNullOrEmpty(directory)) { directory = "."; }

            string path = Path.Combine(directory,
                "诊断报告-" + DateTime.Now.ToString("yyyyMMdd-HHmmss", CultureInfo.InvariantCulture) + ".txt");

            File.WriteAllText(path, text, new UTF8Encoding(true));
            return path;
        }

        public static string Build(AppSettings settings, TrackingPipeline pipeline)
        {
            var sb = new StringBuilder();

            sb.AppendLine("Kinect 2.0 -> VMC -> Warudo  诊断报告");
            sb.AppendLine("生成时间：" + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture));
            sb.AppendLine("==============================================================");
            sb.AppendLine();

            AppendEnvironment(sb);
            AppendKinectSdk(sb);
            AppendDeviceNode(sb);
            AppendServiceAndPipes(sb);
            AppendSensorState(sb);
            AppendOutput(sb, settings, pipeline);
            AppendRecentStats(sb, pipeline);
            AppendFooter(sb);

            return sb.ToString();
        }

        // ---------------------------------------------------------------- sections

        /// <summary>
        /// Version of the application that is producing the report.
        ///
        /// Uses the entry assembly rather than the executing one: this class lives in the
        /// application assembly, but a test host that links against it has its own entry
        /// assembly, and the report must name whichever program actually ran.
        /// </summary>
        private static string HostVersion()
        {
            try
            {
                Assembly host = Assembly.GetEntryAssembly();
                if (host != null && host.GetName().Version != null)
                {
                    return host.GetName().Version.ToString();
                }
            }
            catch { /* fall through to the hosting assembly */ }

            try { return Assembly.GetExecutingAssembly().GetName().Version.ToString(); }
            catch { return "未知"; }
        }

        private static void AppendEnvironment(StringBuilder sb)
        {
            sb.AppendLine("[1] 运行环境");
            sb.AppendLine("  程序版本      : " + HostVersion());
            sb.AppendLine("  运行目录      : " + AppDomain.CurrentDomain.BaseDirectory);
            sb.AppendLine("  64 位进程     : " + (Environment.Is64BitProcess ? "是" : "否（Kinect 需要 64 位）"));
            sb.AppendLine("  .NET 版本     : " + Environment.Version);
            sb.AppendLine("  操作系统      : " + Environment.OSVersion);

            // Being able to write beside the exe is what lets settings.txt and this report exist
            // at all; a protected location such as Program Files would break both.
            string probe = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "_write_probe.tmp");
            try
            {
                File.WriteAllText(probe, "x");
                File.Delete(probe);
                sb.AppendLine("  目录可写      : 是");
            }
            catch (Exception ex)
            {
                sb.AppendLine("  目录可写      : 否  -> " + ex.GetType().Name + ": " + ex.Message);
                sb.AppendLine("                  设置文件和诊断报告将无法保存");
            }
            sb.AppendLine();
        }

        private static void AppendKinectSdk(StringBuilder sb)
        {
            sb.AppendLine("[2] Kinect SDK 程序集");

            try
            {
                Assembly sdk = typeof(Microsoft.Kinect.KinectSensor).Assembly;
                sb.AppendLine("  程序集        : " + sdk.GetName().Name + " " + sdk.GetName().Version);
                sb.AppendLine("  加载位置      : " + sdk.Location);

                // The runtime prefers the GAC copy registered by the SDK installer, so which one
                // actually loaded is worth recording.
                bool fromGac = sdk.Location != null
                    && sdk.Location.IndexOf("GAC_", StringComparison.OrdinalIgnoreCase) >= 0;
                sb.AppendLine("  来源          : " + (fromGac ? "全局程序集缓存 (GAC)" : "本地目录 / 其他"));
            }
            catch (Exception ex)
            {
                sb.AppendLine("  加载失败      : " + ex.GetType().Name + ": " + ex.Message);
                sb.AppendLine("                  说明 Kinect for Windows SDK 2.0 未正确安装");
            }

            // The runtime files themselves are what actually talk to the device.
            AppendFileState(sb, @"C:\Windows\System32\Kinect20.dll", "  运行时 DLL");
            AppendFileState(sb,
                @"C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\Assemblies\Microsoft.Kinect.dll",
                "  SDK 程序集文件");
            sb.AppendLine();
        }

        private static void AppendFileState(StringBuilder sb, string path, string label)
        {
            try
            {
                if (!File.Exists(path))
                {
                    sb.AppendLine(label + "  : 缺失 -> " + path);
                    return;
                }

                var info = new FileInfo(path);
                string version = "?";
                try { version = System.Diagnostics.FileVersionInfo.GetVersionInfo(path).FileVersion; }
                catch { /* version is a nicety, not essential */ }

                sb.AppendLine(label + "  : " + version + "  (" + info.Length.ToString(CultureInfo.InvariantCulture) + " 字节)");
            }
            catch (Exception ex)
            {
                sb.AppendLine(label + "  : 读取失败 " + ex.GetType().Name);
            }
        }

        /// <summary>
        /// Looks for the USB device node the Kinect driver binds to. Its absence means the
        /// device is not enumerating at all, which is a cable, port or power problem rather
        /// than a software one.
        /// </summary>
        private static void AppendDeviceNode(StringBuilder sb)
        {
            sb.AppendLine("[3] USB 设备节点（最重要的一项）");
            sb.AppendLine("  期望的硬件 ID : VID_045E&PID_02C4  (Kinect for Windows v2)");

            const string interfaceKey = @"SYSTEM\CurrentControlSet\Enum\USB\VID_045E&PID_02C4&MI_00";
            const string compositeKey = @"SYSTEM\CurrentControlSet\Enum\USB\VID_045E&PID_02C4";

            bool foundInterface = false;
            try
            {
                using (RegistryKey key = Registry.LocalMachine.OpenSubKey(interfaceKey))
                {
                    if (key != null)
                    {
                        string[] instances = key.GetSubKeyNames();
                        if (instances.Length > 0)
                        {
                            foundInterface = true;
                            sb.AppendLine("  KinectSensor 接口 : 存在，实例数 " + instances.Length);
                        }
                        else
                        {
                            sb.AppendLine("  KinectSensor 接口 : 键存在但没有实例");
                        }
                    }
                    else
                    {
                        sb.AppendLine("  KinectSensor 接口 : 不存在");
                    }
                }
            }
            catch (Exception ex)
            {
                sb.AppendLine("  KinectSensor 接口 : 读取失败 " + ex.GetType().Name + ": " + ex.Message);
            }

            try
            {
                using (RegistryKey key = Registry.LocalMachine.OpenSubKey(compositeKey))
                {
                    sb.AppendLine("  USB 复合设备      : " + (key != null ? "存在" : "不存在"));
                }
            }
            catch (Exception ex)
            {
                sb.AppendLine("  USB 复合设备      : 读取失败 " + ex.GetType().Name);
            }

            if (!foundInterface)
            {
                sb.AppendLine();
                sb.AppendLine("  >>> 设备节点缺失。这不是软件问题，请依次检查：");
                sb.AppendLine("      1) Kinect 的 12V 电源是否通电（这台是改造过供电的，重点怀疑）");
                sb.AppendLine("      2) USB 线是否插在 USB 3.0 口上（蓝口 / SS 标记），不要用延长线");
                sb.AppendLine("      3) 换一个 USB 3.0 口，最好让 Kinect 独占一个控制器");
                sb.AppendLine("      4) 设备管理器里是否有带黄色感叹号的未知设备");
            }
            sb.AppendLine();
        }

        /// <summary>
        /// The runtime talks to the device through the KinectMonitor service and its named
        /// pipes. A missing service or missing pipe explains "device present but unavailable".
        /// </summary>
        private static void AppendServiceAndPipes(StringBuilder sb)
        {
            sb.AppendLine("[4] Kinect 运行时服务与管道");

            try
            {
                using (var controller = new ServiceController("KinectMonitor"))
                {
                    sb.AppendLine("  KinectMonitor : " + controller.Status
                        + "（启动类型 " + controller.StartType + "）");
                    if (controller.Status != ServiceControllerStatus.Running)
                    {
                        sb.AppendLine("  >>> 服务未运行。在管理员命令行执行：net start KinectMonitor");
                    }
                }
            }
            catch (Exception ex)
            {
                sb.AppendLine("  KinectMonitor : 查询失败 " + ex.GetType().Name + ": " + ex.Message);
                sb.AppendLine("  >>> 服务不存在说明 Kinect for Windows Runtime 没装好");
            }

            // The client-side pipe appears only once the service has actually created it.
            int nuiPipes = 0;
            bool sawAppPipe = false;
            try
            {
                string[] pipes = Directory.GetFiles(@"\\.\pipe\");
                var interesting = new List<string>();
                foreach (string pipe in pipes)
                {
                    string name = Path.GetFileName(pipe);
                    if (name.StartsWith("nui", StringComparison.OrdinalIgnoreCase)
                        || name.IndexOf("kinect", StringComparison.OrdinalIgnoreCase) >= 0)
                    {
                        interesting.Add(name);
                        nuiPipes++;
                        if (name.StartsWith("nuiapp", StringComparison.OrdinalIgnoreCase)) { sawAppPipe = true; }
                    }
                }

                sb.AppendLine("  Kinect 相关管道: " + nuiPipes + " 个 / 共 " + pipes.Length + " 个管道");
                foreach (string name in interesting) { sb.AppendLine("      " + name); }

                if (!sawAppPipe)
                {
                    sb.AppendLine();
                    sb.AppendLine("  >>> 未看到 nuiapp.* 客户端管道。这一条几乎可以单独定性：");
                    sb.AppendLine("      - 若本程序是在普通桌面双击运行的：服务或运行时不完整，");
                    sb.AppendLine("        在管理员命令行执行  net stop KinectMonitor && net start KinectMonitor");
                    sb.AppendLine("      - 若本程序是在沙箱 / 受限容器 / 远程受限会话里运行的：");
                    sb.AppendLine("        管道被环境切断，这【不是】Kinect 的故障。");
                    sb.AppendLine("        请把 dist\\ 目录整个拷到普通桌面环境再双击运行。");
                }
            }
            catch (Exception ex)
            {
                sb.AppendLine("  管道枚举      : 失败 " + ex.GetType().Name + ": " + ex.Message);
                sb.AppendLine("  >>> 连管道都列不出来，通常说明当前进程被沙箱/权限限制，");
                sb.AppendLine("      而不是 Kinect 有问题。请在普通桌面环境运行本程序。");
            }
            sb.AppendLine();
        }

        private static void AppendSensorState(StringBuilder sb)
        {
            sb.AppendLine("[5] 传感器状态（由本程序实时读取）");

            try
            {
                Microsoft.Kinect.KinectSensor sensor = Microsoft.Kinect.KinectSensor.GetDefault();
                if (sensor == null)
                {
                    sb.AppendLine("  GetDefault()  : 返回 null（没有传感器对象）");
                }
                else
                {
                    sb.AppendLine("  UniqueKinectId: " + (string.IsNullOrEmpty(sensor.UniqueKinectId)
                        ? "(空)" : sensor.UniqueKinectId));
                    sb.AppendLine("  IsAvailable   : " + sensor.IsAvailable);
                    sb.AppendLine("  IsOpen        : " + sensor.IsOpen);
                    sb.AppendLine("  能力          : " + sensor.KinectCapabilities);
                    sb.AppendLine("  可跟踪人数上限: " + sensor.BodyFrameSource.BodyCount);

                    if (!sensor.IsAvailable)
                    {
                        sb.AppendLine();
                        sb.AppendLine("  >>> 设备存在但不可用。常见原因：");
                        sb.AppendLine("      - 12V 供电不足（改造供电版本的首要怀疑对象）");
                        sb.AppendLine("      - USB 带宽不足：Kinect 需要独占一个 USB 3.0 控制器");
                        sb.AppendLine("      - 其他程序（Kinect Studio / KinectToVR）正占用设备");
                        sb.AppendLine("      - 正在沙箱/受限环境中运行");
                    }
                }
            }
            catch (Exception ex)
            {
                sb.AppendLine("  读取失败      : " + ex.GetType().Name + ": " + ex.Message);
            }
            sb.AppendLine();
        }

        private static void AppendOutput(StringBuilder sb, AppSettings settings, TrackingPipeline pipeline)
        {
            sb.AppendLine("[6] 输出配置");

            if (settings == null)
            {
                sb.AppendLine("  设置不可用");
                sb.AppendLine();
                return;
            }

            sb.AppendLine("  目标地址      : " + settings.TargetAddress);
            sb.AppendLine("  目标端口      : " + settings.TargetPort
                + (settings.TargetPort == 39539 ? "（Warudo 默认）" : "（Warudo 默认是 39539）"));
            sb.AppendLine("  平滑强度      : " + settings.Smoothing.ToString("F2", CultureInfo.InvariantCulture)
                + "  (0=原样, 越大越平滑)");
            sb.AppendLine("  发送头部      : " + (settings.SendHead
                ? "是  (注意：需同时关掉 VSeeFace 的头部发送，否则互相覆盖)"
                : "否  (头部交给 VSeeFace，推荐)"));
            sb.AppendLine("  位移跟随      : " + (settings.SendRootPosition ? "开" : "关"));
            sb.AppendLine("  位移限制      : " + settings.MovementLimit.ToString("F2", CultureInfo.InvariantCulture) + " 米");
            sb.AppendLine("  镜像          : " + (settings.Mirror ? "开" : "关"));

            // The three bones that must NOT be sent for the seated half-body mode to work.
            sb.AppendLine("  发送的骨骼    : " + string.Join(", ", VmcBones.DefaultUpperBody));
            sb.AppendLine("  不发送        : Hips, Neck, Head, 所有腿/脚（坐姿半身模式，腿回落到 Idle 动画）");
            sb.AppendLine();
        }

        private static void AppendRecentStats(StringBuilder sb, TrackingPipeline pipeline)
        {
            sb.AppendLine("[7] 运行统计");

            if (pipeline == null)
            {
                sb.AppendLine("  管线不可用");
                sb.AppendLine();
                return;
            }

            sb.AppendLine("  运行状态      : " + pipeline.State + "  (" + pipeline.StatusText + ")");
            sb.AppendLine("  传感器可用    : " + pipeline.SensorAvailable);
            sb.AppendLine("  传感器 ID     : " + (string.IsNullOrEmpty(pipeline.SensorId) ? "(空)" : pipeline.SensorId));
            sb.AppendLine("  已跟踪人数    : " + pipeline.TrackedBodyCount);
            sb.AppendLine("  帧率          : " + pipeline.FrameRate.ToString("F1", CultureInfo.InvariantCulture) + " 帧/秒");
            sb.AppendLine("  已发骨骼数    : " + pipeline.BoneCount + "  (正常应为 11，开启头部则 13)");
            sb.AppendLine("  已发包数      : " + pipeline.PacketsSent);
            sb.AppendLine("  旋转基准      : " + (pipeline.UsesCapturedNeutral
                ? "当前坐姿（自定义，适合手臂前后摆动）"
                : "水平 T 字（默认，适合手臂前伸/侧展）"));
            if (!string.IsNullOrEmpty(pipeline.LastError))
            {
                sb.AppendLine("  最近错误      : " + pipeline.LastError);
            }
            sb.AppendLine();
        }

        private static void AppendFooter(StringBuilder sb)
        {
            sb.AppendLine("[8] 请把这个文件发回给开发者");
            sb.AppendLine("==============================================================");
            sb.AppendLine("说明：");
            sb.AppendLine("  - 本报告不含任何个人信息，只有设备与配置状态");
            sb.AppendLine("  - 若第 [3] 节显示设备节点缺失，请先解决硬件问题再看其他部分");
            sb.AppendLine("  - 若第 [4] 节显示管道被切断，说明当前不是普通桌面会话，");
            sb.AppendLine("    请把程序拷到普通桌面环境双击运行");
        }
    }
}
