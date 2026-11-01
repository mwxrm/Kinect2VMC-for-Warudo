using System;
using System.Collections.Generic;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Kinect2VMC.Core;

namespace Kinect2VMC.Monitor
{
    /// <summary>
    /// Listens for VMC Protocol traffic and prints a live summary.
    ///
    /// Its purpose is diagnosis: it answers "is anything actually being sent, and with which
    /// bone names?" without involving Warudo. Point the sender at this port to check the
    /// pipeline end to end, then point it back at Warudo's receiver.
    /// </summary>
    internal static class Program
    {
        private static int Main(string[] args)
        {
            Console.OutputEncoding = Encoding.UTF8;

            int port = 39539;
            if (args.Length > 0)
            {
                int parsed;
                if (int.TryParse(args[0], out parsed) && parsed > 0 && parsed <= 65535)
                {
                    port = parsed;
                }
            }

            Console.WriteLine("VMC 监听器");
            Console.WriteLine("==========");
            Console.WriteLine("监听端口：" + port);
            Console.WriteLine("把发送端的目标端口改成这个端口，就能在这里看到收到的数据。");
            Console.WriteLine("按 Ctrl+C 退出。");
            Console.WriteLine();

            UdpClient listener;
            try
            {
                listener = new UdpClient(new IPEndPoint(IPAddress.Any, port));
            }
            catch (SocketException ex)
            {
                Console.WriteLine("无法绑定端口 " + port + "：" + ex.Message);
                Console.WriteLine("端口可能已被占用（例如 Warudo 的 VMC 接收器正在使用它）。");
                Console.WriteLine("用法：VmcMonitor.exe [端口]");
                return 2;
            }

            using (listener)
            {
                listener.Client.ReceiveTimeout = 500;

                var stats = new Dictionary<string, long>(StringComparer.Ordinal);
                var lastRotations = new Dictionary<string, Quat>(StringComparer.Ordinal);
                long totalPackets = 0;
                long totalBytes = 0;
                long lastReportPackets = 0;
                DateTime windowStart = DateTime.UtcNow;
                DateTime lastAnyPacket = DateTime.MinValue;
                string lastStatus = "(尚未收到)";

                var pending = new List<OscMessage>(64);

                while (true)
                {
                    try
                    {
                        var remote = new IPEndPoint(IPAddress.Any, 0);
                        byte[] data = listener.Receive(ref remote);
                        totalPackets++;
                        totalBytes += data.Length;
                        lastAnyPacket = DateTime.UtcNow;

                        OscMessage message;
                        if (OscMessageReader.TryParse(data, data.Length, out message))
                        {
                            long count;
                            stats.TryGetValue(message.Address, out count);
                            stats[message.Address] = count + 1;

                            if (message.Address == "/VMC/Ext/Bone/Pos" && message.Arguments.Length >= 1)
                            {
                                string bone = message.Arguments[0].StringValue ?? "?";
                                if (message.Arguments.Length >= 8)
                                {
                                    lastRotations[bone] = new Quat(
                                        message.Arguments[4].FloatValue,
                                        message.Arguments[5].FloatValue,
                                        message.Arguments[6].FloatValue,
                                        message.Arguments[7].FloatValue);
                                }
                            }
                            else if (message.Address == "/VMC/Ext/OK")
                            {
                                var parts = new StringBuilder();
                                for (int i = 0; i < message.Arguments.Length; i++)
                                {
                                    if (i > 0) { parts.Append(", "); }
                                    parts.Append(message.Arguments[i].IntValue);
                                }
                                lastStatus = parts.ToString();
                            }
                        }
                    }
                    catch (SocketException)
                    {
                        // Receive timeout: fall through to the periodic report.
                    }

                    double elapsed = (DateTime.UtcNow - windowStart).TotalSeconds;
                    if (elapsed >= 1.0)
                    {
                        double packetsPerSecond = (totalPackets - lastReportPackets) / elapsed;
                        lastReportPackets = totalPackets;
                        windowStart = DateTime.UtcNow;

                        Report(totalPackets, totalBytes, packetsPerSecond, stats, lastRotations,
                            lastStatus, lastAnyPacket);
                    }
                }
            }
        }

        private static void Report(long totalPackets, long totalBytes, double packetsPerSecond,
            Dictionary<string, long> stats, Dictionary<string, Quat> rotations,
            string status, DateTime lastAnyPacket)
        {
            Console.WriteLine("──────────────────────────────────────────────────────────────");
            Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "总计 {0} 包 / {1} 字节　速率 {2:F0} 包/秒　状态位 {3}",
                totalPackets, totalBytes, packetsPerSecond, status));

            if (lastAnyPacket != DateTime.MinValue)
            {
                double since = (DateTime.UtcNow - lastAnyPacket).TotalSeconds;
                if (since > 1.5)
                {
                    Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
                        "⚠ 已有 {0:F1} 秒没有收到任何数据", since));
                }
            }

            Console.WriteLine("地址统计：");
            foreach (var pair in stats)
            {
                Console.WriteLine("   " + pair.Key.PadRight(24) + pair.Value.ToString(CultureInfo.InvariantCulture));
            }

            Console.WriteLine("最近骨骼旋转（" + rotations.Count.ToString(CultureInfo.InvariantCulture) + " 根）：");
            foreach (var pair in rotations)
            {
                Quat q = pair.Value;
                Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "   {0,-16} {1,7:F3} {2,7:F3} {3,7:F3} {4,7:F3}",
                    pair.Key, q.X, q.Y, q.Z, q.W));
            }
            Console.WriteLine();
        }
    }
}
