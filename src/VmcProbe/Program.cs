// Reads Warudo's own VMC output so the model's real state can be inspected.
//
// Warudo can act as a VMC SENDER as well as a receiver. Pointing that at a spare port lets us
// see exactly what the character is doing: which bones it has, and what rotations it applies.
// That turns "the hand looks wrong" into a number.
//
// Usage:  VmcProbe.exe [port] [seconds]
//   port     UDP port Warudo's VMC sender is set to (default 39540)
//   seconds  how long to listen (default 10)
//
// It also sends /VMC/Ext/OK and /VMC/Ext/T back on the same port, which is how a VMC sender
// learns a receiver is present. Without that some senders stay silent.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace VmcProbe
{
    internal static class Program
    {
        private static int Main(string[] args)
        {
            int port = args.Length > 0 ? int.Parse(args[0], CultureInfo.InvariantCulture) : 39540;
            int seconds = args.Length > 1 ? int.Parse(args[1], CultureInfo.InvariantCulture) : 10;

            Console.WriteLine("VMC probe: listening on UDP " + port + " for " + seconds + " second(s)");
            Console.WriteLine();

            var seen = new Dictionary<string, string>(StringComparer.Ordinal);
            var addresses = new Dictionary<string, int>(StringComparer.Ordinal);
            var boneNames = new List<string>();
            int packets = 0;
            string lastSender = null;

            using (var client = new UdpClient())
            {
                client.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                client.Client.Bind(new IPEndPoint(IPAddress.Any, port));

                // Announce ourselves as a receiver so a VMC sender starts talking.
                SendSubscription(client, port);

                DateTime deadline = DateTime.UtcNow.AddSeconds(seconds);
                while (DateTime.UtcNow < deadline)
                {
                    int remaining = (int)Math.Max(0, (deadline - DateTime.UtcNow).TotalMilliseconds);
                    if (remaining <= 0) { break; }

                    if (client.Available <= 0)
                    {
                        Thread.Sleep(5);
                        continue;
                    }

                    IPEndPoint from = null;
                    byte[] data = client.Receive(ref from);
                    if (data == null || data.Length == 0) { continue; }

                    packets++;
                    lastSender = from.ToString();

                    string address;
                    List<object> values;
                    if (!Osc.TryParse(data, out address, out values)) { continue; }

                    int count;
                    addresses.TryGetValue(address, out count);
                    addresses[address] = count + 1;

                    // Record one sample per bone so the table stays readable.
                    if (address == "/VMC/Ext/Bone/Pos" && values.Count >= 8)
                    {
                        string bone = values[0] as string;
                        if (bone != null && !seen.ContainsKey(bone))
                        {
                            boneNames.Add(bone);
                            seen[bone] = string.Format(CultureInfo.InvariantCulture,
                                "({0:F4}, {1:F4}, {2:F4}, {3:F4})",
                                ToFloat(values[4]), ToFloat(values[5]), ToFloat(values[6]), ToFloat(values[7]));
                        }
                    }
                    else if (!seen.ContainsKey(address))
                    {
                        var sb = new StringBuilder();
                        for (int i = 0; i < values.Count && i < 8; i++)
                        {
                            if (i > 0) { sb.Append(", "); }
                            sb.Append(values[i] is float
                                ? ((float)values[i]).ToString("F4", CultureInfo.InvariantCulture)
                                : Convert.ToString(values[i], CultureInfo.InvariantCulture));
                        }
                        seen[address] = sb.ToString();
                    }
                }
            }

            Console.WriteLine("packets received : " + packets);
            Console.WriteLine("last sender      : " + (lastSender ?? "(none)"));
            Console.WriteLine();

            if (packets == 0)
            {
                Console.WriteLine("NOTHING ARRIVED. Check:");
                Console.WriteLine("  - Warudo's VMC sender is enabled and set to 127.0.0.1:" + port);
                Console.WriteLine("  - the port here matches it (this run used " + port + ")");
                Console.WriteLine("  - Windows Firewall is not blocking loopback UDP");
                return 2;
            }

            Console.WriteLine("address counts:");
            foreach (var pair in addresses)
            {
                Console.WriteLine("  " + pair.Key.PadRight(24) + " x" + pair.Value);
            }

            Console.WriteLine();
            Console.WriteLine("first sample per address:");
            foreach (var pair in seen)
            {
                Console.WriteLine("  " + pair.Key.PadRight(24) + " " + pair.Value);
            }

            Console.WriteLine();
            Console.WriteLine("bones seen (" + boneNames.Count + "):");
            boneNames.Sort(StringComparer.Ordinal);
            foreach (string bone in boneNames)
            {
                Console.WriteLine("  " + bone.PadRight(24) + " " + seen[bone]);
            }

            return 0;
        }

        private static float ToFloat(object value)
        {
            return value is float ? (float)value : 0f;
        }

        private static void SendSubscription(UdpClient client, int port)
        {
            try
            {
                var target = new IPEndPoint(IPAddress.Loopback, port);

                // /VMC/Ext/OK  i i i
                var ok = new List<byte>();
                AppendString(ok, "/VMC/Ext/OK");
                AppendString(ok, ",iii");
                AppendInt(ok, 1);
                AppendInt(ok, 1);
                AppendInt(ok, 1);
                client.Send(ok.ToArray(), ok.Count, target);

                // /VMC/Ext/T  f
                var time = new List<byte>();
                AppendString(time, "/VMC/Ext/T");
                AppendString(time, ",f");
                AppendFloat(time, (float)(DateTime.UtcNow.Ticks / 1e7));
                client.Send(time.ToArray(), time.Count, target);
            }
            catch
            {
                // Announcing is best-effort; listening is what matters.
            }
        }

        private static void AppendString(List<byte> buffer, string value)
        {
            buffer.AddRange(Encoding.ASCII.GetBytes(value));
            buffer.Add(0);
            while (buffer.Count % 4 != 0) { buffer.Add(0); }
        }

        private static void AppendInt(List<byte> buffer, int value)
        {
            byte[] bytes = BitConverter.GetBytes(value);
            if (BitConverter.IsLittleEndian) { Array.Reverse(bytes); }
            buffer.AddRange(bytes);
        }

        private static void AppendFloat(List<byte> buffer, float value)
        {
            byte[] bytes = BitConverter.GetBytes(value);
            if (BitConverter.IsLittleEndian) { Array.Reverse(bytes); }
            buffer.AddRange(bytes);
        }
    }

    /// <summary>Minimal OSC 1.0 reader: enough for the addresses VMC uses.</summary>
    internal static class Osc
    {
        public static bool TryParse(byte[] data, out string address, out List<object> values)
        {
            address = null;
            values = new List<object>();

            if (data == null || data.Length < 8 || data[0] != (byte)'/') { return false; }

            int offset = 0;
            if (!ReadString(data, ref offset, out address)) { return false; }

            string tags;
            if (!ReadString(data, ref offset, out tags)) { return false; }
            if (tags.Length == 0 || tags[0] != ',') { return false; }

            for (int i = 1; i < tags.Length; i++)
            {
                char tag = tags[i];
                if (tag == 'f')
                {
                    if (offset + 4 > data.Length) { return false; }
                    byte[] raw = new byte[4];
                    Array.Copy(data, offset, raw, 0, 4);
                    if (BitConverter.IsLittleEndian) { Array.Reverse(raw); }
                    values.Add(BitConverter.ToSingle(raw, 0));
                    offset += 4;
                }
                else if (tag == 'i')
                {
                    if (offset + 4 > data.Length) { return false; }
                    byte[] raw = new byte[4];
                    Array.Copy(data, offset, raw, 0, 4);
                    if (BitConverter.IsLittleEndian) { Array.Reverse(raw); }
                    values.Add(BitConverter.ToInt32(raw, 0));
                    offset += 4;
                }
                else if (tag == 's')
                {
                    string text;
                    if (!ReadString(data, ref offset, out text)) { return false; }
                    values.Add(text);
                }
                else
                {
                    // Unknown tag: stop rather than misparse the rest.
                    break;
                }
            }

            return true;
        }

        private static bool ReadString(byte[] data, ref int offset, out string value)
        {
            value = null;
            if (offset >= data.Length) { return false; }

            int start = offset;
            while (offset < data.Length && data[offset] != 0) { offset++; }
            if (offset >= data.Length) { return false; }

            value = Encoding.ASCII.GetString(data, start, offset - start);
            offset++;

            while (offset % 4 != 0 && offset < data.Length) { offset++; }
            return true;
        }
    }
}
