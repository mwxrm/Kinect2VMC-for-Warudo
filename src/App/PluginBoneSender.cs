using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Kinect2VMC.Core;

namespace Kinect2VMC.App
{
    /// <summary>
    /// 把整套骨架直接发给 Warudo 里的插件，不经过 VMC。
    ///
    /// 为什么要有这条路
    /// ----------------
    /// 同一个骨骼有两个写入者时，谁后写谁生效，顺序很脆 —— IK 尤其抢不过 VMC。
    /// Warudo 自带的 MediaPipe 追踪器之所以动作干净，正是因为它是进程内插件、
    /// 独占骨骼。这条通路做的就是同一件事。
    ///
    /// 包格式（小端）：
    ///     "K2VS"                     4
    ///     uint32 counter             4
    ///     uint8  flags               1      bit0 = 根位置有效, bit1 = 根旋转有效
    ///     float  rootPos.x/y/z       12
    ///     float  rootRot.x/y/z/w     16
    ///     uint16 boneCount           2
    ///     每根骨骼：
    ///         uint8  nameLen
    ///         bytes  name (UTF8)     例如 "LeftLowerArm"
    ///         float  x, y, z, w      局部旋转
    ///
    /// 发的是骨骼"名字"而不是序号：名字由 Warudo 自己解析成 HumanBodyBones，
    /// 不依赖手抄的枚举表。之前"发送 55/56、实际是 17/18"那个坑，就是手抄表的代价 ——
    /// 当时所有包都被当成非法骨骼静默丢弃，现象看起来像"网络不通"。
    /// </summary>
    public sealed class PluginBoneSender : IDisposable
    {
        public const int DefaultPort = 39542;

        private static readonly byte[] Magic = { (byte)'K', (byte)'2', (byte)'V', (byte)'S' };

        private UdpClient _socket;
        private IPEndPoint _target;

        // 复用缓冲区：每帧新建数组会让 GC 在 60 Hz 下持续工作。
        private readonly byte[] _buffer = new byte[8192];
        private uint _counter;

        /// <summary>与 VMC 那条路保持一致：根位置的发送单独开关。</summary>
        public bool SendRootPosition = true;

        public string LastError { get; private set; }
        public long FramesSent { get; private set; }
        public long BytesSent { get; private set; }
        public int LastBoneCount { get; private set; }

        public void Configure(string host, int port)
        {
            try
            {
                if (_socket == null)
                {
                    _socket = new UdpClient();
                }

                IPAddress address;
                if (!IPAddress.TryParse(host, out address))
                {
                    address = IPAddress.Loopback;
                }

                _target = new IPEndPoint(address, port);
                LastError = null;
            }
            catch (Exception e)
            {
                LastError = e.Message;
            }
        }

        /// <summary>
        /// 发送一帧。永远不抛：动捕不该因为一个诊断通路而中断。
        /// </summary>
        public bool Send(SkeletonPose pose)
        {
            if (_socket == null || _target == null || pose == null)
            {
                return false;
            }

            try
            {
                int offset = 0;

                Buffer.BlockCopy(Magic, 0, _buffer, offset, Magic.Length);
                offset += Magic.Length;

                _counter++;
                Buffer.BlockCopy(BitConverter.GetBytes(_counter), 0, _buffer, offset, 4);
                offset += 4;

                bool hasPos = SendRootPosition && pose.HasRootPosition && pose.RootPosition.IsFinite();
                bool hasRot = pose.HasRootRotation;

                byte flags = (byte)((hasPos ? 1 : 0) | (hasRot ? 2 : 0));
                _buffer[offset++] = flags;

                WriteFloat3(pose.RootPosition, ref offset);
                WriteQuat(pose.RootRotation, ref offset);

                // 骨骼数量先占位，写完再回填 —— 这样不必为了计数先遍历两遍。
                int countOffset = offset;
                offset += 2;
                int written = 0;

                foreach (var pair in pose.BoneRotations)
                {
                    byte[] name = Encoding.UTF8.GetBytes(pair.Key);
                    if (name.Length == 0 || name.Length > 255)
                    {
                        continue;
                    }

                    // 放不下就停在这一根，不写半个包 —— 半个包比少一根骨骼更难查。
                    if (offset + 1 + name.Length + 16 > _buffer.Length)
                    {
                        break;
                    }

                    _buffer[offset++] = (byte)name.Length;
                    Buffer.BlockCopy(name, 0, _buffer, offset, name.Length);
                    offset += name.Length;
                    WriteQuat(pair.Value, ref offset);
                    written++;
                }

                Buffer.BlockCopy(BitConverter.GetBytes((ushort)written), 0, _buffer, countOffset, 2);

                _socket.Send(_buffer, offset, _target);

                FramesSent++;
                BytesSent += offset;
                LastBoneCount = written;
                LastError = null;
                return true;
            }
            catch (Exception e)
            {
                LastError = e.Message;
                return false;
            }
        }

        private void WriteFloat3(Vec3 value, ref int offset)
        {
            Buffer.BlockCopy(BitConverter.GetBytes(value.X), 0, _buffer, offset, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(value.Y), 0, _buffer, offset + 4, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(value.Z), 0, _buffer, offset + 8, 4);
            offset += 12;
        }

        private void WriteQuat(Quat value, ref int offset)
        {
            Buffer.BlockCopy(BitConverter.GetBytes(value.X), 0, _buffer, offset, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(value.Y), 0, _buffer, offset + 4, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(value.Z), 0, _buffer, offset + 8, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(value.W), 0, _buffer, offset + 12, 4);
            offset += 16;
        }

        public void Dispose()
        {
            try
            {
                if (_socket != null)
                {
                    _socket.Close();
                }
            }
            catch (Exception)
            {
                // 关闭失败没有补救手段，也不该影响退出流程。
            }

            _socket = null;
            _target = null;
        }
    }
}
