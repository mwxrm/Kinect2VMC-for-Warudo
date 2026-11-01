using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;

namespace Kinect2VMC.Core
{
    /// <summary>
    /// Unity <c>HumanBodyBones</c> names used by VMC Protocol. The receiver builds its
    /// lookup table from <c>Enum.GetValues(typeof(HumanBodyBones))</c>, so these strings
    /// must match the enum member names exactly.
    /// </summary>
    public static class VmcBones
    {
        public const string Hips = "Hips";
        public const string Spine = "Spine";
        public const string Chest = "Chest";
        public const string UpperChest = "UpperChest";
        public const string Neck = "Neck";
        public const string Head = "Head";
        public const string LeftShoulder = "LeftShoulder";
        public const string LeftUpperArm = "LeftUpperArm";
        public const string LeftLowerArm = "LeftLowerArm";
        public const string LeftHand = "LeftHand";
        public const string RightShoulder = "RightShoulder";
        public const string RightUpperArm = "RightUpperArm";
        public const string RightLowerArm = "RightLowerArm";
        public const string RightHand = "RightHand";

        /// <summary>
        /// The thumb root of each hand. Sent only when finger tracking is on, and used to carry
        /// the palm's roll: a single bone direction cannot describe a hand's twist, and the thumb
        /// is the only landmark Kinect gives that sits across the palm.
        /// </summary>
        public const string LeftThumb = "LeftThumb";
        public const string RightThumb = "RightThumb";

        // Full-body mode. Sent only when the user asks for it: the legs are unreliable from a
        // seated position and a wrong leg pose looks worse than leaving the idle animation alone.
        public const string LeftUpperLeg = "LeftUpperLeg";
        public const string LeftLowerLeg = "LeftLowerLeg";
        public const string LeftFoot = "LeftFoot";
        public const string RightUpperLeg = "RightUpperLeg";
        public const string RightLowerLeg = "RightLowerLeg";
        public const string RightFoot = "RightFoot";

        /// <summary>
        /// The default upper-body set. Deliberately excludes Hips, Neck and Head:
        ///  - Hips is left to the character's idle animation so a seated avatar keeps a stable pelvis;
        ///  - Neck/Head are owned by the face tracking source (VSeeFace) to avoid two VMC
        ///    senders fighting over the same bones.
        /// </summary>
        public static readonly string[] DefaultUpperBody = new[]
        {
            Spine, Chest, UpperChest,
            LeftShoulder, LeftUpperArm, LeftLowerArm, LeftHand,
            RightShoulder, RightUpperArm, RightLowerArm, RightHand
        };

        public static readonly string[] WithHead = new[]
        {
            Spine, Chest, UpperChest, Neck, Head,
            LeftShoulder, LeftUpperArm, LeftLowerArm, LeftHand,
            RightShoulder, RightUpperArm, RightLowerArm, RightHand
        };

        public static bool IsUpperBody(string name)
        {
            for (int i = 0; i < DefaultUpperBody.Length; i++)
            {
                if (DefaultUpperBody[i] == name) { return true; }
            }
            return name == Head || name == Neck;
        }
    }

    /// <summary>Tracking status flags for the <c>/VMC/Ext/OK</c> message.</summary>
    [Flags]
    public enum VmcTrackingStatus
    {
        None = 0,
        Loaded = 1,
        Tracking = 2
    }

    /// <summary>An immutable snapshot of the pose the sender will transmit.</summary>
    public sealed class SkeletonPose
    {
        /// <summary>World-space (character-space) rotation per bone name, in Unity's left-handed convention.</summary>
        public readonly Dictionary<string, Quat> BoneRotations = new Dictionary<string, Quat>(StringComparer.Ordinal);

        /// <summary>Optional root translation applied to the whole character.</summary>
        public bool HasRootPosition;
        public Vec3 RootPosition;

        /// <summary>
        /// Optional root rotation about the vertical axis, so the avatar turns with the user.
        ///
        /// Without it the character keeps facing one way while the arms swing around a body that
        /// never moves, which is exactly what a small turn of the shoulders looks like: the arms
        /// drift rather than the body rotating.
        /// </summary>
        public bool HasRootRotation;
        public Quat RootRotation = Quat.Identity;

        public VmcTrackingStatus Status = VmcTrackingStatus.Loaded;
        public bool Tracking;

        /// <summary>Monotonic sender time in seconds, echoed through <c>/VMC/Ext/T</c>.</summary>
        public float Time;

        public void SetBone(string name, Quat rotation)
        {
            BoneRotations[name] = rotation;
        }
    }

    /// <summary>
    /// Sends a <see cref="SkeletonPose"/> as VMC Protocol traffic over UDP.
    ///
    /// Only the messages Warudo's VMC receiver actually reads are emitted:
    ///   /VMC/Ext/OK, /VMC/Ext/T, /VMC/Ext/Bone/Pos, /VMC/Ext/Root/Pos
    /// Bone positions are intentionally NOT sent: Warudo overrides local rotations and
    /// does not solve rotations from positions, so sending positions can only add noise
    /// or stretch the rig.
    /// </summary>
    public sealed class VmcSender : IDisposable
    {
        private readonly OscMessageWriter _writer = new OscMessageWriter(2048);
        private UdpClient _client;
        private IPEndPoint _endPoint;
        private string _address = "127.0.0.1";
        private int _port = 39539;

        public string TargetAddress { get { return _address; } }
        public int TargetPort { get { return _port; } }

        public long PacketsSent { get; private set; }
        public long BytesSent { get; private set; }
        public string LastError { get; private set; }

        /// <summary>When false, <c>/VMC/Ext/Root/Pos</c> is never emitted.</summary>
        public bool SendRootPosition = true;

        /// <summary>Re-sends <c>/VMC/Ext/OK</c> and <c>/VMC/Ext/T</c> at most this often, in seconds.</summary>
        public float StatusInterval = 1.0f;

        private float _lastStatusTime = float.NegativeInfinity;

        public void Configure(string address, int port)
        {
            if (string.IsNullOrEmpty(address)) { address = "127.0.0.1"; }
            if (port <= 0 || port > 65535) { port = 39539; }

            IPAddress parsed;
            if (!IPAddress.TryParse(address, out parsed))
            {
                parsed = IPAddress.Loopback;
                address = "127.0.0.1";
            }

            bool changed = _client == null || _address != address || _port != port;
            _address = address;
            _port = port;
            _endPoint = new IPEndPoint(parsed, port);

            if (changed && _client != null)
            {
                try { _client.Close(); } catch { /* recreating below */ }
                _client = null;
            }
        }

        private UdpClient Client
        {
            get
            {
                if (_client == null)
                {
                    _client = new UdpClient();
                    _client.EnableBroadcast = true;
                    try { _client.Client.SendBufferSize = 1 << 16; } catch { /* best effort */ }
                    try { _client.Client.Blocking = false; } catch { /* best effort */ }
                }
                return _client;
            }
        }

        public bool Send(SkeletonPose pose, bool forceStatus = false)
        {
            if (pose == null) { return false; }
            if (_endPoint == null) { Configure(_address, _port); }

            bool ok = true;

            if (forceStatus || pose.Time - _lastStatusTime >= StatusInterval)
            {
                _lastStatusTime = pose.Time;
                ok &= SendStatus(pose);
            }

            // Root translation and/or rotation. The VMC 2.1 form is used, so the message carries
            // the full parameter set; Warudo reads only the first seven arguments and ignores the
            // MR scale/offset fields.
            //
            // The rotation fields were previously hard-coded to identity, which quietly discarded
            // any root rotation. They now carry the pose's own rotation, so the character turns
            // with the user instead of the arms swinging around a body that never moves.
            // Rotation is NOT gated behind SendRootPosition.
            //
            // Both fields travel in the same message, and the earlier code treated the position
            // switch as the switch for the message as a whole. That coupled two settings the UI
            // presents separately: clearing the "position" box silently stopped the character from
            // TURNING as well, with nothing on screen to say why. Turning is requested
            // independently by the solver (HasRootRotation), so honour it independently here and
            // send the message whenever either field is wanted.
            bool wantTranslation = SendRootPosition && pose.HasRootPosition && pose.RootPosition.IsFinite();
            bool wantRotation = pose.HasRootRotation;

            if (wantTranslation || wantRotation)
            {
                Vec3 translation = wantTranslation ? pose.RootPosition : Vec3.Zero;
                Quat rotation = wantRotation ? pose.RootRotation : Quat.Identity;

                _writer.Begin("/VMC/Ext/Root/Pos");
                _writer.AddString("root");
                _writer.AddFloat(translation.X);
                _writer.AddFloat(translation.Y);
                _writer.AddFloat(translation.Z);
                _writer.AddFloat(rotation.X);
                _writer.AddFloat(rotation.Y);
                _writer.AddFloat(rotation.Z);
                _writer.AddFloat(rotation.W);
                _writer.AddFloat(1f);
                _writer.AddFloat(1f);
                _writer.AddFloat(1f);
                _writer.AddFloat(0f);
                _writer.AddFloat(0f);
                _writer.AddFloat(0f);
                ok &= Transmit();
            }

            foreach (var pair in pose.BoneRotations)
            {
                Quat q = pair.Value;
                if (!q.IsFinite()) { continue; }

                WriteBoneTransform(pair.Key, q);
                ok &= Transmit();
            }

            return ok;
        }

        /// <summary>
        /// Writes one <c>/VMC/Ext/Bone/Pos</c> message.
        ///
        /// Position is always zero: Warudo overrides local rotations and never solves
        /// rotations from positions, so sending a position can only add noise or stretch
        /// the rig.
        /// </summary>
        private void WriteBoneTransform(string boneName, Quat rotation)
        {
            _writer.Begin("/VMC/Ext/Bone/Pos");
            _writer.AddString(boneName);
            _writer.AddFloat(0f);
            _writer.AddFloat(0f);
            _writer.AddFloat(0f);
            _writer.AddFloat(rotation.X);
            _writer.AddFloat(rotation.Y);
            _writer.AddFloat(rotation.Z);
            _writer.AddFloat(rotation.W);
        }

        private bool SendStatus(SkeletonPose pose)
        {
            bool ok = true;

            _writer.Begin("/VMC/Ext/OK");
            _writer.AddInt((pose.Status & VmcTrackingStatus.Loaded) != 0 ? 1 : 0);
            _writer.AddInt(3);                                        // calibration: Calibrated
            _writer.AddInt(0);                                        // calibration mode: Normal
            _writer.AddInt(pose.Tracking ? 1 : 0);                    // VMC 2.7 tracking status
            ok &= Transmit();

            _writer.Begin("/VMC/Ext/T");
            _writer.AddFloat(pose.Time);
            ok &= Transmit();

            return ok;
        }

        private bool Transmit()
        {
            int length = _writer.End();
            try
            {
                Client.Send(_writer.Buffer, length, _endPoint);
                PacketsSent++;
                BytesSent += length;
                LastError = null;
                return true;
            }
            catch (SocketException ex)
            {
                // Non-blocking sends can legitimately report WouldBlock; not fatal.
                if (ex.SocketErrorCode != SocketError.WouldBlock) { LastError = ex.Message; }
                return false;
            }
            catch (ObjectDisposedException)
            {
                _client = null;
                return false;
            }
            catch (Exception ex)
            {
                LastError = ex.Message;
                return false;
            }
        }

        /// <summary>Encodes a single message without sending. Used by tests.</summary>
        public byte[] EncodeStatus(SkeletonPose pose, out int length)
        {
            _writer.Begin("/VMC/Ext/OK");
            _writer.AddInt((pose.Status & VmcTrackingStatus.Loaded) != 0 ? 1 : 0);
            _writer.AddInt(3);
            _writer.AddInt(0);
            _writer.AddInt(pose.Tracking ? 1 : 0);
            length = _writer.End();
            var copy = new byte[length];
            Array.Copy(_writer.Buffer, copy, length);
            return copy;
        }

        /// <summary>Encodes a bone message without sending. Used by tests.</summary>
        public byte[] EncodeBone(string name, Quat rotation, out int length)
        {
            WriteBoneTransform(name, rotation);
            length = _writer.End();
            var copy = new byte[length];
            Array.Copy(_writer.Buffer, copy, length);
            return copy;
        }

        public void Dispose()
        {
            if (_client != null)
            {
                try { _client.Close(); } catch { /* nothing left to do */ }
                _client = null;
            }
        }
    }
}
