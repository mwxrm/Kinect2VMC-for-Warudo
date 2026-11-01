using System;
using System.Net;
using System.Net.Sockets;
using UnityEngine;
using RootMotion.FinalIK;
using Warudo.Core.Attributes;
using Warudo.Plugins.Core.Assets.Character;

namespace Kinect2VMC {

    /// <summary>
    /// 骨骼端口探针：接收 UDP 包，直接把四元数写进骨骼的 localRotation。
    ///
    /// 包格式（小端，20 字节）：
    ///     int32  boneIndex     UnityEngine.HumanBodyBones 索引
    ///     float  x, y, z, w    四元数（局部旋转）
    ///
    /// 自检
    /// ----
    /// 外部发送端一直收不到，但同一个机制在 VMC 端口上是通的，所以先要区分
    /// 「套接字/回环本身有问题」和「外部包没送到」。做法：启动后自己向自己发一个包，
    /// 如果连自己发的都收不到，问题就在套接字这一侧；反之则在发送端。
    /// </summary>
    [AssetType(Id = "b1a2c3d4-0001-4a2b-9c3d-000000000001", Title = "Bone Port Probe")]
    public class BonePortProbeAsset : CharacterDaemonAsset {

        [DataInput]
        [Label("监听端口")]
        public int Port = 39542;

        [Markdown]
        public string Status = "尚未启动";

        [DataInput]
        [Label("已接收包数")]
        public int Received;

        [DataInput]
        [Label("已写入骨骼次数")]
        public int Applied;

        [DataInput]
        [Label("最近一次")]
        public string Last = "(还没有收到数据)";

        // ── 掌心帧的坐标换算开关 ──
        //
        // 用户面对摄像头时：MediaPipe 的 x 与角色 x 同向，y 和 z 都要翻转
        //（MediaPipe 的 y 向下、z 指向屏幕内）。三个开关放在这里，是为了万一某个符号不对，
        // 能当场翻，而不用改代码重编译。
        [DataInput]
        [Label("镜像左右手（面对模型时开启）")]
        public bool MirrorHands = true;

        // 用户面对摄像头时：MediaPipe 的 x 与角色 x 同向，y 和 z 都要翻转
        //（MediaPipe 的 y 向下、z 指向屏幕内）。三个开关放在这里，是为了万一某个符号不对，
        // 能当场翻，而不用改代码重编译。
        [DataInput]
        [Label("翻转相机 X")]
        public bool FlipCameraX = true;

        [DataInput]
        [Label("翻转相机 Y")]
        public bool FlipCameraY = true;

        [DataInput]
        [Label("翻转相机 Z")]
        public bool FlipCameraZ = true;

        [DataInput]
        [Label("前臂分担滚转（0=全给手腕，会拧细）")]
        public float ForearmTwistShare = 0.6f;

        [DataInput]
        [Label("上臂分担滚转（肘部跟着转才不拧）")]
        public float UpperArmTwistShare = 0f;

        // 扭转角上限。
        //
        // Atan2(rel.x, rel.w) 在接近 ±180° 时会突然翻号：前臂于是在两个朝向之间来回弹，
        // 表现为"模型胳膊反复抽动"（SDK 和程序端的骨架都正常，因为问题出在插件写骨骼这一步）。
        // 上臂分担会放大这个跳动，所以它也默认关闭。
        [DataInput]
        [Label("扭转角上限（度，防止 ±180° 跳变）")]
        public float TwistLimitDegrees = 60f;

        // 转身死区（度）。
        //
        // 发送端（校准过基准、且基准取的是平滑后的肩线）算出的偏航仍带零点几度的残余
        // 抖动，小于死区的偏航视为"未转身"，不动脊柱。
        //
        // ⚠ 不要靠调大它来消偏移：那样就变成"转到一定角度才突然跳过去"，完全不可用。
        // 真正的漂移必须在发送端解决（基准要取平滑后的肩线，不能用单帧采样）。
        [DataInput]
        [Label("转身死区（度，只用来滤残余抖动，别调大）")]
        public float TurnDeadzoneDegrees = 1f;

        [DataInput]
        [Label("转身平滑（0=不平滑，越大越稳但越迟钝）")]
        public float TurnSmoothing = 0.2f;

        // 转身的诊断显示。
        //
        // 没有这一行就只能靠猜：发送端没发根旋转、基准还没建立、还是被死区吃掉了 ——
        // 三种情况的现象一模一样（模型就是不转身），上一轮就是这么白查了一轮。
        [DataInput]
        [Label("转身（收到的根偏航 / 施加到脊柱的量）")]
        public string TurnStatus = "(还没有收到根旋转)";

        private string turnStatusLast = "";
        private float turnAppliedLast;

        // 转身偏航的平滑状态。
        private float turnSmoothedYaw;
        private bool turnSmoothedValid;

        // ── 扭转分摊（FinalIK）：治"转动手掌时手腕/手肘被拧细" ────────────────
        //
        // 手工分摊（前臂/上臂分担滚转）当年被放弃，是因为它用 Atan2 提取扭转角，
        // 在 ±180° 附近会翻号 → 前臂来回弹、模型抽搐。这里换成 FinalIK 自己的实现：
        // 它与 Warudo 内置的 MediaPipe / LeapMotion 追踪器用的是同一个组件和同一套配置
        //（TwistSolver：transform = 前臂，parent = 上臂，children = [手]）。
        //
        // Relax() 的行为正是我们要的：它只把"前臂绕自身轴"的那一部分扭转摊掉，
        // 并把手的原世界旋转原样还原 —— 所以已经调好的手部朝向不会被破坏，
        // 被拧细的关节则由前臂一起承担。
        //
        // 必须在我们写完骨骼之后手动调用 Relax()：FinalIK 组件自己的 LateUpdate
        // 早于本插件的 OnPostLateUpdate，让它自己跑会被我们的写入覆盖掉。
        [DataInput]
        [Label("消除手腕扭曲（FinalIK 扭转分摊）")]
        public bool TwistRelaxArms = false;

        [DataInput]
        [Label("扭转分摊强度（0 = 不摊，1 = 全摊）")]
        public float TwistRelaxWeight = 1f;

        [DataInput]
        [Label("扭转分摊状态（手腕相对扭转：分摊前 → 分摊后）")]
        public string TwistStatus = "(还没有数据)";

        private TwistSolver twistLeft;
        private TwistSolver twistRight;
        private bool twistReady;

        // 根节点默认不碰。
        //
        // 直接写 Character.MainTransform 会把角色原点搬到髋关节上，结果是
        // "半截身子埋进地面 + 朝向不对"。Warudo 的根节点由动画系统管理
        //（Character.AnimationRootPosition / AnimationRootRotation 都是只读的），
        // 要经由它自己的叠加机制才能正确设置 —— 那是下一步的事。
        //
        // 关掉它不影响转身：髋关节的旋转本来就在骨骼列表里，靠骨骼链一样会转。
        //
        // （原「应用根位置/旋转」开关已废弃：旋转改为增量式自动处理，位置一律不碰。）

        // 两只手的骨骼绑定约定未必相同：模型右手实测是 local X = 手指、local Y = 掌背，
        // 而左手可能是绕某根轴反过来的（实测现象要求"手指轴取反"，而不是掌背轴）。
        // 这种事推理容易出错，所以每只手各给两个开关，由实测一次定死。
        [DataInput]
        [Label("用户左手：手指轴取反")]
        public bool NegFingersL = false;

        [DataInput]
        [Label("用户左手：掌背轴取反")]
        public bool NegPalmL = true;

        [DataInput]
        [Label("用户右手：手指轴取反")]
        public bool NegFingersR = false;

        [DataInput]
        [Label("用户右手：掌背轴取反")]
        public bool NegPalmR = false;

        // 只作用于"手指方向"的 z 取反。
        //
        // 实测：手心朝前、四指朝上（手指垂直于视线）时一切正确；但手指指向摄像头方向时，
        // 手指与掌心会一起反 180° —— 那正是 forward 由 z 主导的姿势，说明它的 z 符号是反的。
        // 而垂直姿势里 forward 的 z 为 0，取反不影响它。
        //
        // 掌背方向不能跟着翻：掌心在"垂直"姿势里正是靠 z 定位的，翻了会把correct的姿势弄坏。
        [DataInput]
        [Label("手指方向：Z 取反（手指指向镜头时才起作用）")]
        public bool FlipForwardZ = false;

        [DataInput]
        [Label("手部平滑（0=不平滑，越大越稳）")]
        public float HandSmoothing = 0.5f;

        [DataInput]
        [Label("已接收掌心帧")]
        public int PalmFrames;

        [DataInput]
        [Label("已写入手掌骨骼")]
        public int PalmWrites;

        private UdpClient listener;
        private bool bound;
        private string boundDescription = "";

        private int pendingBone = -1;
        private float qx, qy, qz, qw;
        private bool hasPending;

        private int receivedCount;
        /// <summary>收到的原始数据报数量（含被校验拒绝的）。用来区分"包没到"和"包到了但被丢弃"。</summary>
        private int rawCount;
        private string rawSample = "";
        /// <summary>最近一个包被拒绝的原因（空 = 通过）。</summary>
        private string rejectReason = "";
        private bool reportedNoBone;
        private string lastWorldLogged = "";
        /// <summary>最近一个手动探针包到达的时刻，用来判断它是否已经过期。</summary>
        private float lastProbeAt;
        // 掌心帧（36 字节，magic K2VH）与手动探针（20 字节）共用同一端口，按长度与 magic 区分。
        // 说明：写骨骼必须写 Character.Animator（画面用的角色）。
        // CharacterDaemon 是影子对象，只写它不会改变画面。
        private int appliedCount;
        private string lastSummary = "(还没有收到数据)";
        private string lastLogged = "";
        private string lastStatusPushed;

        private bool selfTestSent;
        private bool selfTestReported;
        private float selfTestSentAt;
        private float nextSelfTestAt;

        [Trigger]
        [Label("重新监听（改端口后用）")]
        public void RestartListener() {
            StartListening();
            selfTestSent = false;
            selfTestReported = false;
            nextSelfTestAt = 0f;
        }

        protected override void OnCreate() {
            base.OnCreate();
            SetActive(true);
            StartListening();
        }

        protected override void OnDestroy() {
            base.OnDestroy();
            StopListening();
        }

        private void StartListening() {
            StopListening();

            try {
                // 绑定到所有本机地址，而不是只绑 127.0.0.1。
                //
                // 之前绑 127.0.0.1 时，自检包（本进程发出）能收到，外部进程发来的却一个都收不到。
                // Warudo 内置的 VMC 接收器监听所有地址且工作正常，所以这里与它保持一致，
                // 以排除"绑定到特定回环地址"这一项差异。
                var address = new IPEndPoint(IPAddress.Any, Port);
                listener = new UdpClient(address);

                // 非阻塞：由帧循环主动轮询，不用后台线程。
                listener.Client.Blocking = false;

                bound = true;
                boundDescription = listener.Client.LocalEndPoint != null
                    ? listener.Client.LocalEndPoint.ToString()
                    : "(未知)";
                Status = "正在监听 " + boundDescription;
                Debug.Log("[BonePortProbe] " + Status);

                // 把真实的骨骼枚举值打进日志。
                //
                // 之前发送工具用的是 56/55，而这里的校验范围是 [0, LastBone)，
                // 于是所有外部包都被当成非法骨骼静默丢掉了 —— 表现为"收不到包"，
                // 实际是收到了却没通过校验。枚举值必须以运行时为准，不能凭记忆写。
                Debug.Log("[BonePortProbe] 骨骼枚举: LeftHand=" + (int)HumanBodyBones.LeftHand
                        + " RightHand=" + (int)HumanBodyBones.RightHand
                        + " LeftLowerArm=" + (int)HumanBodyBones.LeftLowerArm
                        + " RightLowerArm=" + (int)HumanBodyBones.RightLowerArm
                        + " LeftUpperArm=" + (int)HumanBodyBones.LeftUpperArm
                        + " RightUpperArm=" + (int)HumanBodyBones.RightUpperArm
                        + " Head=" + (int)HumanBodyBones.Head
                        + " Hips=" + (int)HumanBodyBones.Hips
                        + " Chest=" + (int)HumanBodyBones.Chest
                        + " LastBone=" + (int)HumanBodyBones.LastBone);
            } catch (Exception e) {
                bound = false;
                Status = "无法监听端口 " + Port + "：" + e.Message;
                Debug.LogError("[BonePortProbe] " + Status);
                try { listener?.Close(); } catch { }
                listener = null;
            }
        }

        private void StopListening() {
            bound = false;
            try { listener?.Close(); } catch { }
            listener = null;
        }

        /// <summary>把一个包解析进待应用状态。被拒绝时记下原因，避免再出现"静默丢弃"。</summary>
        private void Accept(byte[] data) {
            if (data == null || data.Length < 20) { rejectReason = "长度不足 20"; return; }

            int bone = BitConverter.ToInt32(data, 0);
            if (bone < 0 || bone >= (int)HumanBodyBones.LastBone) {
                rejectReason = "骨骼序号 " + bone + " 超出范围 [0," + (int)HumanBodyBones.LastBone + ")";
                return;
            }

            float x = BitConverter.ToSingle(data, 4);
            float y = BitConverter.ToSingle(data, 8);
            float z = BitConverter.ToSingle(data, 12);
            float w = BitConverter.ToSingle(data, 16);

            if (float.IsNaN(x) || float.IsNaN(y) || float.IsNaN(z) || float.IsNaN(w)) {
                rejectReason = "四元数含 NaN";
                return;
            }

            // 归一化：非单位四元数是一个"看起来合理但实际错误"的旋转，这里是最便宜的检查点。
            double len = Math.Sqrt((double)x * x + (double)y * y + (double)z * z + (double)w * w);
            if (len < 0.5 || len > 1.5) {
                rejectReason = "四元数长度 " + len.ToString("F3") + " 不在 [0.5, 1.5]";
                return;
            }

            rejectReason = "";

            pendingBone = bone;
            qx = (float)(x / len);
            qy = (float)(y / len);
            qz = (float)(z / len);
            qw = (float)(w / len);
            hasPending = true;
            receivedCount++;
            lastProbeAt = Time.realtimeSinceStartup;
            lastSummary = ((HumanBodyBones)bone) + " = ("
                + qx.ToString("F3") + ", " + qy.ToString("F3") + ", "
                + qz.ToString("F3") + ", " + qw.ToString("F3") + ")";
        }

        // ── 掌心帧 ──────────────────────────────────────────────────────────────
        //
        // 骨骼的局部轴已经实测确定：local X = 手指方向，local Y = 掌背法线，local Z = X × Y。
        // 所以只要拿到这两根方向，就能直接构造出骨骼应有的世界旋转：
        //     handBone.rotation = LookRotation(X × Y, Y)
        // 父级（前臂）的朝向、相机到角色的换算、VMC 的语义，全都不需要再猜。
        // 每只手各自保存 —— 只存一份的话，后到的那只手会把前一只覆盖掉，
        // 表现就是"两只手都在画面里时，只有一只手能动"。
        private struct PalmSlot {
            public bool Has;
            public bool Valid;
            public Vector3 Forward;
            public Vector3 Normal;
            public int Count;
            public int Applied;
        }

        private readonly PalmSlot[] palms = new PalmSlot[2];
        /// <summary>每只手上一帧的最终朝向，用于平滑。</summary>
        private readonly Quaternion[] palmSmoothed = { Quaternion.identity, Quaternion.identity };
        private readonly bool[] palmSmoothedValid = new bool[2];   // 0 = 用户的左手，1 = 用户的右手
        private int palmCountTotal;
        private int palmAppliedTotal;
        private string lastPalmLogged = "";
        private bool basisLogged;

        private static bool IsPalmFrame(byte[] data) {
            return data != null && data.Length >= 36
                && data[0] == (byte)'K' && data[1] == (byte)'2'
                && data[2] == (byte)'V' && data[3] == (byte)'H';
        }

        private void AcceptPalmFrame(byte[] data) {
            int slot = data[8] == (byte)'L' ? 0 : 1;
            palms[slot].Valid = data[9] == (byte)'1';
            palms[slot].Forward = new Vector3(
                BitConverter.ToSingle(data, 12),
                BitConverter.ToSingle(data, 16),
                BitConverter.ToSingle(data, 20));
            palms[slot].Normal = new Vector3(
                BitConverter.ToSingle(data, 24),
                BitConverter.ToSingle(data, 28),
                BitConverter.ToSingle(data, 32));
            palms[slot].Has = true;
            palms[slot].Count++;
            palmCountTotal++;
        }

        /// <summary>两只手都要应用：每帧各写各的骨骼，互不覆盖。</summary>
        /// <summary>把"模型到底朝哪"打成日志 —— 这一条我一直在假设，从没量过。</summary>
        private void LogCharacterBasis() {
            if (basisLogged) return;
            var visual = Character != null ? Character.Animator : null;
            if (visual == null) return;
            basisLogged = true;

            var root = Character.MainTransform;
            Debug.Log("[BonePortProbe] 角色基向量: forward=" + Fmt(root.forward)
                    + " right=" + Fmt(root.right) + " up=" + Fmt(root.up)
                    + " | 名字=" + root.name);

            var hips = visual.GetBoneTransform(HumanBodyBones.Hips);
            if (hips != null) {
                Debug.Log("[BonePortProbe] Hips 朝向: forward=" + Fmt(hips.forward)
                        + " | 父级=" + (hips.parent != null ? hips.parent.name : "null"));
            }

            // 手骨的父级链是谁，以及它现在朝哪 —— 我们写的是世界旋转，父级决定 localRotation。
            for (int i = 0; i < 2; i++) {
                var bone = i == 0 ? HumanBodyBones.RightHand : HumanBodyBones.LeftHand;
                var tr = visual.GetBoneTransform(bone);
                if (tr == null) { Debug.Log("[BonePortProbe] " + bone + " 骨骼不存在"); continue; }
                Debug.Log("[BonePortProbe] " + bone + " 父级=" + (tr.parent != null ? tr.parent.name : "null")
                        + " 祖父=" + (tr.parent != null && tr.parent.parent != null ? tr.parent.parent.name : "null")
                        + " | 世界坐标=" + Fmt(tr.position));
            }
        }

        private static string Fmt(Vector3 v) {
            return "(" + v.x.ToString("F2") + "," + v.y.ToString("F2") + "," + v.z.ToString("F2") + ")";
        }

        // ── 整套骨架（阶段1：全部走插件，不经过 VMC） ──────────────────────────
        //
        // 包格式（小端）：
        //   "K2VS" (4)
        //   uint32 counter (4)
        //   uint8  flags (1)             bit0 = 根位置有效, bit1 = 根旋转有效
        //   float rootPos.x/y/z (12)
        //   float rootRot.x/y/z/w (16)
        //   uint16 boneCount (2)
        //   每根骨骼： uint8 nameLen, name(UTF8), float x,y,z,w (16)
        //
        // 为什么发骨骼"名字"而不是序号：名字由 Warudo 自己解析成 HumanBodyBones，
        // 不依赖手抄的枚举表 —— 之前骨骼序号写错（55/56 实际是 17/18）就是手抄表的代价。
        private bool skeletonValid;
        /// <summary>最后一个骨架包到达的时刻。发送端一停，姿态就该放开，不能锁死。</summary>
        private float skeletonLastAt;
        private bool rootProbeLogged;

        // 主干（Character.MainTransform）的原始姿态，在第一次看到角色时记下来。
        //
        // 之前那版把主干直接搬到髋关节位置，热重载不会还原 —— 角色于是一直飘在半空中。
        // 有原值才能在关掉根节点时把它放回该在的位置。
        private bool spineMissingLogged;
        private bool turnLogged;
        private bool rootCaptured;
        private Vector3 rootOriginalPosition;
        private Quaternion rootOriginalRotation = Quaternion.identity;

        /// <summary>被视为"没有转身"的根旋转。第一帧收到骨架包时自动记录。</summary>
        private bool rootReferenceSet;
        private Quaternion rootReferenceRotation = Quaternion.identity;
        private int skeletonFrames;
        private bool skeletonHasRootPos;
        private bool skeletonHasRootRot;
        private Vector3 skeletonRootPos;
        private Quaternion skeletonRootRot = Quaternion.identity;
        private readonly System.Collections.Generic.List<System.Collections.Generic.KeyValuePair<string, Quaternion>> skeletonPose
            = new System.Collections.Generic.List<System.Collections.Generic.KeyValuePair<string, Quaternion>>();
        private readonly System.Collections.Generic.Dictionary<string, Transform> boneCache
            = new System.Collections.Generic.Dictionary<string, Transform>();

        private static bool IsSkeletonFrame(byte[] d) {
            return d != null && d.Length >= 26
                && d[0] == (byte)'K' && d[1] == (byte)'2'
                && d[2] == (byte)'V' && d[3] == (byte)'S';
        }

        private void AcceptSkeletonFrame(byte[] d) {
            try {
                int off = 8;

                // flags：bit0 = 根位置有效，bit1 = 根旋转有效。
                // 没有这一位的话，未校准时发送端只能填 0，角色会被硬拉到原点。
                byte flags = d[off++];
                skeletonHasRootPos = (flags & 1) != 0;
                skeletonHasRootRot = (flags & 2) != 0;

                skeletonRootPos = new Vector3(
                    BitConverter.ToSingle(d, off), BitConverter.ToSingle(d, off + 4),
                    BitConverter.ToSingle(d, off + 8));
                off += 12;
                skeletonRootRot = new Quaternion(
                    BitConverter.ToSingle(d, off), BitConverter.ToSingle(d, off + 4),
                    BitConverter.ToSingle(d, off + 8), BitConverter.ToSingle(d, off + 12));
                off += 16;

                int count = BitConverter.ToUInt16(d, off);
                off += 2;

                skeletonPose.Clear();
                bool needCacheRebuild = boneCache.Count == 0;
                for (int i = 0; i < count && off < d.Length; i++) {
                    int len = d[off++];
                    if (len <= 0 || off + len + 16 > d.Length) break;
                    string name = System.Text.Encoding.UTF8.GetString(d, off, len);
                    off += len;
                    var q = new Quaternion(
                        BitConverter.ToSingle(d, off), BitConverter.ToSingle(d, off + 4),
                        BitConverter.ToSingle(d, off + 8), BitConverter.ToSingle(d, off + 12));
                    off += 16;
                    skeletonPose.Add(
                        new System.Collections.Generic.KeyValuePair<string, Quaternion>(name, q));

                    // 出现没见过的骨骼名就重建缓存。
                    //
                    // 之前缓存只建一次：用户中途打开手指开关之后，新骨骼虽然一直在包里，
                    // 却永远不在缓存里，于是完全不会被应用 —— 现象是"打开了开关，手指还是不动"。
                    if (!boneCache.ContainsKey(name)) needCacheRebuild = true;
                }

                skeletonValid = true;
                skeletonLastAt = Time.realtimeSinceStartup;
                skeletonFrames++;

                // 骨骼名字解析一次就够，之后走缓存；出现新名字时重建。
                if (needCacheRebuild) {
                    boneCache.Clear();
                    var visual = Character != null ? Character.Animator : null;
                    if (visual != null) {
                        string missing = "";
                        string received = "";
                        foreach (var kv in skeletonPose) {
                            received += kv.Key + " ";
                            Transform t = null;
                            try {
                                var bone = (HumanBodyBones)Enum.Parse(typeof(HumanBodyBones), kv.Key);
                                t = visual.GetBoneTransform(bone);
                            } catch (Exception) { }

                            // 前缀兜底：发送端用的是短名（"LeftThumb"），
                            // 而 Unity 枚举里是全名（"LeftThumbProximal"）。
                            // 没有这一步，整根手指会因为命名差异而完全失效 ——
                            // 现象是"骨骼在列表里、数值也在变，模型的手指却不动"。
                            if (t == null && kv.Key.Length > 0) {
                                foreach (HumanBodyBones candidate in
                                         (HumanBodyBones[])Enum.GetValues(typeof(HumanBodyBones))) {
                                    if (!candidate.ToString().StartsWith(kv.Key, StringComparison.Ordinal)) continue;
                                    var found = visual.GetBoneTransform(candidate);
                                    if (found != null) { t = found; break; }
                                }
                            }

                            boneCache[kv.Key] = t;
                            if (t == null) missing += kv.Key + " ";
                        }
                        Debug.Log("[BonePortProbe] 骨架包已收到 " + skeletonPose.Count + " 根骨骼："
                                + received + (missing.Length > 0 ? "| 未找到: " + missing : "| 全部匹配"));
                    }
                }
            } catch (Exception e) {
                Debug.LogError("[BonePortProbe] 骨架包解析失败：" + e.Message);
            }
        }

        /// <summary>把整套骨架写进去。手骨随后由掌心帧覆盖 —— 所以这里不必排除手。</summary>
        private void ApplySkeleton() {
            if (!skeletonValid) return;

            // 超过 0.5 秒没有新包就放开：否则发送端停掉之后，角色会永远卡在最后一帧上。
            if (Time.realtimeSinceStartup - skeletonLastAt > 0.5f) return;

            var visual = Character != null ? Character.Animator : null;
            if (visual == null) return;

            // 第一次看到角色时，把主干的原始姿态记下来。
            if (!rootCaptured && Character.MainTransform != null) {
                rootCaptured = true;
                rootOriginalPosition = Character.MainTransform.position;
                rootOriginalRotation = Character.MainTransform.rotation;
                Debug.Log("[BonePortProbe] 已记录主干原始姿态: pos=" + Fmt(rootOriginalPosition)
                        + " rot=" + rootOriginalRotation.eulerAngles.ToString("F1"));
            }

            // ── 根节点 ──────────────────────────────────────────────────────────
            //
            // 直接写 MainTransform 是错的：那会把角色原点搬到髋关节上
            //（表现为"半截身子埋进地面 / 飘在半空 + 朝向不对"）。
            // Warudo 的根节点由动画系统管理，AnimationRootPosition / AnimationRootRotation
            // 都是只读的 —— 要经由它自己的叠加机制才谈得上正确，那是下一步的事。
            //
            // 但「转身」确实只存在于根旋转里（VMC 那条路就是靠根节点转的，
            // 髋关节骨骼并不携带转身），所以这里必须处理旋转，只处理旋转：
            //
            // 根节点：位置和旋转都不碰。
            //
            // 位置 —— 我们的根位置是"偏移量"语义（不移动时发送 0），而 Warudo 自己把主干
            // 放在髋高（y=0.80）并会自行移动它。把 0 当绝对位置写进去，角色原点就被搬到地面，
            // 人于是飘/埋。连"恢复原值"也不做：那会和 Warudo 自己的移动逻辑互相打架。
            //
            // 旋转 —— 转身量改放到脊柱上（见下面），所以这里也不需要动主干。
            if (skeletonHasRootRot && !rootReferenceSet) {
                rootReferenceSet = true;
                rootReferenceRotation = skeletonRootRot;
            }

            // 记录 Warudo 自己的根值，用来对齐语义（只打一次）。
            if (!rootProbeLogged && skeletonFrames > 30) {
                rootProbeLogged = true;
                Debug.Log("[BonePortProbe] Warudo 自己的根: pos=" + Fmt(Character.AnimationRootPosition)
                        + " rot=" + Character.AnimationRootRotation.eulerAngles.ToString("F1")
                        + " | 我们收到的: pos=" + Fmt(skeletonRootPos)
                        + " rot=" + skeletonRootRot.eulerAngles.ToString("F1")
                        + " | MainTransform.pos=" + Fmt(Character.MainTransform.position));
            }

            for (int i = 0; i < skeletonPose.Count; i++) {
                var kv = skeletonPose[i];
                Transform t;
                if (!boneCache.TryGetValue(kv.Key, out t) || t == null) continue;
                t.localRotation = kv.Value;

                // 手指骨骼的这份值就是"握拳姿态"（程序发来的死数据），
                // 供 MediaPipe 模式做插值端点。
                if (!fistCaptured && openPose.ContainsKey(kv.Key)) fistPose[kv.Key] = kv.Value;
            }

            ApplyTurnToSpine();

            // 两个都要调用：ApplyExtraBones 用 MediaPipe 发来的骨骼帧（两姿态混合），
            // ApplyMediaPipeFingers 用每根手指自己的弯曲量做逐指插值 —— 后者有端点时应该说了算，
            // 所以放在后面覆盖前者。
            //
            // 之前这里只留了 ApplyExtraBones：逐指插值整段代码从来没被执行过，
            // 现象就是"打开开关是两姿态混合（看着像死数据），关闭开关完全没有手指"。
            // 逐指插值（ApplyMediaPipeFingers）已从调用链移除：
            // 它在阈值/端点没配好时会把手指卡在半弯曲状态，比不做更糟。
            // 手指改由 hand_tracker.py 那边算好，插件只负责写 —— 见交接文档。
            ApplyExtraBones();
        }

        /// <summary>
        /// 把转身量放到脊柱上，而不是根节点。
        ///
        /// 为什么不是根节点：根节点一转，**整个角色**跟着转（含脚）。
        /// 为什么不是髋关节：**腿是髋的子节点**，转髋脚一样会转。
        /// 脊柱是髋以上第一根骨骼，转它 → 上半身转、脚不动。
        ///
        /// 转身量 = 我们收到的根旋转相对"参考朝向"的偏航增量。
        /// 用增量而不是绝对值：参考点代表"没有转身"，就不必知道角色默认朝向
        /// 与数据朝向之间差了多少度（实测两者差了 334°，绝对值直接写必然是错的）。
        /// </summary>
        private void ApplyTurnToSpine() {
            turnAppliedLast = 0f;
            if (!skeletonHasRootRot || !rootReferenceSet) return;

            float yaw = Mathf.DeltaAngle(
                rootReferenceRotation.eulerAngles.y, skeletonRootRot.eulerAngles.y);

            // 低通滤波：抑制残余的单帧抖动（平滑的是噪声，不是把转身抹掉）。
            if (!turnSmoothedValid) {
                turnSmoothedValid = true;
                turnSmoothedYaw = yaw;
            } else {
                // DeltaAngle 返回带符号的最短角差，用它做角度插值不会绕错方向。
                float a = Mathf.Clamp(TurnSmoothing, 0.02f, 1f);
                turnSmoothedYaw += a * Mathf.DeltaAngle(turnSmoothedYaw, yaw);
            }

            // 死区只滤零点几度的残余抖动。偏移量本身必须在发送端就不存在 ——
            // 靠在这里把它写成零来"看着不偏"，等于把转身功能废掉（实测：死区调到 10 度
            // 就是"转到一定角度才突然跳过去"）。
            float dead = Mathf.Max(0.1f, TurnDeadzoneDegrees);
            float applied = Mathf.Abs(turnSmoothedYaw) < dead ? 0f : turnSmoothedYaw;
            turnAppliedLast = applied;

            if (Mathf.Abs(applied) < 0.05f) return;

            Transform spine;
            if (!boneCache.TryGetValue("Spine", out spine) || spine == null) {
                if (!spineMissingLogged) {
                    spineMissingLogged = true;
                    Debug.LogWarning("[BonePortProbe] 找不到 Spine 骨骼，转身量无处安放。");
                }
                return;
            }

            // 以世界偏航叠加在脊柱当前朝向上；父级朝向用来换算回 localRotation。
            var parentRotation = spine.parent != null ? spine.parent.rotation : Quaternion.identity;
            var wantedWorld = Quaternion.Euler(0f, applied, 0f) * spine.rotation;
            spine.localRotation = Quaternion.Inverse(parentRotation) * wantedWorld;

            if (!turnLogged && Mathf.Abs(applied) > 5f) {
                turnLogged = true;
                Debug.Log("[BonePortProbe] 转身量已放到脊柱: " + applied.ToString("F1") + " 度");
            }
        }

        // ── 手指（MediaPipe 真实追踪） ─────────────────────────────────────────
        //
        // 只收"每根手指弯了多少"（0..1），不收骨骼旋转。
        //
        // 为什么这样做：弯曲轴是哪一根、符号朝哪边，对每个模型都不一样 ——
        // 手部朝向当初就是靠猜轴耗掉了十几轮。改成本地插值之后就不必猜了：
        //
        //   握拳姿态（程序发来的死数据，本来就在骨架包里）
        //   MediaPipe 给弯曲量 → 在两者之间插值
        //
        // 两个端点都由模型自己提供，中间怎么弯是模型自己的骨骼结构决定的。
        [DataInput]
        [Label("手指输出形式：☑原生（逐指） / ☐死数据（张开握拳）")]
        public bool UseMediaPipeFingers = false;

        [DataInput]
        [Label("每帧收到的手指骨骼数")]
        public int FingerFrames;

        public bool OpenCaptured;

        public bool FistCaptured;

        public string CurlSummary = "(还没有数据)";

        private static readonly string[] FingerNames =
            { "Thumb", "Index", "Middle", "Ring", "Little" };
        private static readonly string[] FingerParts =
            { "Proximal", "Intermediate", "Distal" };

        private readonly float[][] fingerCurls = { new float[5], new float[5] };
        private readonly bool[] fingerCurlValid = new bool[2];
        private readonly System.Collections.Generic.Dictionary<string, Quaternion> openPose
            = new System.Collections.Generic.Dictionary<string, Quaternion>(StringComparer.Ordinal);
        private readonly System.Collections.Generic.Dictionary<string, Quaternion> fistPose
            = new System.Collections.Generic.Dictionary<string, Quaternion>(StringComparer.Ordinal);
        private bool openPoseCaptured;
        private bool fistCaptured;

        /// <summary>
        /// 抓取"张手姿态"。
        ///
        /// 请在**手掌张开**时点这个按钮，并且先把程序里的手指开关**关掉** ——
        /// 那些开关会持续把死数据握拳写进手指骨，开着的时候抓到的就不是张手姿态。
        /// </summary>
        public void CaptureOpenHand() {
            var visual = Character != null ? Character.Animator : null;
            if (visual == null) {
                return;
            }

            openPose.Clear();
            int captured = 0;
            for (int f = 0; f < FingerNames.Length; f++) {
                foreach (string part in FingerParts) {
                    foreach (string side in new[] { "Left", "Right" }) {
                        var t = visual.GetBoneTransform(BoneOf(side + FingerNames[f] + part));
                        if (t == null) continue;
                        openPose[side + FingerNames[f] + part] = t.localRotation;
                        captured++;
                    }
                }
            }

            openPoseCaptured = captured > 0;
            Debug.Log("[BonePortProbe] 张手姿态已记录：" + captured + " 根手指骨骼"
                    + (openPoseCaptured ? "" : "（一根都没找到，检查模型的手指骨名）"));
        }

        /// <summary>抓取"握拳姿态"。请在**握拳**时点，这样就不必依赖程序里的死数据。</summary>
        public void CaptureFistHand() {
            var visual = Character != null ? Character.Animator : null;
            if (visual == null) {
                return;
            }

            int captured = CapturePoseInto(fistPose, visual);
            fistCaptured = captured > 0;
            Debug.Log("[BonePortProbe] 握拳姿态已记录：" + captured + " 根手指骨骼");
        }

        private static int CapturePoseInto(
                System.Collections.Generic.Dictionary<string, Quaternion> into, Animator visual) {
            into.Clear();
            int captured = 0;
            for (int f = 0; f < FingerNames.Length; f++) {
                foreach (string part in FingerParts) {
                    foreach (string side in new[] { "Left", "Right" }) {
                        string name = side + FingerNames[f] + part;
                        var t = visual.GetBoneTransform(BoneOf(name));
                        if (t == null) continue;
                        into[name] = t.localRotation;
                        captured++;
                    }
                }
            }
            return captured;
        }

        private static HumanBodyBones BoneOf(string name) {
            try {
                return (HumanBodyBones)Enum.Parse(typeof(HumanBodyBones), name);
            } catch (Exception) {
                return HumanBodyBones.LastBone;
            }
        }

        private static bool IsFingerFrame(byte[] d) {
            return d != null && d.Length >= 32
                && d[0] == (byte)'K' && d[1] == (byte)'2'
                && d[2] == (byte)'V' && d[3] == (byte)'F';
        }

        private void AcceptFingerFrame(byte[] d) {
            try {
                int slot = d[8] == (byte)'L' ? 0 : 1;
                fingerCurlValid[slot] = d[9] == (byte)'1';
                for (int i = 0; i < 5; i++) {
                    fingerCurls[slot][i] = BitConverter.ToSingle(d, 12 + i * 4);
                }
                FingerFrames++;
                fingerFrameLogged = true;
                CurlSummary = (slot == 0 ? "左 " : "右 ")
                    + fingerCurls[slot][0].ToString("F2") + " "
                    + fingerCurls[slot][1].ToString("F2") + " "
                    + fingerCurls[slot][2].ToString("F2") + " "
                    + fingerCurls[slot][3].ToString("F2") + " "
                    + fingerCurls[slot][4].ToString("F2");
            } catch (Exception) { }
        }

        /// <summary>
        /// 用 MediaPipe 的弯曲量在"张手"和"握拳"之间插值。最后应用，覆盖骨架包里的死数据。
        /// </summary>
        private void ApplyMediaPipeFingers() {
            if (!UseMediaPipeFingers) return;

            // 自动采集两个端点，不需要点按钮。
            //
            // 手动采集要求"腾出一只手按鼠标"，而采集本身又需要双手摆姿势 —— 那个流程没法用。
            // 改成自动：手全张开时存"张手"，全握紧时存"握拳"。用户只要自然地张合一次手。
            // 阈值按实测：张开 ≈ 0.25、握拳 ≈ 0.89，所以取 0.45 / 0.70 留足余量。
            for (int i = 0; i < 2; i++) {
                if (!fingerCurlValid[i]) continue;

                float sum = 0f;
                for (int f = 0; f < 5; f++) sum += Mathf.Clamp01(fingerCurls[i][f]);
                float average = sum / 5f;

                if (!openPoseCaptured && average < 0.45f) {
                    openPoseCaptured = CapturePoseInto(openPose, Character != null ? Character.Animator : null) > 0;
                    if (openPoseCaptured) {
                        OpenCaptured = true;
                        Debug.Log("[BonePortProbe] 已自动记录张开手（请再握一次拳）");
                    }
                } else if (!fistCaptured && average > 0.70f) {
                    fistCaptured = CapturePoseInto(fistPose, Character != null ? Character.Animator : null) > 0;
                    if (fistCaptured) {
                        FistCaptured = true;
                        Debug.Log("[BonePortProbe] 已自动记录握拳（手指开始逐指追踪）");
                    }
                }
            }

            if (!openPoseCaptured || !fistCaptured) return;

            for (int slot = 0; slot < 2; slot++) {
                if (!fingerCurlValid[slot]) continue;

                bool userLeft = slot == 0;
                string side = MirrorHands
                    ? (userLeft ? "Right" : "Left")
                    : (userLeft ? "Left" : "Right");

                for (int f = 0; f < FingerNames.Length; f++) {
                    float curl = Mathf.Clamp01(fingerCurls[slot][f]);

                    foreach (string part in FingerParts) {
                        string bone = side + FingerNames[f] + part;

                        Transform t;
                        if (!boneCache.TryGetValue(bone, out t) || t == null) continue;

                        Quaternion open;
                        if (!openPose.TryGetValue(bone, out open)) continue;

                        // 握拳姿态就是骨架包里程序发来的那一份（死数据）。
                        Quaternion fist;
                        if (!fistPose.TryGetValue(bone, out fist)) continue;

                        t.localRotation = Quaternion.Slerp(open, fist, curl);
                    }
                }
            }
        }

        // ── 骨骼帧（K2VB）：MediaPipe 那边算好的手指旋转，应用在最后 ────────────
        //
        // 复用 hand_tracker.py 里"本来发给 VMC"的那份手指骨骼旋转 —— 那套计算是验证过的，
        // 不需要任何姿态采集，也不需要猜弯曲轴。
        //
        // 开关作用：勾上「手指用 MediaPipe 追踪」= 用这份原生数据；
        //           不勾 = 用骨架包里程序发来的死数据握拳。
        private static readonly System.Collections.Generic.List<
            System.Collections.Generic.KeyValuePair<string, Quaternion>> extraBones
            = new System.Collections.Generic.List<
                System.Collections.Generic.KeyValuePair<string, Quaternion>>();

        private float extraBonesAt;

        // K2VN = 原生（每根手指各用自己的弯曲量混合）。两份都从 MediaPipe 来，
        // 开关只决定"应用哪一份" —— 插件不做任何判断。
        private static readonly System.Collections.Generic.List<
            System.Collections.Generic.KeyValuePair<string, Quaternion>> nativeBones
            = new System.Collections.Generic.List<
                System.Collections.Generic.KeyValuePair<string, Quaternion>>();
        private float nativeBonesAt;
        private bool boneFrameLogged;
        private bool fingerFrameLogged;
        private bool fingerFrameReported;

        private static bool IsBoneFrame(byte[] d) {
            return d != null && d.Length >= 10
                && d[0] == (byte)'K' && d[1] == (byte)'2'
                && d[2] == (byte)'V' && (d[3] == (byte)'B' || d[3] == (byte)'N');
        }

        private void AcceptBoneFrame(byte[] d) {
            try {
                // "K2VB"(4) + 计数(4) + 骨骼数(2) ⟹ 骨骼数在偏移 8。
                // 之前写成 6，读到的是错位的两个字节，得到一个大得离谱的数字，
                // 循环第一次就越界 —— 现象就是"一根骨骼都没解析出来、计数恒为 0"。
                int off = 8;
                int count = BitConverter.ToUInt16(d, off);
                off += 2;

                bool isNative = d[3] == (byte)'N';
                var target = isNative ? nativeBones : extraBones;
                target.Clear();
                for (int i = 0; i < count && off < d.Length; i++) {
                    int len = d[off++];
                    if (len <= 0 || off + len + 16 > d.Length) break;
                    string name = System.Text.Encoding.UTF8.GetString(d, off, len);
                    off += len;
                    var q = new Quaternion(
                        BitConverter.ToSingle(d, off), BitConverter.ToSingle(d, off + 4),
                        BitConverter.ToSingle(d, off + 8), BitConverter.ToSingle(d, off + 12));
                    off += 16;
                    target.Add(
                        new System.Collections.Generic.KeyValuePair<string, Quaternion>(name, q));
                }

                if (isNative) nativeBonesAt = Time.realtimeSinceStartup;
                else extraBonesAt = Time.realtimeSinceStartup;
                FingerFrames = target.Count;

                // 打一次日志：计数为 0 时，仅凭界面无法区分"包没到"和"解析出错"。
                if (!boneFrameLogged) {
                    boneFrameLogged = true;
                    Debug.Log("[BonePortProbe] 骨骼帧已收到：声明 " + count
                            + " 根，解析出 " + extraBones.Count + " 根"
                            + (extraBones.Count > 0 ? "，首根=" + extraBones[0].Key : ""));
                }
            } catch (Exception) { }
        }

        /// <summary>把 MediaPipe 的手指旋转写进去。最后应用，覆盖骨架包里的死数据。</summary>
        private void ApplyExtraBones() {
            // 开关只做一件事：选应用哪一份数据。两份都来自 MediaPipe。
            //   勾选 = K2VN 原生（每根手指各用自己的弯曲量）
            //   不勾 = K2VB 死数据（整体张开/握拳两个状态）
            // 判断全部在 hand_tracker.py 那边做完，这里没有任何阈值或采集状态 ——
            // 之前把插值放在这里，一旦端点没就绪就把手指卡在半弯曲，比不做更糟。
            var list = UseMediaPipeFingers ? nativeBones : extraBones;
            float stamp = UseMediaPipeFingers ? nativeBonesAt : extraBonesAt;

            if (list.Count == 0) return;

            // 超过 0.5 秒没有新帧就放手，避免手指卡在最后一帧。
            if (Time.realtimeSinceStartup - stamp > 0.5f) return;

            var visual = Character != null ? Character.Animator : null;
            if (visual == null) return;

            for (int i = 0; i < list.Count; i++) {
                var kv = list[i];
                Transform t;
                if (!boneCache.TryGetValue(kv.Key, out t) || t == null) {
                    // 骨架包里没出现过这个名字时现场解析一次（例如只开了手部识别、没开身体）。
                    var bone = BoneOf(kv.Key);
                    if (bone == HumanBodyBones.LastBone) continue;
                    t = visual.GetBoneTransform(bone);
                    if (t == null) continue;
                    boneCache[kv.Key] = t;
                }
                t.localRotation = kv.Value;
            }
        }

        private void ApplyPalmFrames() {
            LogCharacterBasis();
            for (int i = 0; i < palms.Length; i++) ApplyPalmSlot(i);
        }

        /// <summary>把一只手的掌心方向转成骨骼旋转并写入。</summary>
        private void ApplyPalmSlot(int slot) {
            if (!palms[slot].Has || !palms[slot].Valid) return;

            float sx = FlipCameraX ? -1f : 1f;
            float sy = FlipCameraY ? -1f : 1f;
            float sz = FlipCameraZ ? -1f : 1f;

            var f = new Vector3(
                palms[slot].Forward.x * sx, palms[slot].Forward.y * sy, palms[slot].Forward.z * sz);
            var n = new Vector3(
                palms[slot].Normal.x * sx, palms[slot].Normal.y * sy, palms[slot].Normal.z * sz);

            // 左右手分开处理：每只手可以独立翻转"手指轴"和"掌背轴"。
            // 翻转手指轴 = 绕掌背轴转 180°（手指反过来、掌心不变）；
            // 翻转掌背轴 = 绕手指轴转 180°（掌心反过来、手指不变）。
            bool negF = slot == 0 ? NegFingersL : NegFingersR;
            bool negN = slot == 0 ? NegPalmL : NegPalmR;
            if (FlipForwardZ) f = new Vector3(f.x, f.y, -f.z);
            if (negF) f = -f;
            if (negN) n = -n;

            if (f.sqrMagnitude < 1e-8f) return;
            f.Normalize();

            // 正交化：把法线里沿手指方向的分量去掉。不做这一步，构造出的矩阵不是纯旋转。
            n -= f * Vector3.Dot(n, f);
            if (n.sqrMagnitude < 1e-8f) return;
            n.Normalize();

            Vector3 s = Vector3.Cross(f, n);
            Quaternion world = Quaternion.LookRotation(s, n);

            // 手的朝向必须**跟随角色朝向**。
            //
            // 上面算出来的 world 是以"角色面向 +z"为前提的角色空间朝向，直接当成世界旋转写进骨骼，
            // 就等于把手的朝向钉死在场景坐标系上 —— 角色转身 180° 之后，手还朝着原来的世界方向，
            // 于是手背朝向用户（用户实测到的现象）。乘以角色朝向才是对的。
            var characterRotation = (Character != null && Character.MainTransform != null)
                ? Character.MainTransform.rotation
                : Quaternion.identity;
            world = characterRotation * world;

            // 平滑：掌心朝向是单目估计出来的，逐帧会有抖动。
            if (HandSmoothing > 0.001f) {
                float keep = Mathf.Clamp(HandSmoothing, 0f, 0.95f);
                if (!palmSmoothedValid[slot]) {
                    palmSmoothedValid[slot] = true;
                    palmSmoothed[slot] = world;
                }
                palmSmoothed[slot] = Quaternion.Slerp(world, palmSmoothed[slot], keep);
                world = palmSmoothed[slot];
            } else {
                palmSmoothedValid[slot] = true;
                palmSmoothed[slot] = world;
            }

            bool userLeft = slot == 0;
            // 手腕朝向与手臂、手指一致：不换手（用户左手 → 模型左手骨，用户右手 → 模型右手骨）。
            // 之前这里用 MirrorHands 把数据换手（用户右手 → 模型左手骨），
            // 于是只有手腕跑到另一边、而手臂和手指是对侧的 —— 也就是"手腕左右反了"。
            HumanBodyBones bone = userLeft ? HumanBodyBones.LeftHand : HumanBodyBones.RightHand;

            var visual = Character != null ? Character.Animator : null;
            if (visual == null) return;
            var target = visual.GetBoneTransform(bone);
            if (target == null) return;

            var forearm = target.parent;
            var upperArm = forearm != null ? forearm.parent : null;
            // 开启扭转分摊（FinalIK）时，手工分摊必须让位：两套机制同时改前臂只会打架，
            // 而且手工那套用 Atan2 提取扭转角，在 ±180° 附近会跳变（历史问题）。
            float share = TwistRelaxArms ? 0f : Mathf.Clamp(ForearmTwistShare, 0f, 0.9f);
            float upperShare = TwistRelaxArms ? 0f : Mathf.Clamp(UpperArmTwistShare, 0f, 0.9f);

            if (forearm != null) {
                // 相对旋转里绕"手指轴"的那一部分就是手腕的滚转。
                //
                // 全部压给手腕会出糖纸效应：手骨转半圈而前臂不动，蒙皮被拧细
                // —— 用户看到的就是"手腕扭曲变形变细"。真人做这个动作时前臂旋前/旋后
                // 承担了大部分，所以这里按比例分摊。
                Quaternion rel = Quaternion.Inverse(forearm.rotation) * world;
                float twist = 2f * Mathf.Atan2(rel.x, rel.w);

                // 限幅：Atan2 在 ±180° 附近翻号会让前臂来回弹（模型端抽动）。
                float limit = Mathf.Max(1f, TwistLimitDegrees);
                twist = Mathf.Clamp(twist * Mathf.Rad2Deg, -limit, limit) * Mathf.Deg2Rad;

                // 上臂先转：肘部跟着走，否则整圈滚转全卡在肘关节上（用户看到的"肘部扭曲"）。
                // Warudo 内置的 MediaPipe 之所以永不拧，是因为它走肢体 IK 让整条链一起解算；
                // 直接写骨骼没有那个机制，只能这样手工分摊。
                if (upperArm != null && upperShare > 0.001f) {
                    upperArm.rotation = upperArm.rotation * Quaternion.AngleAxis(
                        upperShare * twist * Mathf.Rad2Deg, Vector3.right);

                    // 上臂带动了前臂，手腕要按新的前臂朝向重算。
                    rel = Quaternion.Inverse(forearm.rotation) * world;
                    twist = 2f * Mathf.Atan2(rel.x, rel.w);
                }

                if (share > 0.001f) {
                    Quaternion twistPart = Quaternion.AngleAxis(
                        share * twist * Mathf.Rad2Deg, Vector3.right);
                    forearm.rotation = forearm.rotation * twistPart;
                    // 前臂多转的部分由手腕反向抵消，手的世界朝向仍然是 world。
                    target.localRotation = Quaternion.Inverse(twistPart) * rel;
                } else {
                    target.rotation = world;
                }
            } else {
                target.rotation = world;
            }

            palms[slot].Applied++;
            palmAppliedTotal++;

            if (fingerFrameLogged && !fingerFrameReported) {
                fingerFrameReported = true;
                Debug.Log("[BonePortProbe] 手指帧已收到：弯曲量 " + CurlSummary);
            }

            SetDataInput(nameof(FingerFrames), FingerFrames, broadcast: true);

            if (palmAppliedTotal % 30 == 0) {
                // 同时打印"MediaPipe 原始向量"与"换算后向量"。
                // 只看换算后的值，永远分不清是传入的数据不对，还是坐标换算不对。
                var rawF = palms[slot].Forward;
                var rawN = palms[slot].Normal;
                string line = (userLeft ? "左手" : "右手")
                    + " | 原始 手指(" + rawF.x.ToString("F2") + "," + rawF.y.ToString("F2") + "," + rawF.z.ToString("F2") + ")"
                    + " 掌背(" + rawN.x.ToString("F2") + "," + rawN.y.ToString("F2") + "," + rawN.z.ToString("F2") + ")"
                    + " | 换算 手指(" + f.x.ToString("F2") + "," + f.y.ToString("F2") + "," + f.z.ToString("F2") + ")"
                    + " 掌背(" + n.x.ToString("F2") + "," + n.y.ToString("F2") + "," + n.z.ToString("F2") + ")";
                if (line != lastPalmLogged) {
                    lastPalmLogged = line;
                    Debug.Log("[BonePortProbe] " + line);
                }
            }
        }

        /// <summary>把这一帧到达的所有包收干净，只保留最后一个（姿态保持）。</summary>
        private void DrainPackets() {            if (!bound || listener == null) return;

            int guard = 0;
            while (guard++ < 200) {
                int available;
                try {
                    available = listener.Available;
                } catch (Exception) {
                    return;
                }
                if (available <= 0) return;

                try {
                    var remote = new IPEndPoint(IPAddress.Any, 0);
                    byte[] data = listener.Receive(ref remote);

                    // 先记原始包，再交给校验。校验不过的包会计入 rawCount 但不计入 receivedCount，
                    // 于是"包没到"和"包到了却被丢掉"这两种情况就能分开。
                    rawCount++;
                    if (rawCount <= 3 && data != null) {
                        var sb = new System.Text.StringBuilder();
                        for (int i = 0; i < data.Length && i < 20; i++) sb.Append(data[i].ToString("X2")).Append(' ');
                        rawSample = "长度=" + data.Length + " 内容=" + sb.ToString().Trim()
                                  + " 来自=" + remote;
                    }

                    if (IsBoneFrame(data)) {
                        AcceptBoneFrame(data);
                    } else if (IsFingerFrame(data)) {
                        AcceptFingerFrame(data);
                    } else if (IsSkeletonFrame(data)) {
                        AcceptSkeletonFrame(data);
                    } else if (IsPalmFrame(data)) {
                        AcceptPalmFrame(data);
                    } else {
                        Accept(data);
                    }
                } catch (SocketException) {
                    return;
                } catch (ObjectDisposedException) {
                    return;
                } catch (Exception) {
                    return;
                }
            }
        }

        /// <summary>向自己发一个包，验证回环收发是否正常。</summary>
        private void SendSelfTestPacket() {
            try {
                using (var probe = new UdpClient()) {
                    byte[] packet = new byte[20];
                    Buffer.BlockCopy(BitConverter.GetBytes((int)HumanBodyBones.RightHand), 0, packet, 0, 4);
                    Buffer.BlockCopy(BitConverter.GetBytes(0f), 0, packet, 4, 4);
                    Buffer.BlockCopy(BitConverter.GetBytes(0f), 0, packet, 8, 4);
                    Buffer.BlockCopy(BitConverter.GetBytes(0f), 0, packet, 12, 4);
                    Buffer.BlockCopy(BitConverter.GetBytes(1f), 0, packet, 16, 4);
                    probe.Send(packet, packet.Length, "127.0.0.1", Port);
                }
                selfTestSentAt = Time.realtimeSinceStartup;
                Debug.Log("[BonePortProbe] 自检包已发出 -> 127.0.0.1:" + Port);
            } catch (Exception e) {
                selfTestReported = true;
                Debug.LogError("[BonePortProbe] 自检包发送失败：" + e.Message);
            }
        }

        private void LogThrottled(string where) {
            string summary = where + " | 原始包 " + rawCount + " | 有效包 " + receivedCount
                           + " | 写入 " + appliedCount + " 次"
                           + (rejectReason.Length > 0 ? (" | 最近被拒: " + rejectReason) : "")
                           + (rawSample.Length > 0 ? (" | 样本: " + rawSample) : "");
            if (summary == lastLogged) return;
            lastLogged = summary;
            Debug.Log("[BonePortProbe] " + summary);
        }

        public override void OnPostLateUpdate() {
            base.OnPostLateUpdate();

            if (Status != lastStatusPushed) {
                lastStatusPushed = Status;
                SetDataInput(nameof(Status), Status, broadcast: true);
            }

            DrainPackets();

            // 自检：绑定后延迟一小段再发，避免和启动时的帧竞争。
            if (bound && !selfTestSent && Time.realtimeSinceStartup > 1f) {
                selfTestSent = true;
                SendSelfTestPacket();
            }

            if (selfTestSent && !selfTestReported && receivedCount > 0) {
                selfTestReported = true;
                Debug.Log("[BonePortProbe] 自检通过：回环收发正常（收到 " + receivedCount + " 个包）");
            }

            if (selfTestSent && !selfTestReported
                && Time.realtimeSinceStartup - selfTestSentAt > 3f) {
                selfTestReported = true;
                Debug.LogError("[BonePortProbe] 自检失败：连自己发的包都收不到 → 问题在套接字/回环这一侧");
            }

            // 周期性自检：验证套接字"持续"在收货，而不是只收到启动那一个包。
            //
            // 这一步很关键：如果连每 3 秒重复的自检包都收不到（计数停在 1 不动），
            // 那问题在接收这一侧，与外部发送端无关；如果计数在涨，则接收正常，
            // 问题就确定在"外部进程的包进不来"。
            if (bound && Time.realtimeSinceStartup >= nextSelfTestAt) {
                nextSelfTestAt = Time.realtimeSinceStartup + 3f;
                SendSelfTestPacket();
            }

            LogThrottled("OnPostLateUpdate");

            // 先应用 20 字节的手动探针，再应用掌心帧 —— 掌心帧是真正驱动手的一方，
            // 必须最后写，否则会被探针残留的姿态覆盖。
            // 顺序：整套骨架 → 手动探针 → 掌心帧。后面的覆盖前面的，手骨最终由掌心帧决定。
            ApplySkeleton();
            ApplyProbeIfFresh();
            ApplyPalmFrames();
            UpdateTurnStatus();

            // 扭转分摊放最后：它读的是"这一帧最终的"世界姿态（骨架 + 掌心帧 + 转身都写完了），
            // 放在前面会被后面的写入覆盖。
            ApplyTwistRelax();

            if (fingerFrameLogged && !fingerFrameReported) {
                fingerFrameReported = true;
                Debug.Log("[BonePortProbe] 手指帧已收到：弯曲量 " + CurlSummary);
            }

            SetDataInput(nameof(FingerFrames), FingerFrames, broadcast: true);

            if (palmAppliedTotal % 30 == 0) {
                SetDataInput(nameof(PalmFrames), palmCountTotal, broadcast: true);
                SetDataInput(nameof(PalmWrites), palmAppliedTotal, broadcast: true);
                PalmFrames = palmCountTotal;
                PalmWrites = palmAppliedTotal;
            }
        }

        /// <summary>
        /// 把转身的状态显示在面板上。
        ///
        /// 这一行是"上一轮白查"的直接产物：不转身有三种完全相同的现象 ——
        /// 发送端没发根旋转、收到了但插件还没建立基准、或者偏航被死区吃掉了。
        /// 面板上写清楚是哪一种，就不用再去翻日志猜。
        /// </summary>
        private void UpdateTurnStatus() {
            // 半秒左右刷新一次就够，避免每帧推送面板数据。
            if (skeletonFrames % 15 != 0) return;

            string status;
            if (!skeletonHasRootRot) {
                status = "发送端没有发根旋转（模型不会转身）";
            } else if (!rootReferenceSet) {
                status = "已收到根旋转，基准未建立";
            } else {
                status = "收到 " + skeletonRootRot.eulerAngles.y.ToString("F1") + " 度"
                       + " | 施加到脊柱 " + turnAppliedLast.ToString("F1") + " 度";
            }

            if (status == turnStatusLast) return;
            turnStatusLast = status;
            TurnStatus = status;
            SetDataInput(nameof(TurnStatus), status, broadcast: true);
        }

        // ── 扭转分摊（FinalIK TwistSolver） ────────────────────────────────────

        /// <summary>
        /// 建立两只前臂的扭转分摊器。
        ///
        /// 骨骼与 Warudo 自家追踪器一致：
        ///   左：transform = LeftLowerArm  parent = LeftUpperArm  children = [LeftHand]
        ///   右：transform = RightLowerArm parent = RightUpperArm children = [RightHand]
        /// </summary>
        private void EnsureTwistRelaxers() {
            if (twistReady) return;

            var visual = Character != null ? Character.Animator : null;
            if (visual == null) return;

            Transform lowerL = visual.GetBoneTransform(HumanBodyBones.LeftLowerArm);
            Transform upperL = visual.GetBoneTransform(HumanBodyBones.LeftUpperArm);
            Transform handL = visual.GetBoneTransform(HumanBodyBones.LeftHand);
            Transform lowerR = visual.GetBoneTransform(HumanBodyBones.RightLowerArm);
            Transform upperR = visual.GetBoneTransform(HumanBodyBones.RightUpperArm);
            Transform handR = visual.GetBoneTransform(HumanBodyBones.RightHand);

            if (lowerL == null || upperL == null || handL == null) return;
            if (lowerR == null || upperR == null || handR == null) return;

            twistLeft = new TwistSolver();
            twistLeft.transform = lowerL;
            twistLeft.parent = upperL;
            twistLeft.children = new[] { handL };
            twistLeft.Initiate();

            twistRight = new TwistSolver();
            twistRight.transform = lowerR;
            twistRight.parent = upperR;
            twistRight.children = new[] { handR };
            twistRight.Initiate();

            twistReady = true;
            Debug.Log("[BonePortProbe] 扭转分摊已建立：前臂←上臂，手→前臂");
        }

        /// <summary>
        /// 把手腕上集中的扭转摊一部分给前臂。
        ///
        /// 必须由我们自己调用，而且要在这帧所有骨骼写入之后：FinalIK 组件自身的
        /// LateUpdate 早于本插件的 OnPostLateUpdate，让它自己跑会被我们的写入覆盖。
        /// </summary>
        private void ApplyTwistRelax() {
            if (!TwistRelaxArms) {
                // 关掉之后重新开启时重建：中途换模型/换骨骼也不会用错引用。
                twistReady = false;
                PushTwistStatus("已关闭（使用手工分摊）");
                return;
            }

            EnsureTwistRelaxers();
            if (!twistReady) {
                PushTwistStatus("找不到前臂/上臂/手骨骼，无法分摊");
                return;
            }

            float weight = Mathf.Clamp01(TwistRelaxWeight);
            twistLeft.weight = weight;
            twistRight.weight = weight;

            var visual = Character != null ? Character.Animator : null;
            Transform fl = visual != null ? visual.GetBoneTransform(HumanBodyBones.LeftLowerArm) : null;
            Transform hl = visual != null ? visual.GetBoneTransform(HumanBodyBones.LeftHand) : null;
            Transform fr = visual != null ? visual.GetBoneTransform(HumanBodyBones.RightLowerArm) : null;
            Transform hr = visual != null ? visual.GetBoneTransform(HumanBodyBones.RightHand) : null;

            // 分摊前后各量一次"手腕相对扭转"，把效果做成可读的数字 ——
            // 不然只能靠肉眼判断"好像没那么拧了"。
            float beforeL = RelativeTwistDegrees(fl, hl);
            float beforeR = RelativeTwistDegrees(fr, hr);

            twistLeft.Relax();
            twistRight.Relax();

            float afterL = RelativeTwistDegrees(fl, hl);
            float afterR = RelativeTwistDegrees(fr, hr);

            if (skeletonFrames % 30 == 0) {
                PushTwistStatus("强度 " + (weight * 100f).ToString("F0") + "%"
                    + " | 手腕相对扭转 左 " + beforeL.ToString("F0") + "°→" + afterL.ToString("F0") + "°"
                    + " 右 " + beforeR.ToString("F0") + "°→" + afterR.ToString("F0") + "°");
            }
        }

        /// <summary>
        /// 前臂与手之间"绕前臂自身轴"的相对扭转角（度）。
        ///
        /// 用 swing-twist 投影，不用 Atan2(rel.x, rel.w)：后者在 ±180° 附近会翻号跳变，
        /// 这正是当年手工分摊方案抽搐的原因。
        /// </summary>
        private static float RelativeTwistDegrees(Transform forearm, Transform hand) {
            if (forearm == null || hand == null) return 0f;

            Vector3 axis = hand.position - forearm.position;
            if (axis.sqrMagnitude < 1e-10f) return 0f;
            axis = forearm.InverseTransformDirection(axis.normalized);

            Quaternion rel = Quaternion.Inverse(forearm.rotation) * hand.rotation;
            Vector3 vector = new Vector3(rel.x, rel.y, rel.z);
            float along = Vector3.Dot(vector, axis);
            return 2f * Mathf.Atan2(along, rel.w) * Mathf.Rad2Deg;
        }

        private void PushTwistStatus(string text) {
            if (text == TwistStatus) return;
            TwistStatus = text;
            SetDataInput(nameof(TwistStatus), text, broadcast: true);
        }

        /// <summary>
        /// 应用最近收到的手动探针包。
        ///
        /// 只在一秒内收到过包时才写：hasPending 一旦置位就不会清除（姿态要保持），
        /// 若不判断新鲜度，探针用过一次之后会永远每帧覆盖同一根骨骼，
        /// 把掌心帧的结果盖掉。
        /// </summary>
        private void ApplyProbeIfFresh() {
            if (!hasPending) return;
            if (Time.realtimeSinceStartup - lastProbeAt > 1f) return;

            var rotation = new Quaternion(qx, qy, qz, qw);
            var bone = (HumanBodyBones)pendingBone;

            // 关键：必须写"真正显示的那个角色"。
            //
            // CharacterDaemon 是一个影子对象（官方示例里先把姿势写进守护对象，
            // 再在 OnPostLateUpdate 里把它复制回 Character.Animator）。只写守护对象
            // 不会改变画面 —— 这正是之前"包收到了、骨骼写入次数在涨、模型却不动"的原因。
            //
            // 两边都写：Character.Animator 负责画面，守护对象保持同步以免内部状态不一致。
            var visual = Character != null ? Character.Animator : null;
            var visualBone = visual != null ? visual.GetBoneTransform(bone) : null;

            var daemonBone = (CharacterDaemon != null && CharacterDaemon.IsCreated())
                ? CharacterDaemon.Animator.GetBoneTransform(bone)
                : null;

            if (visualBone == null && daemonBone == null) {
                if (!reportedNoBone) {
                    reportedNoBone = true;
                    Debug.LogWarning("[BonePortProbe] 找不到骨骼 " + bone
                        + "；角色动画器=" + (visual == null ? "null" : visual.name));
                }
                return;
            }

            if (visualBone != null) visualBone.localRotation = rotation;
            if (daemonBone != null) daemonBone.localRotation = rotation;

            appliedCount++;

            // 记录骨骼的世界坐标：姿态变了它就会变，
            // 这样不必看画面也能判断写入是否真的产生了效果。
            if (appliedCount % 30 == 0) {
                string world = visualBone != null
                    ? visualBone.position.x.ToString("F4") + ", "
                      + visualBone.position.y.ToString("F4") + ", "
                      + visualBone.position.z.ToString("F4")
                    : "(无显示骨骼)";

                SetDataInput(nameof(Received), receivedCount, broadcast: true);
                SetDataInput(nameof(Applied), appliedCount, broadcast: true);
                SetDataInput(nameof(Last), lastSummary, broadcast: true);
                Last = lastSummary;
                Received = receivedCount;
                Applied = appliedCount;

                string line = "写入 " + bone + " | 显示骨骼世界坐标 " + world;
                if (line != lastWorldLogged) {
                    lastWorldLogged = line;
                    Debug.Log("[BonePortProbe] " + line);
                }
            }
        }
    }
}

