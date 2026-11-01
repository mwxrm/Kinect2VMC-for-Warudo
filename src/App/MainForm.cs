using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Text;
using System.Windows.Forms;
using Kinect2VMC.Core;

namespace Kinect2VMC.App
{
    /// <summary>
    /// The control panel. Built entirely in code: this environment has no XAML build support,
    /// and WinForms needs no designer to be maintainable at this size.
    /// </summary>
    public sealed class MainForm : Form
    {
        private readonly AppSettings _settings = new AppSettings();
        private readonly TrackingPipeline _pipeline;
        private readonly Timer _refreshTimer = new Timer();

        private readonly Label _stateValue = new Label();
        private readonly Label _deviceValue = new Label();
        private readonly Label _bodyValue = new Label();
        private readonly Label _rateValue = new Label();
        private readonly Label _sendValue = new Label();
        private readonly Label _calibValue = new Label();
        private readonly Label _messageValue = new Label();
        private readonly Label _bonesValue = new Label();

        private readonly TextBox _addressBox = new TextBox();
        private readonly NumericUpDown _portBox = new NumericUpDown();
        private readonly TrackBar _smoothingBar = new TrackBar();
        private readonly Label _smoothingValue = new Label();
        private readonly CheckBox _sendHeadBox = new CheckBox();
        private readonly CheckBox _sendRootBox = new CheckBox();
        private readonly CheckBox _mirrorBox = new CheckBox();
        private readonly NumericUpDown _limitBox = new NumericUpDown();
        private readonly NumericUpDown _tiltBox = new NumericUpDown();
        private readonly ComboBox _armPoseBox = new ComboBox();
        private readonly CheckBox _worldRotationBox = new CheckBox();
        private readonly CheckBox _handLogBox = new CheckBox();
        private readonly CheckBox _fistBox = new CheckBox();
        private readonly CheckBox _handBox = new CheckBox();

        private readonly Button _handStartButton = new Button();
        private readonly Button _handStopButton = new Button();
        private readonly ComboBox _handPreviewBox = new ComboBox();
        private readonly CheckBox _relayBox = new CheckBox();
        private readonly CheckBox _cameraHandBox = new CheckBox();
        private readonly CheckBox _pluginSkeletonBox = new CheckBox();
        private readonly Label _palmStatusValue = new Label();
        private readonly CheckBox _absoluteHandBox = new CheckBox();
        private readonly CheckBox _mirrorHandBox = new CheckBox();
        private readonly CheckBox _divideParentBox = new CheckBox();
        private readonly ComboBox _handAxisBox = new ComboBox();
        private readonly ComboBox[] _handAxisChoosers = new ComboBox[3];
        private readonly CheckBox[] _handSignBoxes = new CheckBox[3];
        private readonly CheckBox[] _mirrorPlaneBoxes = new CheckBox[3];
        private readonly Label _handStatusValue = new Label();
        private readonly HandTrackerProcess _handTracker = new HandTrackerProcess();
        private readonly CheckBox _orientationBox = new CheckBox();
        private readonly CheckBox _fullBodyBox = new CheckBox();
        private readonly CheckBox _rootRotationBox = new CheckBox();
        private readonly CheckBox _curlInwardBox = new CheckBox();
        private readonly Label _handValue = new Label();

        private readonly Button _startStopButton = new Button();
        private readonly Button _zeroButton = new Button();
        private readonly Button _calibrateButton = new Button();
        private readonly Button _clearCalibrationButton = new Button();
        private readonly Button _saveButton = new Button();
        private readonly Button _diagnosticsButton = new Button();

        private readonly SkeletonPreview _preview = new SkeletonPreview();
        private readonly ListBox _rotationList = new ListBox();

        private bool _loadingSettings = true;

        public MainForm()
        {
            _pipeline = new TrackingPipeline(_settings);

            Text = "Kinect 2.0 → VMC → Warudo（上半身动捕）";
            MinimumSize = new Size(880, 620);
            Size = new Size(940, 660);
            StartPosition = FormStartPosition.CenterScreen;
            Font = new Font("Microsoft YaHei UI", 9f);

            BuildLayout();
            LoadSettings();

            _loadingSettings = false;
            PushSettingsToPipeline();

            // 33 ms matches the 30 Hz body frame rate, so the skeleton preview and the readout
            // track the sensor instead of visibly lagging behind it. The earlier 200 ms was
            // purely cosmetic and made the preview look like the tracking itself was slow.
            _refreshTimer.Interval = 33;
            _refreshTimer.Tick += (s, e) => RefreshUi();
            _refreshTimer.Start();
        }

        // ------------------------------------------------------------------ layout

        private void BuildLayout()
        {
            var root = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                Padding = new Padding(10)
            };
            root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 52f));
            root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 48f));

            root.Controls.Add(BuildLeftColumn(), 0, 0);
            root.Controls.Add(BuildRightColumn(), 1, 0);
            Controls.Add(root);
        }

        private Control BuildLeftColumn()
        {
            var stack = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoScroll = true
            };

            stack.Controls.Add(SectionLabel("设备状态"));
            var status = new TableLayoutPanel { ColumnCount = 2, AutoSize = true, Width = 430 };
            status.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 110f));
            status.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 310f));
            AddStatusRow(status, "运行状态", _stateValue);
            AddStatusRow(status, "Kinect", _deviceValue);
            AddStatusRow(status, "人体", _bodyValue);
            AddStatusRow(status, "帧率", _rateValue);
            AddStatusRow(status, "已发骨骼", _bonesValue);
            AddStatusRow(status, "发包数", _sendValue);
            AddStatusRow(status, "旋转基准", _calibValue);
            AddStatusRow(status, "手部姿态", _handValue);
            stack.Controls.Add(status);

            _messageValue.AutoSize = false;
            _messageValue.Width = 420;
            _messageValue.Height = 40;
            _messageValue.ForeColor = Color.DimGray;
            stack.Controls.Add(_messageValue);

            stack.Controls.Add(SectionLabel("控制"));
            var buttons = new FlowLayoutPanel { AutoSize = true, WrapContents = true, Width = 430 };

            _startStopButton.Text = "开始追踪";
            _startStopButton.Width = 110;
            _startStopButton.Height = 32;
            _startStopButton.Click += (s, e) => ToggleRunning();
            buttons.Controls.Add(_startStopButton);

            _zeroButton.Text = "位移归零";
            _zeroButton.Width = 90;
            _zeroButton.Height = 32;
            _zeroButton.Click += (s, e) => _pipeline.RequestZeroPosition();
            buttons.Controls.Add(_zeroButton);

            // Kept separate from 位移归零 on purpose: this changes the ROTATION reference, which
            // trades one arm motion plane against another. See the tooltip.
            _calibrateButton.Text = "基准改为当前坐姿";
            _calibrateButton.Width = 150;
            _calibrateButton.Height = 32;
            _calibrateButton.Click += (s, e) => _pipeline.RequestCalibration();
            var calibrateTip = new ToolTip();
            calibrateTip.SetToolTip(_calibrateButton,
                "默认基准是「水平 T 字」，适合手臂前伸/侧展。\r\n" +
                "若你的手臂主要在身体前后摆动，可点此把基准改为你当前的坐姿。\r\n" +
                "用「清除基准」可以改回 T 字。");
            buttons.Controls.Add(_calibrateButton);

            _clearCalibrationButton.Text = "清除基准";
            _clearCalibrationButton.Width = 90;
            _clearCalibrationButton.Height = 32;
            _clearCalibrationButton.Click += (s, e) => _pipeline.ClearCalibration();
            buttons.Controls.Add(_clearCalibrationButton);

            _saveButton.Text = "保存设置";
            _saveButton.Width = 90;
            _saveButton.Height = 32;
            _saveButton.Click += (s, e) => SaveSettings(true);
            buttons.Controls.Add(_saveButton);

            // The first real run always happens on the user's machine, so make it trivial to
            // capture everything needed to explain a failure.
            _diagnosticsButton.Text = "导出诊断报告";
            _diagnosticsButton.Width = 110;
            _diagnosticsButton.Height = 32;
            _diagnosticsButton.Click += (s, e) => WriteDiagnostics();
            buttons.Controls.Add(_diagnosticsButton);

            stack.Controls.Add(buttons);

            // ---- hand recognition: one button, and one choice of window ----------------------
            //
            // The hand tracker is a separate program, and starting it by hand means opening a
            // terminal, remembering the interpreter's full path (the `python3` on PATH is a Store
            // stub that does nothing) and typing arguments. All of that is the same every time, so
            // it belongs behind a button.
            //
            // The window choice exists because the preview shows the user's camera. While streaming,
            // or while recording, that must not appear -- but the window is also the only way to
            // quit the script cleanly and toggle mirroring, so a BLACK window is offered as well as
            // no window at all.
            stack.Controls.Add(SectionLabel("手部识别"));
            var hand = new FlowLayoutPanel { AutoSize = true, WrapContents = true, Width = 430 };

            _handStartButton.Text = "启动手部识别";
            _handStartButton.Width = 120;
            _handStartButton.Height = 32;
            _handStartButton.Click += (s, e) => ToggleHandTracker();
            hand.Controls.Add(_handStartButton);

            _handStopButton.Text = "停止";
            _handStopButton.Width = 70;
            _handStopButton.Height = 32;
            _handStopButton.Enabled = false;
            _handStopButton.Click += (s, e) => StopHandTracker();
            hand.Controls.Add(_handStopButton);

            _handPreviewBox.Text = "预览窗口";
            _handPreviewBox.Width = 110;
            _handPreviewBox.DropDownStyle = ComboBoxStyle.DropDownList;
            _handPreviewBox.Items.AddRange(new object[] { "纯黑（可按键）", "不显示", "显示画面" });
            _handPreviewBox.SelectedIndex = 0;
            var previewTip = new ToolTip();
            previewTip.SetToolTip(_handPreviewBox,
                "纯黑：只弹出一个小黑窗口，画面不外泄，但 q/m/d 按键仍然可用（默认）。\r\n" +
                "不显示：完全不弹窗口，此时只能用「停止」按钮结束。\r\n" +
                "显示画面：正常的摄像头预览，用于调试。");
            hand.Controls.Add(_handPreviewBox);

            stack.Controls.Add(hand);
            // Kinect colour as the image source.
            //
            // The laptop webcam is unreliable, and Warudo's face capture may be holding it already.
            // Only this program can open the Kinect, so the frames are forwarded to the tracker over
            // loopback instead. The Kinect's field of view is wide, so the hand is smaller in frame
            // than a webcam would show it -- worth trying rather than assuming.
            var relayRow = new FlowLayoutPanel { AutoSize = true, WrapContents = true, Width = 430 };
            _relayBox.Text = "用 Kinect 彩色画面（摄像头坏了或不方便时）";
            _relayBox.Width = 320;
            _relayBox.CheckedChanged += (s, e) => PushSettingsToPipeline();
            var relayTip = new ToolTip();
            relayTip.SetToolTip(_relayBox,
                "勾选后由本程序把 Kinect 的彩色画面转发给手部识别，\r\n" +
                "不再使用笔记本摄像头。\r\n" +
                "注意：Kinect 是广角镜头，手在画面里会偏小。");
            relayRow.Controls.Add(_relayBox);
            stack.Controls.Add(relayRow);

            // 整套骨架直接走插件（不经 VMC）。
            //
            // 默认关闭：这条通路刚建立，要先和 VMC 对照。打开后建议同时停掉 VMC 发送 ——
            // 同一根骨骼有两个写入者时，谁后写谁生效，IK 尤其抢不过。
            var pluginRow = new FlowLayoutPanel { AutoSize = true, WrapContents = true, Width = 430 };
            _pluginSkeletonBox.Text = "整套骨架直接走插件（不用 VMC）";
            _pluginSkeletonBox.Width = 300;
            _pluginSkeletonBox.CheckedChanged += (s, e) => PushSettingsToPipeline();
            var pluginTip = new ToolTip();
            pluginTip.SetToolTip(_pluginSkeletonBox,
                "把身体和手臂也交给 Warudo 插件直接写骨骼，不再经 VMC。\r\n" +
                "目的：让骨骼只有一个写入者，之后上 IK 才不会被 VMC 抢写。\r\n" +
                "开启后请把 VMC 那条路停掉对照验证。");
            pluginRow.Controls.Add(_pluginSkeletonBox);
            stack.Controls.Add(pluginRow);
            stack.Controls.Add(SectionLabel("网络（Warudo 的 VMC 接收器）"));
            var net = new TableLayoutPanel { ColumnCount = 2, AutoSize = true, Width = 430 };
            net.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 110f));
            net.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 310f));
            AddStatusRow(net, "目标地址", _addressBox);
            AddStatusRow(net, "端口", _portBox);
            _portBox.Minimum = 1;
            _portBox.Maximum = 65535;
            _portBox.Width = 100;
            _addressBox.Width = 160;
            stack.Controls.Add(net);

            stack.Controls.Add(SectionLabel("追踪参数"));
            var tuning = new TableLayoutPanel { ColumnCount = 2, AutoSize = true, Width = 430 };
            tuning.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 110f));
            tuning.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 310f));

            // Rows for the UNFINISHED features. They are populated here as the controls are built,
            // but placed on the form much later under their own heading, so a working setting and a
            // half-verified one are never presented as if they were the same thing.
            var experimental = new TableLayoutPanel { ColumnCount = 2, AutoSize = true, Width = 430 };
            experimental.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 110f));
            experimental.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 310f));

            _smoothingBar.Minimum = 0;
            _smoothingBar.Maximum = 90;
            _smoothingBar.TickFrequency = 10;
            _smoothingBar.Width = 220;
            _smoothingBar.ValueChanged += (s, e) =>
            {
                _smoothingValue.Text = (_smoothingBar.Value / 100f).ToString("F2", CultureInfo.InvariantCulture);
                PushSettingsToPipeline();
            };
            var smoothingPanel = new FlowLayoutPanel { AutoSize = true, WrapContents = false };
            smoothingPanel.Controls.Add(_smoothingBar);
            smoothingPanel.Controls.Add(_smoothingValue);
            _smoothingValue.Width = 50;
            _smoothingValue.TextAlign = ContentAlignment.MiddleLeft;
            AddStatusRow(tuning, "平滑强度", smoothingPanel);

            _limitBox.DecimalPlaces = 2;
            _limitBox.Increment = 0.05m;
            _limitBox.Minimum = 0m;
            _limitBox.Maximum = 3m;
            _limitBox.Width = 80;
            _limitBox.ValueChanged += (s, e) => PushSettingsToPipeline();
            AddStatusRow(tuning, "位移限制(米)", _limitBox);

            // Software compensation for a Kinect mounted above the user and angled downwards.
            // Positive tilts the tracked skeleton backwards, cancelling the lean that a
            // downward-looking sensor reports for an upright sitter.
            _tiltBox.DecimalPlaces = 1;
            _tiltBox.Increment = 1m;
            _tiltBox.Minimum = -60m;
            _tiltBox.Maximum = 60m;
            _tiltBox.Width = 80;
            _tiltBox.ValueChanged += (s, e) => PushSettingsToPipeline();
            var tiltTip = new ToolTip();
            tiltTip.SetToolTip(_tiltBox,
                "Kinect 俯仰角修正（度）。\r\n" +
                "正值＝把捕捉到的骨架整体向后掰，用来抵消\r\n" +
                "「Kinect 装在前上方斜向下拍」造成的假性前倾。\r\n" +
                "边调边看，坐直后角色也直立即可，0 表示不修正。");
            AddStatusRow(tuning, "俯仰修正(度)", _tiltBox);

            // The arm rest direction must match the avatar's BIND pose, which is the pose its bones
            // are skinned in -- not the pose a freshly loaded model happens to show. A model can
            // display a lowered-arms idle animation while its skeleton is still bound horizontally.
            _armPoseBox.DropDownStyle = ComboBoxStyle.DropDownList;
            _armPoseBox.Width = 120;
            _armPoseBox.Items.Add("水平 T 字");
            _armPoseBox.Items.Add("双臂下垂");
            _armPoseBox.SelectedIndex = 0;
            _armPoseBox.SelectedIndexChanged += (s, e) => PushSettingsToPipeline();
            var armPoseTip = new ToolTip();
            armPoseTip.SetToolTip(_armPoseBox,
                "角色骨骼的「绑定姿势」(bind pose)，不是它默认站着的姿势。\r\n" +
                "多数 VRM/Warudo 模型绑定为水平 T 字 —— 选这个。\r\n" +
                "若选 T 字后手臂方向仍然大面积不对，再试「双臂下垂」。\r\n" +
                "切换后会清掉已捕获的基准。");
            AddStatusRow(tuning, "手臂基准", _armPoseBox);

            // VMC carries each bone's rotation RELATIVE TO ITS PARENT. Using world rotations makes
            // every joint compound its parent's, so a straight arm bends a little more at each
            // joint down the chain. Off by default; available because a receiver could differ.
            _worldRotationBox.Text = "发送世界旋转（默认关，一般不要勾）";
            _worldRotationBox.Width = 240;
            _worldRotationBox.CheckedChanged += (s, e) => PushSettingsToPipeline();
            var worldTip = new ToolTip();
            worldTip.SetToolTip(_worldRotationBox,
                "VMC 协议规定骨骼旋转是「相对父骨骼」的局部旋转。\r\n" +
                "勾上会改成发送世界旋转，导致每级关节把父级旋转再叠加一次，\r\n" +
                "表现为手臂越往下游越弯（小臂、手各弯约 90°）。\r\n" +
                "只有在确认接收端要世界旋转时才勾选。");
            AddStatusRow(tuning, "旋转模式", _worldRotationBox);

            // Kinect reports the fingertip and thumb joints as inferred, so how usable they are
            // cannot be predicted -- it has to be measured on the real sensor.
            _handLogBox.Text = "记录手部数据到 hand-log.csv";
            _handLogBox.Width = 240;
            _handLogBox.CheckedChanged += (s, e) => PushSettingsToPipeline();
            var handLogTip = new ToolTip();
            handLogTip.SetToolTip(_handLogBox,
                "把每帧测到的手部指标写进 exe 旁边的 hand-log.csv。\r\n" +
                "用来确定「拳头/张开」的判定阈值 —— Kinect 的指尖关节\r\n" +
                "是推断值，只能实测，不能靠猜。");
            AddStatusRow(experimental, "手部日志", _handLogBox);

            // Two-state only, with hysteresis: a real hand passes through every intermediate shape,
            // so a three-state readout would flicker and make the model's hand twitch.
            _fistBox.Text = "拳头识别（勾选后发送手指骨骼）";
            _fistBox.Width = 260;
            _fistBox.CheckedChanged += (s, e) => PushSettingsToPipeline();
            var fistTip = new ToolTip();
            fistTip.SetToolTip(_fistBox,
                "只区分「拳头」和「张开」两种状态，中间态保持原状不切换。\r\n" +
                "勾选后会发送 30 根手指骨骼，模型的手会在握拳时并拢。\r\n" +
                "默认关闭：Kinect 的指尖是推断值，信号会抖，\r\n" +
                "而抖动的手比一直张开更难看。");
            AddStatusRow(experimental, "拳头识别", _fistBox);

            // A switch to prove where a palm that points the wrong way comes from.
            //
            // The hand's palm orientation cannot be recovered from Kinect joint positions at all:
            // they give a direction, and the shortest-arc rotation that follows from it fixes
            // where the hand POINTS while leaving its roll to whatever the rig inherits down the
            // arm. Probing the solver found the hand bone's own rotation is 0 degrees with the arm
            // hanging, so nothing here can be producing a 180 degree flip.
            //
            // Turning this off sends no hand bone at all. If the palm still flips with it off,
            // the cause is in the rig or in how it applies the rotation, not in this program --
            // which is the one question no amount of reading the maths can settle.
            _handBox.Text = "发送手骨（取消勾选可测试手掌翻转来源）";
            _handBox.Width = 300;
            _handBox.Checked = true;
            _handBox.CheckedChanged += (s, e) => PushSettingsToPipeline();
            var handTip = new ToolTip();
            handTip.SetToolTip(_handBox,
                "开启时发送 LeftHand / RightHand 骨骼，手腕会跟随手臂朝向。\r\n" +
                "取消勾选后完全不发送手骨，模型的手保持自身待机姿势。\r\n" +
                "用来判断「手掌朝向不对」是来自本程序还是模型绑定：\r\n" +
                "关掉后仍然翻转，就说明与本程序无关。");
            AddStatusRow(experimental, "发送手骨", _handBox);
            // Kinect's own joint orientations: the rotation about each bone's own axis.
            //
            // This is the data that was never read, and its absence is why the palm could never be
            // oriented. A shortest-arc swing introduces no twist, so with the arm hanging the
            // upper arm carried the forearm and hand round rigidly. Inferring the roll from the
            // thumb joint was tried instead and landed a constant 180 degrees out.
            _orientationBox.Text = "关节朝向（修正前臂扭转 / 掌心方向）";
            _orientationBox.Width = 320;
            _orientationBox.Checked = true;
            _orientationBox.CheckedChanged += (s, e) => PushSettingsToPipeline();
            var orientationTip = new ToolTip();
            orientationTip.SetToolTip(_orientationBox,
                "使用 Kinect 的 JointOrientations，它包含绕骨骼自身轴的旋转。\r\n" +
                "关节位置无法表达这个旋转，所以关闭后手臂放下时掌心会朝外。\r\n" +
                "需要先按「基准改为当前坐姿」，中性姿势才会是零旋转。");
            AddStatusRow(tuning, "关节朝向", _orientationBox);

            _curlInwardBox.Text = "折指反向（反关节时勾选）";
            _curlInwardBox.Width = 240;
            _curlInwardBox.CheckedChanged += (s, e) => PushSettingsToPipeline();
            var curlTip = new ToolTip();
            curlTip.SetToolTip(_curlInwardBox,
                "握拳时手指折叠的方向。这是靠真机试出来的，不是推导出来的：\r\n" +
                "掌心法线（手指 × 拇指）的符号并不可靠，因为拇指方向不垂直于手指方向。\r\n" +
                "如果握拳时手指是反关节，勾选这一项即可换向。");
            AddStatusRow(experimental, "折指方向", _curlInwardBox);

            _fullBodyBox.Text = "全身模式（含髋部与双腿）";
            _fullBodyBox.Width = 240;
            _fullBodyBox.CheckedChanged += (s, e) => PushSettingsToPipeline();
            var fullTip = new ToolTip();
            fullTip.SetToolTip(_fullBodyBox,
                "额外发送 Hips 与左右腿各 3 节骨骼。\r\n" +
                "坐姿下 Kinect 看不到腿，发出去的姿势可能比待机动画更差，\r\n" +
                "所以默认关闭；坐远一点、腿能进画面时再开。");
            AddStatusRow(tuning, "骨骼范围", _fullBodyBox);

            _rootRotationBox.Text = "转身（发送根节点旋转）";
            _rootRotationBox.Width = 240;
            _rootRotationBox.CheckedChanged += (s, e) => PushSettingsToPipeline();
            var rootTip = new ToolTip();
            rootTip.SetToolTip(_rootRotationBox,
                "发送根节点旋转，让角色跟着你转身。\r\n" +
                "关闭时角色朝向固定，小幅转身会表现为手臂在漂移。\r\n" +
                "肩线在标定时的朝向被当作「正面」。\r\n" +
                "与下面的「位移」互不影响，可以只开这一个。");
            AddStatusRow(tuning, "转身", _rootRotationBox);

            _sendRootBox.Text = "身体跟随位移";
            _sendRootBox.AutoSize = true;
            _sendRootBox.CheckedChanged += (s, e) => PushSettingsToPipeline();
            AddStatusRow(tuning, "位移", _sendRootBox);

            _sendHeadBox.Text = "发送头部旋转（需关闭 VSeeFace 的头部发送）";
            _sendHeadBox.AutoSize = true;
            _sendHeadBox.CheckedChanged += (s, e) => PushSettingsToPipeline();
            AddStatusRow(tuning, "头部", _sendHeadBox);

            _mirrorBox.Text = "左右镜像（Kinect 在人后方时用）";
            _mirrorBox.AutoSize = true;
            _mirrorBox.CheckedChanged += (s, e) => PushSettingsToPipeline();
            AddStatusRow(tuning, "镜像", _mirrorBox);

            stack.Controls.Add(tuning);

            // ---- experimental ----------------------------------------------------------------
            //
            // These are kept because they are genuinely useful for investigating, but none of them
            // is a finished feature, and presenting them beside the working settings implies they
            // are. The rows above are the ones that are known to work; a warning here says plainly
            // which side of that line the rest are on.
            stack.Controls.Add(SectionLabel("实验性（功能未完成）"));
            var experimentalWarning = new Label
            {
                Text = "以下选项仍在验证中，默认值即为当前推荐值。\r\n" +
                       "改动它们可能让手部表现变差，出问题时把它们复位即可。",
                AutoSize = false,
                Width = 430,
                Height = 36,
                ForeColor = Color.FromArgb(160, 96, 0),
            };
            stack.Controls.Add(experimentalWarning);

            // The experimental rows were appended to this table as they were built above, so the
            // table itself is created here and simply placed after the warning.
            stack.Controls.Add(experimental);

            return stack;
        }

        private Control BuildRightColumn()
        {
            var stack = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 3
            };
            stack.RowStyles.Add(new RowStyle(SizeType.Absolute, 26f));
            stack.RowStyles.Add(new RowStyle(SizeType.Percent, 55f));
            stack.RowStyles.Add(new RowStyle(SizeType.Percent, 45f));

            stack.Controls.Add(SectionLabel("骨架预览（上半身，正对镜头）"), 0, 0);

            _preview.Dock = DockStyle.Fill;
            _preview.BackColor = Color.FromArgb(24, 26, 32);
            stack.Controls.Add(_preview, 0, 1);

            var listPanel = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 2 };
            listPanel.RowStyles.Add(new RowStyle(SizeType.Absolute, 26f));
            listPanel.RowStyles.Add(new RowStyle(SizeType.Percent, 100f));
            listPanel.Controls.Add(SectionLabel("正在发送的骨骼旋转"), 0, 0);

            _rotationList.Dock = DockStyle.Fill;
            _rotationList.Font = new Font("Consolas", 8.5f);
            listPanel.Controls.Add(_rotationList, 0, 1);

            stack.Controls.Add(listPanel, 0, 2);

            return stack;
        }

        /// <summary>
        /// Starts or stops the Python hand tracker.
        ///
        /// The VMC port is the one this program is sending to, because the tracker sends to the
        /// same Warudo receiver -- pointing them at different ports is a silent failure where the
        /// hand simply never moves.
        /// </summary>
        private void ToggleHandTracker()
        {
            if (_handTracker.IsRunning) { StopHandTracker(); return; }

            int port = (int)_portBox.Value;
            string preview;
            switch (_handPreviewBox.SelectedIndex)
            {
                case 1: preview = "hidden"; break;
                case 2: preview = "shown"; break;
                default: preview = "black"; break;
            }

            // The relay port comes from the pipeline, which owns the listener.
            int relayPort = _pipeline.RelayPort;
            if (_handTracker.Start(port, 0, preview, null, relayPort))
            {
                _handStartButton.Text = "重启";
                _handStopButton.Enabled = true;
            }

            _handStatusValue.Text = _handTracker.LastMessage
                + (preview == "hidden" ? "" : "\r\n（在该窗口内按 q 可退出，m 切换镜像）");
        }

        private void StopHandTracker()
        {
            _handTracker.Stop();
            _handStartButton.Text = "启动手部识别";
            _handStopButton.Enabled = false;
            _handStatusValue.Text = _handTracker.LastMessage;
        }

        private static Label SectionLabel(string text)
        {
            return new Label
            {
                Text = text,
                AutoSize = true,
                Font = new Font("Microsoft YaHei UI", 10f, FontStyle.Bold),
                Margin = new Padding(3, 10, 3, 4)
            };
        }

        private static void AddStatusRow(TableLayoutPanel table, string caption, Control value)
        {
            var label = new Label
            {
                Text = caption,
                AutoSize = true,
                ForeColor = Color.DimGray,
                Margin = new Padding(3, 6, 3, 0)
            };
            int row = table.RowCount;
            table.RowCount = row + 1;
            table.Controls.Add(label, 0, row);
            table.Controls.Add(value, 1, row);
            value.Margin = new Padding(3, 3, 3, 3);
        }

        // ------------------------------------------------------------------ settings

        private static string SettingsPath
        {
            get
            {
                string directory = Path.GetDirectoryName(
                    System.Reflection.Assembly.GetExecutingAssembly().Location);
                return Path.Combine(directory ?? ".", "settings.txt");
            }
        }

        private void LoadSettings()
        {
            try
            {
                if (File.Exists(SettingsPath))
                {
                    _settings.LoadFromKeyValueText(File.ReadAllText(SettingsPath, Encoding.UTF8));
                }
            }
            catch (Exception ex)
            {
                _messageValue.Text = "读取设置失败：" + ex.Message;
            }

            _addressBox.Text = _settings.TargetAddress;
            _portBox.Value = _settings.TargetPort;
            _smoothingBar.Value = (int)Math.Round(_settings.Smoothing * 100f);
            _smoothingValue.Text = _settings.Smoothing.ToString("F2", CultureInfo.InvariantCulture);
            _sendHeadBox.Checked = _settings.SendHead;
            _sendRootBox.Checked = _settings.SendRootPosition;
            _mirrorBox.Checked = _settings.Mirror;
            _limitBox.Value = (decimal)Math.Min(3f, Math.Max(0f, _settings.MovementLimit));
            _tiltBox.Value = (decimal)Math.Min(60f, Math.Max(-60f, _settings.KinectTiltDegrees));
            _armPoseBox.SelectedIndex = _settings.HangingArmReference ? 1 : 0;
            _worldRotationBox.Checked = _settings.EmitWorldRotations;
            _handLogBox.Checked = _settings.LogHandMetrics;
            _fistBox.Checked = _settings.EmitFingers;
            _handBox.Checked = _settings.SendHand;
            _relayBox.Checked = _settings.RelayColour;
            _cameraHandBox.Checked = _settings.UseCameraHand;
            _pluginSkeletonBox.Checked = _settings.SendSkeletonToPlugin;
            _absoluteHandBox.Checked = _settings.AbsoluteHand;
            _mirrorHandBox.Checked = _settings.MirrorHandY;
            _divideParentBox.Checked = _settings.DivideParentFromHand;
            if (_mirrorPlaneBoxes[0] != null)
            {
                _mirrorPlaneBoxes[0].Checked = _settings.MirrorPlaneYZ;
                _mirrorPlaneBoxes[1].Checked = _settings.MirrorPlaneXZ;
                _mirrorPlaneBoxes[2].Checked = _settings.MirrorPlaneXY;
            }

            // Each role picks its own measured axis, so all six permutations are reachable.
            if (_handAxisChoosers[0] != null)
            {
                _handAxisChoosers[0].SelectedIndex = Clamp(_settings.HandAxisX);
                _handAxisChoosers[1].SelectedIndex = Clamp(_settings.HandAxisY);
                _handAxisChoosers[2].SelectedIndex = Clamp(_settings.HandAxisZ);
            }
            _orientationBox.Checked = _settings.UseJointOrientations;
            _fullBodyBox.Checked = _settings.SendFullBody;
            _rootRotationBox.Checked = _settings.SendRootRotation;
            _curlInwardBox.Checked = _settings.FingerCurlInward;
        }

        private void SaveSettings(bool notify)
        {
            try
            {
                File.WriteAllText(SettingsPath, _settings.ToKeyValueText(), Encoding.UTF8);
                if (notify) { _messageValue.Text = "设置已保存到 " + SettingsPath; }
            }
            catch (Exception ex)
            {
                _messageValue.Text = "保存设置失败：" + ex.Message;
            }
        }

        /// <summary>Clamps a stored axis index into the valid range.</summary>
        private static int Clamp(int value)
        {
            if (value < 0) { return 0; }
            if (value > 2) { return 2; }
            return value;
        }

        /// <summary>The chosen measured axis for a role, defaulting to the identity permutation.</summary>
        private int AxisChoice(int role)
        {
            ComboBox chooser = _handAxisChoosers[role];
            if (chooser == null || chooser.SelectedIndex < 0) { return role; }
            return chooser.SelectedIndex;
        }
        private void PushSettingsToPipeline()
        {
            if (_loadingSettings) { return; }

            _settings.TargetAddress = _addressBox.Text.Trim();
            _settings.TargetPort = (int)_portBox.Value;
            _settings.Smoothing = _smoothingBar.Value / 100f;
            _settings.SendHead = _sendHeadBox.Checked;
            _settings.SendRootPosition = _sendRootBox.Checked;
            _settings.Mirror = _mirrorBox.Checked;
            _settings.MovementLimit = (float)_limitBox.Value;
            _settings.KinectTiltDegrees = (float)_tiltBox.Value;
            _settings.HangingArmReference = _armPoseBox.SelectedIndex == 1;
            _settings.EmitWorldRotations = _worldRotationBox.Checked;
            _settings.LogHandMetrics = _handLogBox.Checked;
            _settings.EmitFingers = _fistBox.Checked;
            _settings.SendHand = _handBox.Checked;
            _settings.RelayColour = _relayBox.Checked;
            _settings.UseCameraHand = _cameraHandBox.Checked;
            _settings.SendSkeletonToPlugin = _pluginSkeletonBox.Checked;
            _settings.AbsoluteHand = _absoluteHandBox.Checked;
            _settings.MirrorHandY = _mirrorHandBox.Checked;
            _settings.DivideParentFromHand = _divideParentBox.Checked;
            if (_mirrorPlaneBoxes[0] != null)
            {
                _settings.MirrorPlaneYZ = _mirrorPlaneBoxes[0].Checked;
                _settings.MirrorPlaneXZ = _mirrorPlaneBoxes[1].Checked;
                _settings.MirrorPlaneXY = _mirrorPlaneBoxes[2].Checked;
            }
            _settings.HandAxisX = AxisChoice(0);
            _settings.HandAxisY = AxisChoice(1);
            _settings.HandAxisZ = AxisChoice(2);
            _settings.HandSignX = _handSignBoxes[0] != null && _handSignBoxes[0].Checked ? -1f : 1f;
            _settings.HandSignY = _handSignBoxes[1] != null && _handSignBoxes[1].Checked ? -1f : 1f;
            _settings.HandSignZ = _handSignBoxes[2] != null && _handSignBoxes[2].Checked ? -1f : 1f;
            _settings.UseJointOrientations = _orientationBox.Checked;
            _settings.SendFullBody = _fullBodyBox.Checked;
            _settings.SendRootRotation = _rootRotationBox.Checked;
            _settings.FingerCurlInward = _curlInwardBox.Checked;

            _pipeline.ApplySettings();
        }

        /// <summary>
        /// Writes a diagnostic report beside the executable and shows it, so the user can read
        /// it and send it on without hunting for the file.
        /// </summary>
        private void WriteDiagnostics()
        {
            PushSettingsToPipeline();

            try
            {
                string text;
                string path = DiagnosticsReport.Write(_settings, _pipeline, out text);

                _messageValue.Text = "诊断报告已保存：" + path;

                using (var viewer = new DiagnosticsViewer(path, text))
                {
                    viewer.ShowDialog(this);
                }
            }
            catch (Exception ex)
            {
                _messageValue.Text = "导出诊断报告失败：" + ex.Message;
            }
        }

        // ------------------------------------------------------------------ actions

        private void ToggleRunning()
        {
            PushSettingsToPipeline();

            // Follow the real state rather than the button caption: if the worker thread ever
            // stops on its own, a caption check would leave the button stuck on "stop".
            if (_pipeline.IsRunning)
            {
                _pipeline.Stop();
                _startStopButton.Text = "开始追踪";
            }
            else
            {
                _pipeline.Start();
                _startStopButton.Text = "停止追踪";
                SaveSettings(false);
            }
        }

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            _refreshTimer.Stop();
            SaveSettings(false);

            // The hand tracker is a separate process, so closing this window does not end it. Left
            // running it would hold the camera and keep sending VMC after the UI is gone, which
            // looks like a ghost controlling the avatar.
            _handTracker.Dispose();

            _pipeline.Dispose();
            base.OnFormClosing(e);
        }

        // ------------------------------------------------------------------ refresh

        private void RefreshUi()
        {
            PipelineState state = _pipeline.State;

            _stateValue.Text = DescribeState(state);
            _stateValue.ForeColor = state == PipelineState.Tracking ? Color.SeaGreen
                : (state == PipelineState.Stopped ? Color.DimGray : Color.DarkOrange);

            _deviceValue.Text = _pipeline.SensorAvailable
                ? "已连接" + (string.IsNullOrEmpty(_pipeline.SensorId) ? "" : "（" + ShortId(_pipeline.SensorId) + "）")
                : "未连接";
            _deviceValue.ForeColor = _pipeline.SensorAvailable ? Color.SeaGreen : Color.Firebrick;

            // The hand tracker runs without a console, so this label is the only place its state can
            // be seen -- and it is the difference between "the palm correction is broken" and "the
            // camera never saw a hand", which look identical from the avatar.
            _palmStatusValue.Text = _pipeline.PalmReferenceStatus;

            _bodyValue.Text = _pipeline.TrackedBodyCount > 0
                ? "跟踪到 " + _pipeline.TrackedBodyCount + " 人"
                : "未检测到";
            _bodyValue.ForeColor = _pipeline.TrackedBodyCount > 0 ? Color.SeaGreen : Color.DarkOrange;

            _rateValue.Text = _pipeline.FrameRate.ToString("F1", CultureInfo.InvariantCulture) + " 帧/秒";
            _bonesValue.Text = _pipeline.BoneCount.ToString(CultureInfo.InvariantCulture) + " 根";
            _sendValue.Text = _pipeline.PacketsSent.ToString(CultureInfo.InvariantCulture) + " 包";

            // The rotation reference is a trade-off, not a quality setting: which arm motion is
            // geometrically clean depends on it. Say which one is active.
            _calibValue.Text = _pipeline.UsesCapturedNeutral
                ? "当前坐姿（自定义，适合手臂前后摆动）"
                : "水平 T 字（默认，适合手臂前伸/侧展）";
            _calibValue.ForeColor = Color.SeaGreen;

            // Hand shape is a rough estimate: Kinect supplies five hand joints and no
            // per-finger angles, so only fist vs open is meaningful.
            _handValue.Text = DescribeHand(_pipeline.LeftHand, "左") + "  " + DescribeHand(_pipeline.RightHand, "右");

            string message = _pipeline.StatusText;
            if (!string.IsNullOrEmpty(_pipeline.LastError))
            {
                message += "　|　" + _pipeline.LastError;
            }
            _messageValue.Text = message;

            _preview.SetFrame(_pipeline.LatestJoints);
            _preview.Invalidate();

            UpdateRotationList();
        }

        /// <summary>
        /// One hand's state as a short label plus the raw tip-to-thumb ratio, so the number can be
        /// compared against the log when calibrating the fist threshold.
        /// </summary>
        private static string DescribeHand(Kinect2VMC.Core.HandMetrics metrics, string side)
        {
            if (!metrics.Valid) { return side + "手: 未检测"; }

            string pose;
            switch (metrics.Pose)
            {
                case Kinect2VMC.Core.HandPose.Fist: pose = "握拳"; break;
                case Kinect2VMC.Core.HandPose.LooseFist: pose = "半握"; break;
                case Kinect2VMC.Core.HandPose.Open: pose = "张开"; break;
                default: pose = "?"; break;
            }

            return side + "手: " + pose
                + "(" + metrics.TipToThumb.ToString("F2", CultureInfo.InvariantCulture) + ")";
        }

        private void UpdateRotationList()
        {
            Dictionary<string, Quat> rotations = _pipeline.LatestRotations;
            if (rotations == null || rotations.Count == 0)
            {
                if (_rotationList.Items.Count != 0) { _rotationList.Items.Clear(); }
                return;
            }

            // Only rebuild when the set of bones or the rounded values actually changed; a
            // 5 Hz UI refresh over 11 rows is cheap, but flicker is not.
            var lines = new List<string>(rotations.Count + 1);
            lines.Add("骨骼              四元数 (x, y, z, w)");
            foreach (var pair in rotations)
            {
                Quat q = pair.Value;
                lines.Add(string.Format(CultureInfo.InvariantCulture,
                    "{0,-16} {1,7:F3} {2,7:F3} {3,7:F3} {4,7:F3}",
                    pair.Key, q.X, q.Y, q.Z, q.W));
            }

            bool same = _rotationList.Items.Count == lines.Count;
            if (same)
            {
                for (int i = 0; i < lines.Count; i++)
                {
                    if (!string.Equals((string)_rotationList.Items[i], lines[i], StringComparison.Ordinal))
                    {
                        same = false;
                        break;
                    }
                }
            }

            if (same) { return; }

            _rotationList.BeginUpdate();
            try
            {
                _rotationList.Items.Clear();
                foreach (string line in lines) { _rotationList.Items.Add(line); }
            }
            finally
            {
                _rotationList.EndUpdate();
            }
        }

        private static string DescribeState(PipelineState state)
        {
            switch (state)
            {
                case PipelineState.Tracking: return "追踪中";
                case PipelineState.WaitingForSensor: return "等待 Kinect";
                case PipelineState.WaitingForBody: return "等待人体";
                default: return "已停止";
            }
        }

        private static string ShortId(string id)
        {
            if (string.IsNullOrEmpty(id)) { return string.Empty; }
            return id.Length <= 8 ? id : id.Substring(0, 8) + "…";
        }
    }

    /// <summary>
    /// Shows a diagnostic report as selectable text, with a button to open the folder so the
    /// user can attach the file to a message.
    /// </summary>
    public sealed class DiagnosticsViewer : Form
    {
        private readonly string _path;

        public DiagnosticsViewer(string path, string text)
        {
            _path = path;

            Text = "诊断报告";
            StartPosition = FormStartPosition.CenterParent;
            Size = new Size(760, 620);
            MinimumSize = new Size(520, 400);
            Font = new Font("Microsoft YaHei UI", 9f);

            var box = new TextBox
            {
                Dock = DockStyle.Fill,
                Multiline = true,
                ReadOnly = true,
                ScrollBars = ScrollBars.Both,
                WordWrap = false,
                Font = new Font("Consolas", 9f),
                Text = text.Replace("\n", Environment.NewLine)
            };

            var pathLabel = new Label
            {
                Dock = DockStyle.Fill,
                Text = "已保存到：" + path,
                TextAlign = ContentAlignment.MiddleLeft,
                AutoEllipsis = true,
                Padding = new Padding(6, 0, 6, 0)
            };

            var openFolder = new Button { Text = "打开所在文件夹", Width = 130, Height = 28 };
            openFolder.Click += (s, e) => OpenContainingFolder();

            var copy = new Button { Text = "复制全部", Width = 90, Height = 28 };
            copy.Click += (s, e) =>
            {
                try
                {
                    Clipboard.SetText(box.Text);
                    pathLabel.Text = "已复制到剪贴板。原文件：" + _path;
                }
                catch (Exception ex)
                {
                    pathLabel.Text = "复制失败：" + ex.Message;
                }
            };

            var close = new Button { Text = "关闭", Width = 80, Height = 28, DialogResult = DialogResult.OK };

            var bottom = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.LeftToRight,
                WrapContents = false,
                AutoSize = true
            };
            bottom.Controls.Add(openFolder);
            bottom.Controls.Add(copy);
            bottom.Controls.Add(close);

            var layout = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 3 };
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100f));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 28f));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 40f));
            layout.Controls.Add(box, 0, 0);
            layout.Controls.Add(pathLabel, 0, 1);
            layout.Controls.Add(bottom, 0, 2);

            Controls.Add(layout);
            AcceptButton = close;
            CancelButton = close;
        }

        private void OpenContainingFolder()
        {
            try
            {
                System.Diagnostics.Process.Start("explorer.exe", "/select,\"" + _path + "\"");
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, "无法打开文件夹：" + ex.Message, "诊断报告",
                    MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
        }
    }

    /// <summary>
    /// Draws the measured upper body from the front, which is how the user is facing the
    /// sensor. Purely a sanity check: it shows instantly whether joint tracking has collapsed.
    /// </summary>
    public sealed class SkeletonPreview : Control
    {
        private Vec3[] _joints;

        // Bones worth drawing, as joint index pairs. Only the upper body is tracked meaningfully.
        private static readonly int[][] Bones =
        {
            new[] { (int)KinectJoint.SpineBase, (int)KinectJoint.SpineMid },
            new[] { (int)KinectJoint.SpineMid, (int)KinectJoint.SpineShoulder },
            new[] { (int)KinectJoint.SpineShoulder, (int)KinectJoint.Neck },
            new[] { (int)KinectJoint.Neck, (int)KinectJoint.Head },
            new[] { (int)KinectJoint.SpineShoulder, (int)KinectJoint.ShoulderLeft },
            new[] { (int)KinectJoint.ShoulderLeft, (int)KinectJoint.ElbowLeft },
            new[] { (int)KinectJoint.ElbowLeft, (int)KinectJoint.WristLeft },
            new[] { (int)KinectJoint.WristLeft, (int)KinectJoint.HandLeft },
            new[] { (int)KinectJoint.SpineShoulder, (int)KinectJoint.ShoulderRight },
            new[] { (int)KinectJoint.ShoulderRight, (int)KinectJoint.ElbowRight },
            new[] { (int)KinectJoint.ElbowRight, (int)KinectJoint.WristRight },
            new[] { (int)KinectJoint.WristRight, (int)KinectJoint.HandRight }
        };

        public SkeletonPreview()
        {
            DoubleBuffered = true;
            ResizeRedraw = true;
        }

        public void SetFrame(Vec3[] joints)
        {
            _joints = joints;
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);

            Graphics g = e.Graphics;
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

            using (var caption = new SolidBrush(Color.FromArgb(150, 150, 160)))
            {
                if (_joints == null)
                {
                    g.DrawString("等待数据…", Font, caption, 10f, 10f);
                    return;
                }
            }

            Vec3[] joints = _joints;

            // Project X/Y straight to the screen: the user faces the sensor, so this is a
            // front view. A fixed window in metres keeps the scale stable between frames.
            const float windowHeight = 1.30f;
            float scale = Height / windowHeight;
            float centreX = Width * 0.5f;
            float originY = Height - 24f;
            float baseY = joints[(int)KinectJoint.SpineBase].Y;

            Func<Vec3, PointF> project = p => new PointF(
                centreX + p.X * scale,
                originY - (p.Y - baseY) * scale);

            using (var pen = new Pen(Color.FromArgb(90, 190, 255), 3f))
            using (var jointBrush = new SolidBrush(Color.FromArgb(255, 220, 120)))
            using (var faint = new SolidBrush(Color.FromArgb(120, 120, 130)))
            {
                foreach (int[] bone in Bones)
                {
                    PointF a = project(joints[bone[0]]);
                    PointF b = project(joints[bone[1]]);
                    pen.Color = Color.FromArgb(90, 190, 255);
                    g.DrawLine(pen, a, b);

                    g.FillEllipse(jointBrush, a.X - 3.5f, a.Y - 3.5f, 7f, 7f);
                }

                PointF head = project(joints[(int)KinectJoint.Head]);
                g.DrawEllipse(pen, head.X - 9f, head.Y - 20f, 18f, 22f);

                g.DrawString("正视图，仅上半身", Font, faint, 10f, 10f);
            }
        }
    }
}
