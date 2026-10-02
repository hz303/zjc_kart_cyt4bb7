using System;
using System.Drawing;
using System.Windows.Forms;

namespace KartHost.Ui
{
    /// <summary>
    /// 行程标定：把转向推到左右机械硬限位，各记一次"速度积分"，就同时得到
    ///   ① 机械行程 ↔ 速度积分的定量映射（mrad/°）——这就是"角度-速度积分"的换算系数；
    ///   ② 机械中点（积分值），用于算出"距中点多少度"。
    ///
    /// 为什么用速度积分而不是绝对角度：编码器装在多圈轴上，单圈绝对角**不唯一**；
    /// 而速度积分是连续、带方向的多圈量 —— 只有它能在整段行程上和机械角一一对应。
    ///
    /// 纯计算部分单独放在 TravelCal 里，自检可以直接断言，不用碰 UI。
    /// </summary>
    public sealed class TravelCal
    {
        public const double MradPerTurn = 2000.0 * Math.PI;   // 一圈 = 2π rad = 2000π mrad ≈ 6283.19
        public const double MinSpanMrad = 200.0;              // 跨度 < 200 mrad(≈11.5°) 视为没标定好

        /// <summary>左限位时的速度积分（mrad）。</summary>
        public double LeftSum { get; set; } = double.NaN;
        /// <summary>右限位时的速度积分（mrad）。</summary>
        public double RightSum { get; set; } = double.NaN;
        /// <summary>左限位时的绝对角度（°）—— 只做旁证，不参与换算。</summary>
        public double LeftDeg { get; set; } = double.NaN;
        public double RightDeg { get; set; } = double.NaN;
        /// <summary>机械左右硬限位之间的总行程（°），默认按卡丁车舵机 ±30° 填 60。</summary>
        public double MechSpanDeg { get; set; } = 60.0;

        public bool HasLeft => !double.IsNaN(LeftSum);
        public bool HasRight => !double.IsNaN(RightSum);
        public bool Complete => HasLeft && HasRight;

        /// <summary>右 − 左（mrad，带符号）。</summary>
        public double SpanSum => Complete ? RightSum - LeftSum : double.NaN;
        /// <summary>跨度是否够大，值得拿去做换算。</summary>
        public bool SpanOk => Complete && Math.Abs(SpanSum) >= MinSpanMrad;
        /// <summary>跨度折算成"圈"。</summary>
        public double Turns => SpanOk ? Math.Abs(SpanSum) / MradPerTurn : double.NaN;
        /// <summary>机械中点的积分值（mrad）。</summary>
        public double MidSum => Complete ? (LeftSum + RightSum) * 0.5 : double.NaN;

        /// <summary>★ 核心换算系数：每机械度对应多少 mrad 的速度积分。</summary>
        public double MradPerDeg
            => (SpanOk && MechSpanDeg > 0.5) ? Math.Abs(SpanSum) / MechSpanDeg : double.NaN;

        /// <summary>由速度积分反算"距机械中点多少度"（朝右为正）；没标定好返回 NaN。</summary>
        public double AngleDeg(double sum)
        {
            double k = MradPerDeg;
            if (double.IsNaN(k) || k <= 0.0 || double.IsNaN(sum)) return double.NaN;
            return (sum - MidSum) / k;
        }

        /// <summary>半量程（°）：标定完成后左限位 = −半量程、右限位 = +半量程。</summary>
        public double HalfSpanDeg => SpanOk ? MechSpanDeg * 0.5 : double.NaN;

        public void Clear() => LeftSum = RightSum = LeftDeg = RightDeg = double.NaN;

        /// <summary>按"哪边限位"记录一组采样。传入 NaN 表示没有数据，返回 false。</summary>
        public bool Capture(bool left, double sumMrad, double deg)
        {
            if (double.IsNaN(sumMrad)) return false;
            if (left) { LeftSum = sumMrad; LeftDeg = deg; }
            else { RightSum = sumMrad; RightDeg = deg; }
            return true;
        }
    }

    /// <summary>行程标定的界面（挂在右侧栏）。</summary>
    public sealed class TravelCalPanel : Panel
    {
        public TravelCal Cal { get; } = new TravelCal();

        private readonly NumericUpDown _numSpan;
        private readonly Label _lbHead, _lbBody, _lbNow;
        private readonly Button _btnLeft, _btnRight, _btnClear, _btnZero;

        private int _chSum = -1, _chDeg = -1, _chAngle = -1;
        private double _lastSum = double.NaN;

        private readonly ChannelStore _store;
        private readonly Action<float> _setMagZero;
        private readonly Action<string> _log;

        public TravelCalPanel(ChannelStore store, Action<float> setMagZero, Action<string> log)
        {
            _store = store;
            _setMagZero = setMagZero;
            _log = log;

            BackColor = Theme.Panel;
            Dock = DockStyle.Top;
            Height = Dpi.Px(230);

            _lbHead = new Label
            {
                Text = "  行程标定（速度积分）",
                Dock = DockStyle.Top,
                Height = Dpi.Px(22),
                TextAlign = ContentAlignment.MiddleLeft,
                BackColor = Theme.PanelAlt,
                ForeColor = Theme.Text,
            };

            // 按钮用 AutoSize + MinimumSize：字号/DPI 变了也不会"字被吃掉"（布局体检认这种写法）。
            // 位置用绝对坐标，两列两行 —— 比 FlowLayoutPanel 好推理，也不会撑破侧栏。
            // ⚠ 第 2 个参数是**逻辑**宽度（MkBtn 里会过 Dpi.Sz），第 3/4 个是**物理**坐标（已过 Dpi.Px）。
            //   一开始我两个都传了 Dpi.Px()，结果宽度被缩放两次 → 按钮右沿越出侧栏。
            _btnLeft  = MkBtn("记左限位",    96,  Dpi.Px(6),   Dpi.Px(26));
            _btnRight = MkBtn("记右限位",    96,  Dpi.Px(112), Dpi.Px(26));
            _btnClear = MkBtn("清空",        66,  Dpi.Px(6),   Dpi.Px(58));
            _btnZero  = MkBtn("当前位置→0°", 130, Dpi.Px(84),  Dpi.Px(58));
            _btnLeft.Click  += (_, __) => DoCapture(true);
            _btnRight.Click += (_, __) => DoCapture(false);
            _btnClear.Click += (_, __) => { Cal.Clear(); _log("行程标定已清空"); };
            _btnZero.Click  += (_, __) => DoZeroHere();

            var lbSpan = new Label
            {
                Text = "机械行程",
                AutoSize = true,
                ForeColor = Theme.TextDim,
                Location = new Point(Dpi.Px(6), Dpi.Px(96)),
            };
            _numSpan = new NumericUpDown
            {
                Minimum = 5, Maximum = 360, Value = 60,
                DecimalPlaces = 0, Increment = 1,
                Width = Dpi.Px(58),
                Location = new Point(Dpi.Px(74), Dpi.Px(92)),
            };
            _numSpan.ValueChanged += (_, __) => Cal.MechSpanDeg = (double)_numSpan.Value;

            _lbBody = new Label
            {
                AutoSize = true,
                MaximumSize = new Size(Dpi.Px(224), 0),   // 够宽不换行，又不越出侧栏
                ForeColor = Theme.Text,
                Location = new Point(Dpi.Px(6), Dpi.Px(122)),
                Text = BodyText(),
            };

            _lbNow = new Label
            {
                AutoSize = true,
                ForeColor = Theme.Accent,
                Location = new Point(Dpi.Px(6), Dpi.Px(204)),
                Text = "当前角 --",
            };

            Controls.Add(_lbNow);
            Controls.Add(_lbBody);
            Controls.Add(_numSpan);
            Controls.Add(lbSpan);
            Controls.Add(_btnZero);
            Controls.Add(_btnClear);
            Controls.Add(_btnRight);
            Controls.Add(_btnLeft);
            Controls.Add(_lbHead);

            ResolveChannels();
        }

        /// <param name="logicalMinW">最小宽度，**逻辑值**（内部会过 Dpi）</param>
        /// <param name="x">已经过 Dpi.Px 的物理 x</param>
        /// <param name="y">已经过 Dpi.Px 的物理 y</param>
        private static Button MkBtn(string text, int logicalMinW, int x, int y) => new Button
        {
            Text = text,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            MinimumSize = Dpi.Sz(logicalMinW, 26),
            Location = new Point(x, y),
        };

        /// <summary>按名字找通道下标 —— 换档案也不会指错（找不到返回 -1）。</summary>
        private static int Idx(string name)
        {
            for (int i = 0; i < Defaults.ChannelNames.Length; i++)
                if (Defaults.ChannelName(i) == name) return i;
            return -1;
        }

        /// <summary>
        /// 单电机档案：积分角 = mag_sum；换到别的档案这里会变成 -1，只读按钮自动禁用
        /// （「清空」永远可用 —— 它不依赖通道）。
        /// </summary>
        public void ResolveChannels()
        {
            _chSum = Idx("mag_sum");
            _chDeg = Idx("mag_deg");
            _chAngle = Idx("mag_angle");
            bool ok = _chSum >= 0;
            _btnLeft.Enabled = _btnRight.Enabled = _btnZero.Enabled = ok;
            _lbNow.Text = ok ? "当前角 --" : "当前档案没有速度积分通道，标定不可用";
        }

        /// <summary>取最近约 0.5 s（50 Hz × 25）的均值 —— 单点值会抖，均值才敢用来标定。</summary>
        private double Avg(int ch, int n = 25)
        {
            if (ch < 0) return double.NaN;
            var dv = new float[n];
            var dt = new double[n];
            int got = _store.Snapshot(ch, dv, dt, n);
            if (got <= 0) return double.NaN;
            double s = 0;
            for (int i = 0; i < got; i++) s += dv[i];
            return s / got;
        }

        private void DoCapture(bool left)
        {
            double sum = Avg(_chSum);
            double deg = Avg(_chDeg);
            if (!Cal.Capture(left, sum, deg))
            {
                _log("还没收到数据 —— 先连上车端（或在模拟模式下）再记限位");
                return;
            }
            _log($"已记录{(left ? "左" : "右")}限位：Σ = {sum:0} mrad（{sum / TravelCal.MradPerTurn:+0.000;-0.000} 圈）");
        }

        private void DoZeroHere()
        {
            double ang = Avg(_chAngle);
            if (double.IsNaN(ang)) { _log("还没收到角度数据，无法置零"); return; }
            _setMagZero?.Invoke((float)ang);
            _log($"已把当前位置写进 mag_zero = {ang:0}（角度寄存器原始值）→ mag_deg 现在读 0°");
        }

        private string BodyText()
        {
            string F(double v, string unit) => double.IsNaN(v) ? "--" : v.ToString("0") + unit;
            string d(double v) => double.IsNaN(v) ? "" : $"  ({v:0.0}°)";

            if (!Cal.Complete)
                // 每行都短，保证不换行 —— 换行会把下面的"当前角"顶掉（踩过）
                return "左硬限位 →「记左限位」\n右硬限位 →「记右限位」\n跨度 --\n映射 --";

            if (!Cal.SpanOk)
                return $"左   {F(Cal.LeftSum, " mrad")}\n右   {F(Cal.RightSum, " mrad")}\n"
                     + "跨度太小 → 重记（要顶到硬限位）\n映射 --";

            return $"左   {Cal.LeftSum:0} mrad{d(Cal.LeftDeg)}\n"
                 + $"右   {Cal.RightSum:0} mrad{d(Cal.RightDeg)}\n"
                 + $"跨度 {Cal.SpanSum:0} mrad = {Cal.Turns:0.000} 圈\n"
                 + $"映射 {Cal.MradPerDeg:0.00} mrad/°（中点 {Cal.MidSum:0}）";
        }

        /// <summary>33 ms 调一次（挂在主窗体的 UI 定时器里）。</summary>
        public void RefreshValues()
        {
            if (_chSum < 0) return;
            double sum = Avg(_chSum, 12);
            _lastSum = sum;

            string body = BodyText();
            if (_lbBody.Text != body) _lbBody.Text = body;

            double ang = Cal.AngleDeg(sum);
            string now = double.IsNaN(ang)
                ? (Cal.SpanOk ? $"当前角 --（Σ={sum:0}）" : "当前角 --（先标定）")
                : $"当前角 {ang:+0.0;-0.0;0.0}°  限位 ±{Cal.HalfSpanDeg:0.0}°";
            if (_lbNow.Text != now) _lbNow.Text = now;
        }

        public double LastSum => _lastSum;
    }
}
