using System.Net;
using System.Net.Sockets;

namespace KartHost;

/// <summary>
/// 内置"假车端"：起一个 TCP 服务端，按协议推送遥测帧、响应下行命令。
/// 作用：**手上没有车/模块时也能把整条链路跑通、把界面调好**。
/// 它严格走真实协议，所以能顺带验证帧同步、CRC、丢包统计写得对不对。
/// </summary>
public sealed class Simulator : IDisposable
{
    private TcpListener _srv;
    private Thread _worker;
    private volatile bool _run;
    private Socket _sock;                       // 当前连接（一次只接一个上位机）

    public int Port { get; set; } = 8086;
    public long FramesSent { get; private set; }
    public long CommandsGot { get; private set; }
    public ushort Enable { get; private set; }
    public bool IsRunning { get; private set; }

    /// <summary>与车端 motor_link_online() 同义：收到过下行帧 && 距上次 ≤ lost_ms</summary>
    public bool Online { get; private set; }

    public event Action<string> Log;

    private readonly List<ParamMeta> _params = Defaults.BuildDefaultParams();
    private float[] _ch = new float[Defaults.ChannelNames.Length];   // 档案一变就重分配（见 BuildTelemetry）
    private double _mSum;                                            // 角速度积分（mrad）
    private readonly Random _rnd = new(20260917);
    private string _warnedNoFill;                                    // "这个档案没有造数分支"只提示一次
    /// <summary>「转向闭环」那套 26 通道布局的暂存区（KartRC / Calib 先算到这里再搬家）。</summary>
    private readonly float[] _clBuf = new float[26];

    // ---- 模拟物理状态（独立字段，不要借用 _ch 存状态）----
    private double _t;
    private float _vAct;
    private double _dist, _posX, _posY;
    private double _lastSendT;
    private double _lastCmdT;                   // 最近一次收到下行帧的时刻（秒）
    private bool   _everRx;                     // 上位机开口说过话没有

    // ---- 惯导 + 路表档案专用状态（工程 8）----
    private double _insX, _insY, _insYaw, _insDist;   // 模拟车的位姿（x 右 / y 前 / yaw 度 / 里程 m）
    // ★ 路表的【起点车体系】基准（与车端 ins_route.c 的 origin_x/y/yaw 同义）
    private double _rtX0, _rtY0, _rtYaw0;
    private float  _srvApplied;                       // 本拍舵量（考虑使能与否）
    private int    _insSeq;                           // 采样序号（应匀速增长）
    private bool   _zeroPrev, _routePrev, _magcalPrev;// 三个"边沿命令"的上一拍状态
    private bool   _routeRunning, _magDone;
    private double _routeNextAt;                      // 下一点的里程阈值
    private readonly List<double> _routeX = new();    // 模拟录到的路表（供日志/自检断言"非空"）
    private readonly List<double> _routeY = new();

    internal int      InsRoutePointCount => _routeX.Count;
    internal bool     InsRouteRunning    => _routeRunning;


    public Simulator()
    {
        var z = _params.Find(p => p.Name == "gyro_bias_z");
        if (z != null) z.Value = 0.012f;
    }

    private float Prm(string name)
    {
        var p = _params.Find(x => x.Name == name);
        return p?.Value ?? 0f;
    }

    public void Start()
    {
        if (IsRunning) return;
        _srv = new TcpListener(IPAddress.Loopback, Port);
        _srv.Start();
        _run = true;
        IsRunning = true;
        _worker = new Thread(Run) { IsBackground = true, Name = "KartHost.Sim" };
        _worker.Start();
        Log?.Invoke($"模拟车端已启动：127.0.0.1:{Port}");
    }

    public void Stop()
    {
        _run = false;
        try { _sock?.Close(); } catch { }
        try { _srv?.Stop(); } catch { }
        IsRunning = false;
        Log?.Invoke("模拟车端已停止");
    }

    private void Run()
    {
        while (_run)
        {
            try
            {
                if (!_srv.Pending()) { Thread.Sleep(50); continue; }
                using var cli = _srv.AcceptTcpClient();
                cli.NoDelay = true;
                _sock = cli.Client;
                Log?.Invoke("模拟车端：上位机已接入");
                Serve();
            }
            catch (Exception ex)
            {
                if (_run) Log?.Invoke("模拟车端异常: " + ex.Message);
                Thread.Sleep(200);
            }
            finally
            {
                _sock = null;
            }
        }
    }

    private void Serve()
    {
        var rxbuf = new byte[4096];
        int rxlen = 0;

        var sw = System.Diagnostics.Stopwatch.StartNew();
        double lastT = sw.Elapsed.TotalSeconds;

        while (_run && _sock != null)
        {
            double now = sw.Elapsed.TotalSeconds;
            double dt = Math.Min(now - lastT, 0.05);
            lastT = now;
            if (dt <= 0) dt = 0.001;
            _t += dt;

            // ---- 按 telemetry_hz 推遥测（限频是车端责任，PC 端不做节流）----
            // 单电机 / 双电机 / 转向闭环档案没有 telemetry_hz 这个参数，直接给 50 Hz
            float hz = Defaults.IsClosedLoop ? 50f : Math.Clamp(Prm("telemetry_hz"), 5f, 200f);
            if (now - _lastSendT >= 1.0 / hz)
            {
                _lastSendT = now;
                if (!SendFrame((byte)FrameType.Telemetry, BuildTelemetry())) break;
                FramesSent++;
            }

            // ---- 收命令（非阻塞）----
            try
            {
                while (_sock.Available > 0)
                {
                    if (rxlen >= rxbuf.Length) rxlen = 0;
                    int n = _sock.Receive(rxbuf, rxlen, rxbuf.Length - rxlen, SocketFlags.None);
                    if (n <= 0) return;
                    rxlen += n;
                    rxlen = HandleCommands(rxbuf, rxlen);
                }
            }
            catch { return; }

            Thread.Sleep(2);
        }
    }

    // ==================================================================
    // 下行命令处理
    // ==================================================================
    private int HandleCommands(byte[] b, int len)
    {
        int i = 0;
        while (i + Proto.DN_OVERHEAD + Proto.DN_PAYLOAD <= len)
        {
            if (b[i] != Proto.MAGIC_DN_0 || b[i + 1] != Proto.MAGIC_DN_1) { i++; continue; }
            if (b[i + 2] != (byte)FrameType.Command) { i++; continue; }
            int pl = Proto.GetU16(b, i + 3);
            if (pl != Proto.DN_PAYLOAD) { i++; continue; }
            int total = Proto.DN_OVERHEAD + pl;
            if (i + total > len) break;                                  // 半包，等下次
            if (Proto.Crc16(b, i + 2, 3 + pl) != Proto.GetU16(b, i + 5 + pl)) { i++; continue; }

            byte cmd = b[i + 5];
            byte pid = b[i + 6];
            uint raw = Proto.GetU32(b, i + 7);
            float val = Proto.GetF32(b, i + 7);
            ushort token = Proto.GetU16(b, i + 11);

            // ★ 记录"上位机还活着"。真固件用 rx_age < lost_ms 判在线，这里必须一起建模，
            //   否则"上位机不发心跳 → 车端判失联 → 电机自停"在自检里完全看不见。
            _lastCmdT = _t;
            _everRx   = true;

            CommandsGot++;
            Dispatch(cmd, pid, raw, val, token);
            i += total;
        }

        if (i > 0 && i < len) Buffer.BlockCopy(b, i, b, 0, len - i);
        return len - i;
    }

    private void Dispatch(byte cmd, byte pid, uint raw, float val, ushort token)
    {
        switch ((Cmd)cmd)
        {
            case Cmd.SetParam:
                if (pid < _params.Count)
                {
                    var p = _params[pid];
                    p.Value = p.Snap(val);
                    Log?.Invoke($"参数写入：{p.Label} = {Fmt(p.Value, p.Step)} {p.Unit}");
                    SendParamValue(p.Id, p.Value);      // 回读确认
                }
                Ack(Cmd.SetParam, token);
                break;

            case Cmd.GetParam:
                if (pid < _params.Count) SendParamValue(pid, _params[pid].Value);
                Ack(Cmd.GetParam, token);
                break;

            case Cmd.SetEnable:
                // ★ VALUE 字段按"数值"解释（不是 IEEE754 位模式）：
                //   上位机发 6.0f，这里就该读到 6。两边必须同一个约定，否则使能位恒为 0，
                //   表现是"怎么点使能，电机都不动"——不会报错，纯静默失效。
                Enable = (ushort)Math.Clamp(Math.Round(val), 0, 65535);
                Log?.Invoke("使能 = 0x" + Enable.ToString("X2") + "  " + DescribeEnable(Enable));
                Ack(Cmd.SetEnable, token);
                break;

            case Cmd.Ping:
                Ack(Cmd.Ping, token);
                break;

            case Cmd.GetTable:
                SendParamTable();
                Ack(Cmd.GetTable, token);
                break;

            case Cmd.RouteDump:
                // ★ 模拟车端的「读路表」：把上面 RecTbl 录到的那张表按真实协议分帧发出去。
                //   首尾各 1 帧 + 每 RtChunk 点 1 帧，与车端 motor_rt_tick() 同构。
                if (_routeX.Count < 2)
                {
                    Log?.Invoke("模拟车端：路表为空（先在面板上喂 ins_route=1 让它录一段）");
                    Nak(Cmd.RouteDump, token, 2);
                    break;
                }
                Ack(Cmd.RouteDump, token);
                SendRouteDump();
                break;

            case Cmd.SaveParams:
                foreach (var p in _params) p.Dirty = false;
                Log?.Invoke("参数已保存到 Flash");
                Ack(Cmd.SaveParams, token);
                break;

            case Cmd.LoadParams:
                foreach (var p in _params) p.Value = p.Def;
                Log?.Invoke("已恢复默认参数");
                Ack(Cmd.LoadParams, token);
                break;

            default:
                Log?.Invoke($"收到命令 0x{cmd:X2}（未专门处理，已回 ACK）");
                Ack((Cmd)cmd, token);
                break;
        }
    }

    private static string Fmt(float v, float step)
    {
        int dec = step >= 1f ? 0 : step >= 0.01f ? 2 : 3;
        return v.ToString("F" + dec);
    }

    private static string DescribeEnable(ushort e)
    {
        var parts = new List<string>();
        if ((e & (ushort)EnableBits.Master) != 0) parts.Add("整体");
        if ((e & (ushort)EnableBits.Steer) != 0) parts.Add("舵机");
        if ((e & (ushort)EnableBits.Motor) != 0) parts.Add("电机");
        return parts.Count == 0 ? "[全部失能]" : "[" + string.Join(" + ", parts) + "]";
    }

    // ==================================================================
    // 上行帧发送
    // ==================================================================
    private bool SendFrame(byte type, byte[] payload)
    {
        if (_sock == null) return false;
        int pl = payload?.Length ?? 0;
        var f = new byte[Proto.UP_OVERHEAD + pl];
        f[0] = Proto.MAGIC_UP_0; f[1] = Proto.MAGIC_UP_1;
        f[2] = type;
        Proto.PutU16(f, 3, (ushort)pl);
        if (pl > 0) Buffer.BlockCopy(payload, 0, f, 5, pl);
        Proto.PutU16(f, 5 + pl, Proto.Crc16(f, 2, 3 + pl));
        Array.Copy(Proto.TAIL, 0, f, 7 + pl, 4);
        try
        {
            _sock.Send(f, 0, f.Length, SocketFlags.None);
            return true;
        }
        catch { return false; }
    }

    private void Ack(Cmd cmd, ushort token)
    {
        var pl = new byte[4];
        Proto.PutU16(pl, 0, token);
        pl[2] = (byte)cmd;
        pl[3] = 0;
        SendFrame((byte)FrameType.Ack, pl);
    }

    private void Nak(Cmd cmd, ushort token, byte err)
    {
        var pl = new byte[4];
        Proto.PutU16(pl, 0, token);
        pl[2] = (byte)cmd;
        pl[3] = err;
        SendFrame((byte)FrameType.Nak, pl);
    }

    /// <summary>模拟车端把路表按协议 dump 出去（车端 motor_rt_tick() 的镜像）</summary>
    private void SendRouteDump()
    {
        const int Chunk = 64;                       // = 车端 MOTOR_RT_CHUNK

        // --- 表头 32 B（字段偏移与车端 rt_head_t 逐项一致）---
        var hd = new byte[RouteProto.HeadBytes];
        Proto.PutU32(hd, 0, RouteProto.Magic);
        Proto.PutU16(hd, 4, RouteProto.Ver);
        Proto.PutU16(hd, 6, RouteProto.RecSize);
        Proto.PutU32(hd, 8, (uint)_routeX.Count);
        Proto.PutU32(hd, 12, 50u);                 // ds_mm（模拟车端用 0.05 m）
        double lap = _routeX.Count * 0.05;
        Proto.PutU32(hd, 16, (uint)(lap * 1000));
        Proto.PutU32(hd, 20, 0u);                  // origin_x_mm
        Proto.PutU32(hd, 24, 0u);                  // origin_y_mm
        Proto.PutU16(hd, 28, 0);                   // origin_yaw_cdeg
        Proto.PutU16(hd, 30, (ushort)(_routeX.Count >= 1500 ? 1 : 0));
        SendFrame((byte)FrameType.RouteHead, hd);

        // --- 数据块 ---
        for (int first = 0; first < _routeX.Count; first += Chunk)
        {
            int n = Math.Min(Chunk, _routeX.Count - first);
            var pl = new byte[6 + n * RouteProto.RecSize];
            Proto.PutU32(pl, 0, (uint)first);
            Proto.PutU16(pl, 4, (ushort)n);
            for (int k = 0; k < n; k++)
            {
                int o = 6 + k * RouteProto.RecSize;
                Proto.PutU32(pl, o, (uint)(int)Math.Round(_routeX[first + k] * 1000.0));
                Proto.PutU32(pl, o + 4, (uint)(int)Math.Round(_routeY[first + k] * 1000.0));
                Proto.PutU16(pl, o + 8, 0);        // yaw_cdeg（模拟侧不记航向）
                Proto.PutU16(pl, o + 10, 0);       // v_cmps
            }
            SendFrame((byte)FrameType.RouteData, pl);
        }

        // --- 结束帧 ---
        var ed = new byte[8];
        Proto.PutU32(ed, 0, (uint)_routeX.Count);
        Proto.PutU32(ed, 4, (uint)(lap * 1000));
        SendFrame((byte)FrameType.RouteEnd, ed);

        Log?.Invoke($"模拟车端：已 dump 路表 {_routeX.Count} 点（{(_routeX.Count + Chunk - 1) / Chunk + 2} 帧）");
    }

    private void SendParamValue(byte id, float v)
    {
        var pl = new byte[5];
        pl[0] = id;
        Proto.PutF32(pl, 1, v);
        SendFrame((byte)FrameType.ParamValue, pl);
    }

    private void SendParamTable()
    {
        foreach (var p in _params)
        {
            // ID(1) + LO(4) + HI(4) + DEF(4) + STEP(4) + UNIT(8) + NAME(16) + LABEL(32) = 73
            var pl = new byte[73];
            pl[0] = p.Id;
            Proto.PutF32(pl, 1, p.Lo);
            Proto.PutF32(pl, 5, p.Hi);
            Proto.PutF32(pl, 9, p.Def);
            Proto.PutF32(pl, 13, p.Step);
            Proto.PutStr(pl, 17, 8, p.Unit);
            Proto.PutStr(pl, 25, 16, p.Name);
            Proto.PutStr(pl, 41, 32, p.Group + "|" + p.Label);
            SendFrame((byte)FrameType.ParamMeta, pl);
        }
        Log?.Invoke($"已下发参数表（{_params.Count} 项）");
    }

    // ==================================================================
    // 遥测数据生成
    // ==================================================================
    private byte[] BuildTelemetry()
    {
        // ★ 档案可能在运行期切换（卡丁车 32 / 单电机 20），长度对不上就重分配 ——
        //   否则模拟车端会按旧档案的长度发帧（以前就是按构造时的 32 发，切档案后对不上）。
        if (_ch.Length != Defaults.ChannelNames.Length) _ch = new float[Defaults.ChannelNames.Length];
        var ch = _ch;
        Array.Clear(ch, 0, ch.Length);

        // ⚠⚠ **每个档案都必须有自己的分支**。最后那支 FillKart 只认"卡丁车"那套下标，
        //   别的档案落进去就是"数据一套、通道名另一套"：轻则图例/数值框全错位
        //   （2026-10-13 现场：遥控档案的"速度"格读到了 ch15 = 换向次数），
        //   重则**越界**（标定档案只有 16 个通道，FillKart 会写到 ch[30] 直接抛异常）。
        //   ⇒ 兜底换成"全 0 + 日志"：宁可显示"没数据"，也不要显示"错数据"。
        //   ⚠ 新增档案时，这里和 Defaults.ValueBoxes() 必须同时加一支（SelfTest 会挨个跑一遍）。
        if (Defaults.IsMotor) FillMotor(ch);
        else if (Defaults.IsDualMotor) FillDualMotor(ch);
        else if (Defaults.IsSteerLoop) FillSteerLoop(ch);
        else if (Defaults.IsInsRoute) FillInsRoute(ch);
        else if (Defaults.IsKartRC) FillKartRC(ch);
        else if (Defaults.IsCalib) FillCalib(ch);
        else if (Defaults.IsKart) FillKart(ch);
        else if (_warnedNoFill != Defaults.ProfileDisplayName)
        {
            _warnedNoFill = Defaults.ProfileDisplayName;
            Log?.Invoke($"[模拟] 档案「{_warnedNoFill}」没有专用造数分支 —— 该档案全 0（请在 Simulator 里补一支）");
        }

        // ---- 组帧 ----
        int cnt = ch.Length;
        int pl = Proto.TELEM_OVERHEAD + cnt * 4;
        var p = new byte[pl];
        Proto.PutU16(p, 0, (ushort)(FramesSent & 0xFFFF));
        Proto.PutU32(p, 2, (uint)(_t * 1000));
        // VALID_MASK 是**哨兵值**（全 1 = 全部有效），不是逐通道位图 —— 通道数已超过 16，
        // 位图根本表达不下。车端 motor_link.c 也发 0xFFFF，这里跟它对齐（以前发 0x0FFF 是笔误）。
        Proto.PutU16(p, 6, 0xFFFF);
        Proto.PutU16(p, 8, (ushort)cnt);
        for (int c = 0; c < cnt; c++) Proto.PutF32(p, 10 + c * 4, ch[c]);
        return p;
    }

    /// <summary>单电机档案：完全照 firmware 的 motor_drv / motor_link 语义造数。</summary>
    private void FillMotor(float[] ch)
    {
        const float dt = 0.02f;                     // 模拟车端固定 50 Hz 步进

        bool master = (Enable & (ushort)EnableBits.Master) != 0;
        bool motorEn = (Enable & (ushort)EnableBits.Motor) != 0;

        // ---- 目标：与车端 motor_policy_target 逐条对齐 ----
        //   ① 使能不全 → 0（Motor 与 Master 都要置位）
        //   ② 上位机从没说过话 → 0
        //   ③ 距上次下行帧超过 lost_ms → 失联 → 0（这条以前漏了，见 Online 的注释）
        float lostMs   = Math.Clamp(Prm("lost_ms"), 100f, 3000f);
        bool  autoStop = Prm("auto_stop") > 0.5f;

        Online = _everRx && (_t - _lastCmdT) * 1000.0 <= lostMs;

        // rawCmd = 参数本身（ch[0] 报的就是它，真固件 CH_DUTY_CMD 也是取参数值）；
        // 下面几条只决定"实际输出"，不动 ch[0]。
        float rawCmd = Prm("duty_cmd");

        float cmd = (master && motorEn) ? rawCmd : 0f;
        if (!_everRx) cmd = 0f;
        if (autoStop && !Online) cmd = 0f;

        float lim = Math.Clamp(Prm("duty_limit"), 0f, 100f);
        if (lim <= 0f) lim = 100f;
        if (cmd > lim) cmd = lim;
        if (cmd < -lim) cmd = -lim;

        // ---- 斜坡 ----
        float ramp = Math.Max(0f, Prm("ramp_ms"));
        if (ramp > 0f)
        {
            // dt 是秒、ramp 是毫秒 → ×1000；全行程（-100%→+100%）是 200 个百分点
            float step = 200f * dt * 1000f / ramp;
            float err = cmd - _mAct;
            if (err > step) err = step;
            if (err < -step) err = -step;
            _mAct += err;
        }
        else
        {
            _mAct = cmd;
        }

        // ---- 死区补偿 ----
        float dz = Math.Clamp(Prm("deadzone"), 0f, 20f);
        float abs = Math.Abs(_mAct);
        float outs = abs < 0.05f ? 0f : (dz > 0f ? dz + abs * (100f - dz) / 100f : abs);
        if (outs > 100f) outs = 100f;
        float act = _mAct >= 0f ? outs : -outs;

        ch[0] = rawCmd;                                     // duty_cmd（参数原值，不受门控影响）
        ch[1] = act;                                        // duty_act
        ch[2] = cmd >= 0f ? 1f : 0f;                        // dir
        ch[3] = Math.Abs(act) * 100f;                       // pwm_raw（PWM_DUTY_MAX=10000）
        ch[4] = Enable;                                     // enable bits
        ch[5] = Online ? 1f : 0f;                           // online（真建模，不再写死 1）
        ch[6] = (float)((_t - _lastCmdT) * 1000.0);         // rx_age ms（距上次下行帧）
        ch[7] = lim;                                        // duty_limit
        ch[8] = ramp;                                       // ramp_ms
        ch[9] = (FramesSent % 100000f);                     // tx_count
        ch[10] = (float)_t;                                 // uptime s
        ch[11] = 0f;                                        // init_err

        // ---- 磁编码器：模拟"匀速转动的磁铁"（15 位 / 32768 一圈，约 11 s 转一圈）----
        float ang15 = (float)((_t * 3000.0) % 32768.0);
        ch[12] = ang15;                                     // mag_raw
        ch[13] = ang15;                                     // mag_angle
        ch[14] = ang15 * 360f / 32768f;                     // mag_deg
        ch[15] = 3000f * 0.02f;                             // mag_delta（20ms 走这么多）

        // ---- 2026-10-01 新增：角速度 / 积分 / 圆上最短差 / 毛刺 ----
        ch[16] = 3000f / 16384f * 6.2831853f;               // mag_speed（rad/s，与上面 3000 计数/s 自洽）
        _mSum += ch[16] * 0.02 * 1000.0;                    // mag_sum（mrad，按 20ms 累加）
        ch[17] = (float)_mSum;
        ch[18] = 3000f * 0.02f;                             // mag_delta_w（模拟器不跨零 → 与 mag_delta 相同）
        ch[19] = 0f;                                        // mag_glitch（模拟器无噪声）
    }

    private float _mAct;                                    // 单电机档案：斜坡后的实际占空比

    // ---- 双电机速度环档案的物理状态（独立字段，别借用 _ch 存状态）----
    private double _vL, _vR;                                // 实测转速 rps
    private double _zL, _zR;                                // 积分状态 rps·s
    private double _posL, _posR;                            // 带符号累计计数
    private float  _uPrevL, _uPrevR;                         // 上一拍输出（换向检测用）
    private int    _flipL, _flipR;                           // 换向丢拍累计

    // ---- 转向闭环档案的物理状态（转向位置环，独立于后轮那套）----
    private double _steerAng;                                // 转向实测转角（单位: 方向盘度, 向右为正）
    private float  _steerApplied;                            // 目标斜坡之后的实际施加目标（度）
    private bool   _steerEnPrev;                             // 上一拍的内环使能（判 0→1 上升沿）

    /// <summary>
    /// 双电机速度环档案：照 speed_loop 的 LQI 语义造数。
    ///   u = u_ff + k_v·(r−v) + k_z·z，u_ff = r/K + u0·sign(r)，z 为条件积分；
    ///   目标先过"使能 / 收到过命令 / 失联自停"三道门控，再看被控对象（K·u 的一阶跟随）。
    /// 目的只是让界面/波形能验证，**不代表真实车辆动态**（与 FillKart/FillMotor 同性质）。
    /// </summary>
    private void FillDualMotor(float[] ch)
    {
        const float dt   = 0.02f;                  // 模拟车端固定 50 Hz 步进
        const float tau  = 0.06f;                  // 被控对象时间常数
        const float cpr  = 1024f;                  // 每圈计数（脉冲+方向编码器）

        bool master  = (Enable & (ushort)EnableBits.Master) != 0;
        bool motorEn = (Enable & (ushort)EnableBits.Motor) != 0;

        float lostMs   = Math.Clamp(Prm("lost_ms"), 100f, 3000f);
        bool  autoStop = Prm("auto_stop") > 0.5f;

        Online = _everRx && (_t - _lastCmdT) * 1000.0 <= lostMs;

        // ---- 门控：使能不全 / 从没收过命令 / 失联 → 有效目标 = 0 ----
        bool gate = master && motorEn && _everRx && !(autoStop && !Online);

        // ch[0]/ch[3] 报的是参数原值（滑条下发的目标），下面的门控只影响"实际怎么走"
        float cmdL = Prm("target_l");
        float cmdR = Prm("target_r");
        float rL = gate ? cmdL : 0f;
        float rR = gate ? cmdR : 0f;

        // ---- 控制器系数 ----
        float invK = Prm("inv_K"); if (invK <= 1e-6f) invK = 0.005f;
        float K    = 1f / invK;                    // 稳态增益 rps / 单位占空比
        float u0   = Prm("u0");
        float kV   = Prm("k_v");
        float kZ   = Prm("k_z");
        float uMax = Math.Clamp(Prm("duty_limit"), 0f, 100f) / 100f;   // 参数是 % → 归一化
        if (uMax <= 0f) uMax = 0.3f;

        float ffL = 0f, ffR = 0f, uL = 0f, uR = 0f;
        if (gate)
        {
            ffL = rL * invK + (Math.Abs(rL) > 0.05f ? u0 * Math.Sign(rL) : 0f);
            ffR = rR * invK + (Math.Abs(rR) > 0.05f ? u0 * Math.Sign(rR) : 0f);
            float eL = rL - (float)_vL;
            float eR = rR - (float)_vR;

            uL = ffL + kV * eL + kZ * (float)_zL;
            uR = ffR + kV * eR + kZ * (float)_zR;
            bool satL = uL > uMax || uL < -uMax;
            bool satR = uR > uMax || uR < -uMax;
            if (uL > uMax) uL = uMax; else if (uL < -uMax) uL = -uMax;
            if (uR > uMax) uR = uMax; else if (uR < -uMax) uR = -uMax;

            // 条件积分抗饱和：饱和且误差同向 → 冻结积分（与固件同一条规则）
            if (!(satL && Math.Sign(eL) == Math.Sign(uL))) _zL += eL * dt;
            if (!(satR && Math.Sign(eR) == Math.Sign(uR))) _zR += eR * dt;
        }

        // ---- 被控对象：K·u − v 的一阶跟随 ----
        float a = (float)Math.Min(1.0, dt / tau);
        _vL += (K * uL - (float)_vL) * a;
        _vR += (K * uR - (float)_vR) * a;

        // ---- 计数增量：换向那一拍丢弃（DIR 在两次采样间变过，与固件口径一致）----
        float dltL = (float)(_vL * cpr * dt);
        float dltR = (float)(_vR * cpr * dt);
        if (_uPrevL != 0f && Math.Sign(uL) != Math.Sign(_uPrevL)) { _flipL++; dltL = 0f; }
        if (_uPrevR != 0f && Math.Sign(uR) != Math.Sign(_uPrevR)) { _flipR++; dltR = 0f; }
        _uPrevL = uL; _uPrevR = uR;
        _posL += dltL; _posR += dltR;

        ch[0]  = cmdL;                             // target_l（参数原值，不受门控影响）
        ch[1]  = (float)_vL;                       // v_l（滤波后实测）
        ch[2]  = uL * 100f;                        // u_l（实际输出占空比 %，带符号）
        ch[3]  = cmdR;                             // target_r
        ch[4]  = (float)_vR;                       // v_r
        ch[5]  = uR * 100f;                        // u_r
        ch[6]  = ffL * 100f;                       // uff_l
        ch[7]  = ffR * 100f;                       // uff_r
        ch[8]  = (float)_zL;                       // z_l
        ch[9]  = (float)_zR;                       // z_r
        ch[10] = (float)_posL;                     // pos_l
        ch[11] = (float)_posR;                     // pos_r
        ch[12] = dltL;                             // dlt_l
        ch[13] = dltR;                             // dlt_r
        ch[14] = _flipL;                           // flip_l
        ch[15] = _flipR;                           // flip_r
        ch[16] = Online ? 1f : 0f;                 // online
        ch[17] = (float)((_t - _lastCmdT) * 1000.0); // rx_age ms
        ch[18] = Enable;                           // enable 位图
        ch[19] = 0f;                               // safety 位图（模拟器不产生保护事件）
    }

    /// <summary>
    /// 转向闭环档案（工程 7.steer_loop_wifi_cyt4bb7）：
    /// 后轮速度环与「双电机」档案**逐位一致**（直接复用 FillDualMotor），
    /// 额外补 20~25 的转向组 —— 转向位置环 + 绝对角度编码器。
    ///
    /// 车端事实（对应 steer_ctrl.c / steer_abs.c）：
    ///   · 位置式 PID（积分**不乘 dt**，同北极星队 kart_pid.c 口径）
    ///   · 输出是**带符号 duty**（−10000~10000）
    ///   · 绝对角 raw 0~4095；delta 是"已减中位、已解环绕"的计数
    ///   · steer_status 的 bit0 = 自检通过（0 = 失败）
    /// 这里只做一阶速率受限的简化模型，够跑通界面与自检，不追求复刻静摩擦等细节。
    /// </summary>
    private void FillSteerLoop(float[] ch)
    {
        FillDualMotor(ch);                         // 0~19 与双电机档案完全一致

        const float dt      = 0.02f;
        const float vMaxDeg = 90f;                 // 满偏时机构角速度上限（度/秒）

        bool  en     = Prm("steer_en") > 0.5f;
        float kp     = Prm("steer_kp");
        float u0     = Prm("steer_u0");
        float db     = Math.Max(0f, Prm("steer_db"));
        float slew   = Math.Max(0f, Prm("steer_slew"));
        float limit  = Math.Max(1f, Prm("steer_lim"));
        float outMax = Math.Clamp(Prm("steer_umax"), 0f, 10000f);
        if (outMax <= 0f) outMax = 9000f;      // 与车端 STEER_U_MAX_DEFAULT 同档（2026-10-15 起 9000）

        // ★ 0→1 上升沿：用"当前实测角"重置目标 —— 复刻固件 steer_ctrl_set_angle_enable() 的行为。
        //   车端参数变量就是固件变量（`&steer.target_delta`），所以这里直接改参数值，与真机一致：
        //   **先给目标再使能，那个目标会被吞掉** —— 这是防大跳变的保护，不是 bug。
        if (en && !_steerEnPrev)
        {
            var tgt = _params.Find(x => x.Name == "steer_tgt");
            if (tgt != null) tgt.Value = (float)_steerAng;
            _steerApplied = (float)_steerAng;
        }
        _steerEnPrev = en;

        // ---- 目标软限位（每拍补夹）+ 斜坡（与固件 update 里"目标处理在门控之前"一致）----
        //   ⚠ 分清两个 target：targetRaw 是"上位机写的值（夹过限位）"，对应固件的 steer.target_delta；
        //      `_steerApplied` 是**实际施加**的目标（软限位夹紧 + 斜坡之后）= 固件的 steer.applied_delta。
        //   ★★ 2026-10-14 起遥测 ch[20] 发的是 `_steerApplied`（照队友把 Servo_Target_Applied_Deg
        //      单独导出去的做法）：斜坡开着时若发命令本身，波形上是一条干净阶跃，
        //      会让人把"斜坡滞后"算到机械/增益头上。`steer_slew = 0`(默认) 时两者相等。
        float targetRaw = Math.Clamp(Prm("steer_tgt"), -limit, limit);
        float target    = targetRaw;

        float u = 0f;
        if (en)
        {
            if (slew > 0f)
            {
                float lim = slew * dt;
                if (target > _steerApplied + lim) target = _steerApplied + lim;
                if (target < _steerApplied - lim) target = _steerApplied - lim;
            }
            _steerApplied = target;

            float e = target - (float)_steerAng;

            // 越限保护：实测已超软限位、且误差还指向限位方向 -> 本拍不出力
            if (((_steerAng >  limit) && (e > 0f)) ||
                ((_steerAng < -limit) && (e < 0f))) e = 0f;

            // 误差死区
            if (e < db && e > -db) e = 0f;

            // ★ 位置环 + 静摩擦前馈（2026-10-07 换的控制律）: u = kp*e + u0*sign(e)
            u = kp * e + (e > 0f ? u0 : (e < 0f ? -u0 : 0f));
            if (u > outMax) u = outMax; else if (u < -outMax) u = -outMax;

            // 被控对象：把 duty 当速度指令，推到限速就停（静摩擦细节不复刻，够跑界面与自检）
            float v = u / outMax * vMaxDeg;
            _steerAng += v * dt;
            if (_steerAng >  32f) _steerAng =  32f;
            if (_steerAng < -32f) _steerAng = -32f;
        }
        else
        {
            _steerApplied = targetRaw;             // 不使能 -> 斜坡状态跟住目标，输出 0
        }

        // raw 映射：deg = -(raw - 2157)/50.08  =>  raw = 2157 - deg*50.08
        int raw = (int)Math.Round(2157.0 - _steerAng * 50.08);
        raw = ((raw % 4096) + 4096) % 4096;

        ch[20] = _steerApplied;                    // steer_tgt：★ 实际施加目标（软限位+斜坡之后），非命令
        ch[21] = (float)_steerAng;                 // steer_delta（度）
        ch[22] = u;                                // steer_u（带符号 duty）
        ch[23] = raw;                              // steer_raw 0~4095
        ch[24] = en ? 1f : 0f;                     // steer_en
        ch[25] = 1f;                               // steer_status：bit0=1 = 自检通过
    }

    /// <summary>
    /// 卡丁·遥控档案（工程 9 全量 39 通道）：0~25 与「转向闭环」**逐位一致**（工程 9 = 工程 7 + 遥控），
    /// 所以直接复用 FillSteerLoop；26~36 是遥控组；37~38 是增益回读。
    ///
    /// ⚠ 遥控组（26~36）一律填 0：模拟器**不模拟接收机**（没有 SBUS、没有 CH5/CH6 开关），
    ///   编一组"看起来在接管"的假值比不填更容易误导现场 —— 0 = 没插接收机 / 遥控 OFF，
    ///   与车端 `remote_ctrl` 的 FAILSAFE 语义一致（`rc_frames` 不涨就是没在发）。
    ///
    /// ★ 回读的 k_v / k_z 填的是**模拟参数面板里当前的值** —— 真机上这两个通道就是这么来的
    ///   （`spd_g.k_v` = 上位机下发、车端钳位之后的值）。这样在模拟器里拖滑条，
    ///   回读通道会跟着动，"滑条值 != 生效值"这件事在**没有车**的时候也能看见。
    /// </summary>
    private void FillKartRC(float[] ch)
    {
        // 39 = 26(转向闭环) + 11(遥控组) + 2(增益回读)。表变了就当场说清楚，别静默错位。
        if (ch.Length < 39)
        {
            Array.Clear(ch, 0, ch.Length);
            Log?.Invoke($"[模拟] KartRC 档案通道数异常（{ch.Length} < 39）—— 本帧全 0");
            return;
        }

        FillSteerLoop(ch);                                     // 0~25

        // 26~36：没有接收机就是 0。⚠ 只清这一段，别顺手把 37/38 的回读值也清了。
        for (int i = 26; i <= 36 && i < ch.Length; i++) ch[i] = 0f;

        ch[37] = Prm("k_v");                                   // CH_K_V 本拍生效的比例增益
        ch[38] = Prm("k_z");                                   // CH_K_Z 本拍生效的积分增益
    }

    /// <summary>
    /// 标定·精简档案（工程 9 的 `tel_sel = 1`，18 通道）：就是"转向闭环"那套状态
    /// **按车端 CT_* 的 18 个下标重新排一遍**。
    ///
    /// 先算进 26 长的暂存区再搬家，而不是另写一份动力学 —— 这样两条档案的仿真
    /// （K·u 一阶跟随、条件积分抗饱和、换向丢拍）**永远是同一份代码**，不会各自漂移。
    /// ⚠ index 必须与车端 motor_link.h 第五节的 CT_* 一致，改一处就要改两处。
    /// </summary>
    private void FillCalib(float[] ch)
    {
        if (ch.Length < 18)
        {
            Array.Clear(ch, 0, ch.Length);
            Log?.Invoke($"[模拟] Calib 档案通道数异常（{ch.Length} < 18）—— 本帧全 0");
            return;
        }

        FillSteerLoop(_clBuf);                             // 26 长暂存：0~25 那套布局

        ch[0]  = _clBuf[0];                                // CT_TARGET_L   目标转速 L(rps)
        ch[1]  = _clBuf[1];                                // CT_V_L        实测转速 L
        ch[2]  = _clBuf[2];                                // CT_U_L        输出占空比 L(%)
        ch[3]  = _clBuf[3];                                // CT_TARGET_R
        ch[4]  = _clBuf[4];                                // CT_V_R
        ch[5]  = _clBuf[5];                                // CT_U_R
        ch[6]  = _clBuf[20];                               // CT_STEER_TGT  目标转角(°)
        ch[7]  = _clBuf[21];                               // CT_STEER_DELTA 实测转角(°)
        ch[8]  = _clBuf[22];                               // CT_STEER_U    舵机输出(duty)
        ch[9]  = _clBuf[16];                               // CT_ONLINE     链路在线
        ch[10] = _clBuf[18];                               // CT_ENABLE     使能位图
        ch[11] = _clBuf[19];                               // CT_SAFETY     保护位图
        ch[12] = _clBuf[24];                               // CT_STEER_EN   转向内环使能
        ch[13] = _clBuf[25];                               // CT_STEER_ST   舵机状态(bit0 自检 / bit1,2 锁存)
        ch[14] = 0f;                                       // CT_OL_EN      模拟器不模拟开环模式
        ch[15] = 0f;                                       // CT_RESV0      预留
        ch[16] = Prm("k_v");                               // CT_K_V        本拍生效的比例增益
        ch[17] = Prm("k_z");                               // CT_K_Z        本拍生效的积分增益
    }

    /// <summary>
    /// 自检用：按**当前档案**造一帧通道值（不发送、不推进时间、不改状态机）。
    /// 用来断言"每个档案都有专用造数分支、且长度对得上" —— 这类错位不抛异常时最难查。
    /// </summary>
    internal float[] TestFillOnce()
    {
        var buf = new float[Defaults.ChannelNames.Length];
        if (Defaults.IsMotor) FillMotor(buf);
        else if (Defaults.IsDualMotor) FillDualMotor(buf);
        else if (Defaults.IsSteerLoop) FillSteerLoop(buf);
        else if (Defaults.IsInsRoute) FillInsRoute(buf);
        else if (Defaults.IsKartRC) FillKartRC(buf);
        else if (Defaults.IsCalib) FillCalib(buf);
        else FillKart(buf);
        return buf;
    }

    // ==================================================================
    // 惯导 + 路表档案（工程 8.ins_route_cyt4bb7，C 车）
    //
    // 这一段刻意**不用随机游走糊一条假曲线**，而是把"左右轮实测转速"当成真的车速去积分：
    //      v = (v_l + v_r)/2 × 米每转          ω = (v_r − v_l) / 轮距
    //      yaw += ω·dt  →  x += −sin(yaw)·ds  →  y += cos(yaw)·ds
    // 这样拖 target_l / target_r 滑条就能看到车真的走/转，**和车端 ins_odom.c 是同一套公式**，
    // 也才能用自检断言"路表点数是真在长、位姿是真在动"。
    // ==================================================================
    private void FillInsRoute(float[] ch)
    {
        FillDualMotor(ch);                         // 0~19 与双电机档案完全一致（含 5ms 口径的轮速）

        const double dt = 0.02;                    // 模拟车端 50 Hz 步进
        const double WHEELBASE = 0.16;             // m，小模型车近似轮距（只影响模拟转弯半径，不影响真机）

        // ---------- 舵机（20~22）----------
        bool  srvEn = Prm("srv_en") > 0.5f;
        float norm  = srvEn ? Math.Clamp(Prm("srv_norm"), -1f, 1f) : 0f;
        _srvApplied = norm;

        // duty 口径与 steer_srv.h 一致：中位 2420；**左右半程不对称**（左 240 / 右 260）
        float duty = norm >= 0f ? 2420f + norm * 240f : 2420f + norm * 260f;
        ch[20] = _srvApplied;
        ch[21] = duty;
        ch[22] = srvEn ? 1f : 0f;

        // ---------- 三个边沿命令（复刻车端"只在 0→1 / 1→0 那一拍动手"的语义）----------
        bool zeroNow   = Prm("ins_zero")   > 0.5f;
        bool routeNow  = Prm("ins_route")  > 0.5f;
        bool magcalNow = Prm("ins_magcal") > 0.5f;

        if (zeroNow && !_zeroPrev)
        {
            _insX = _insY = _insYaw = _insDist = 0.0;
            _routeX.Clear(); _routeY.Clear(); _routeNextAt = 0;
        }
        _zeroPrev = zeroNow;

        if (routeNow != _routePrev)                // 0→1 开、1→0 停
        {
            _routeRunning = routeNow;
            if (routeNow)
            {
                // 起点一定是第 0 点（车端 ins_route_start 就是这么做的）。
                // ★ 必须先锁存起点位姿：车端存的是【相对起点】的坐标，
                //   所以第 0 点恒为 (0,0)。若直接存世界坐标，上位机就无法把
                //   路表和实时轨迹叠在一起比较（两者原点不同）。
                _routeX.Clear(); _routeY.Clear();
                _rtX0 = _insX; _rtY0 = _insY; _rtYaw0 = _insYaw;
                _routeX.Add(0.0); _routeY.Add(0.0);
                _routeNextAt = _insDist + 0.05;    // 车端阈值 INS_ROUTE_DIST_THRESH
            }
        }
        _routePrev = routeNow;

        if (magcalNow != _magcalPrev)              // 0→1 开始、1→0 结算
        {
            if (!magcalNow) _magDone = true;       // 结算过了 -> 磁修正生效（车端置 INS_ST_MAG_CAL_DONE）
        }
        _magcalPrev = magcalNow;

        // ---------- 惯导：用实测轮速积分 ----------
        float mpm    = Prm("ins_mpm");
        if (mpm <= 0f) mpm = 1f;
        bool  odomEn = Prm("ins_odom_en") > 0.5f;

        double vL = ch[1] * mpm;                   // m/s
        double vR = ch[4] * mpm;
        double v  = (vL + vR) * 0.5;
        double w  = (vR - vL) / WHEELBASE;         // rad/s
        double ds = v * dt;

        if (odomEn)
        {
            _insYaw += w * 180.0 / Math.PI * dt;
            double rad = _insYaw * Math.PI / 180.0;
            _insX    += -Math.Sin(rad) * ds;
            _insY    +=  Math.Cos(rad) * ds;
            _insDist += Math.Abs(ds);
        }

        // ---------- 路表录制（阈值 0.05 m，与车端一致）----------
        if (_routeRunning && _insDist >= _routeNextAt)
        {
            {
                double a = -_rtYaw0 * Math.PI / 180.0;      // 车端：x' = cos(a0)dx + sin(a0)dy
                double c = Math.Cos(a), sn = Math.Sin(a);
                double dx = _insX - _rtX0, dy = _insY - _rtY0;
                _routeX.Add(c * dx + sn * dy);
                _routeY.Add(-sn * dx + c * dy);
            }
            _routeNextAt = _insDist + 0.05;
            if (_routeX.Count >= 1500) _routeRunning = false;   // 车端 MAX_POINTS，录满即停
        }

        // ---------- 填通道（23~31）----------
        // 姿态：给一点小噪声，静态时看着像真的（roll/pitch 只在坡道/侧倾时才有意义）
        float roll  = (float)(Math.Sin(_t * 0.7) * 0.8 + (_rnd.NextDouble() - 0.5) * 0.15);
        float pitch = (float)(Math.Sin(_t * 0.5) * 0.6 + (_rnd.NextDouble() - 0.5) * 0.15);

        ch[23] = roll;
        ch[24] = pitch;
        ch[25] = (float)(w * 180.0 / Math.PI + (_rnd.NextDouble() - 0.5) * 0.05);  // 陀螺 Z（°/s）
        ch[26] = (float)_insX;
        ch[27] = (float)_insY;
        ch[28] = (float)_insYaw;
        ch[29] = (float)_insDist;

        // 状态位图（与车端 main_cm7_0.c 的打包方式逐位一致；**用位或，不是加法**）
        uint fl = 0;
        if (odomEn)             fl |= 0x01;        // bit0 已就绪
        if (_magDone)           fl |= 0x02;        // bit1 磁已标定
        if (Math.Abs(v) < 0.01) fl |= 0x04;        // bit2 静止
        if (_routeRunning)      fl |= 0x10;        // bit4 路表录制中
        if (_routeX.Count >= 1500) fl |= 0x20;     // bit5 路表满
        if (!odomEn)            fl |= 0x80;        // bit7 告警（推算被关掉了）
        ch[30] = fl;
        ch[31] = _insSeq++;                        // 采样序号：应匀速增长（上位机用它统计丢帧）
    }

    private void FillKart(float[] ch)
    {
        float vMax = Math.Max(0.5f, Prm("v_max"));
        bool master = (Enable & (ushort)EnableBits.Master) != 0;
        bool motor = (Enable & (ushort)EnableBits.Motor) != 0;
        float vTgt = (master && motor) ? vMax : 0f;

        // ---- 速度一阶跟随 ----
        _vAct += (vTgt - _vAct) * (float)Math.Min(1.0, 6.0 * 0.02);
        _vAct += (float)(_rnd.NextDouble() - 0.5) * 0.02f;
        if (_vAct < 0) _vAct = 0;

        // ---- 航向：缓慢正弦（模拟车在路上来回摆）----
        double om = 0.45;
        double yawTgt = 35.0 * Math.Sin(om * _t);
        double yawAct = 35.0 * Math.Sin(om * _t - 0.25);   // 滞后 0.25 rad → 有跟踪误差
        double gyroZ = 35.0 * om * Math.Cos(om * _t);
        double biasZ = Prm("gyro_bias_z");

        ch[0] = (float)yawAct;
        ch[1] = (float)(gyroZ + biasZ);
        ch[2] = (float)biasZ;
        ch[3] = 1f;
        ch[4] = (float)_posX;
        ch[5] = (float)_posY;

        // ---- 编码器：1024 线，1 计数 ≈ 0.77 mm ----
        float cntPerSec = _vAct / 0.00077f;
        ch[6] = cntPerSec * Prm("enc_scale_L") * 0.001f;
        ch[7] = cntPerSec * Prm("enc_scale_R") * 0.0011f;

        // ---- 电机占空比 ----
        float duty = 100f * _vAct / vMax;
        ch[8] = duty; ch[9] = duty * 0.97f;
        ch[10] = duty; ch[11] = duty * 0.99f;

        // ---- 转向：由航向误差反推 ----
        double hdgErr = yawTgt - yawAct;
        float kp = Prm("steer_kp");
        float steerCmd = (float)Math.Clamp(-hdgErr * kp * 0.35, -25, 25);
        ch[12] = steerCmd;
        ch[13] = steepFilter(steerCmd, 0.82f) + (float)(_rnd.NextDouble() - 0.5) * 1.2f;

        ch[14] = vTgt;
        ch[15] = _vAct;

        _dist += _vAct * 0.02;
        _posX += _vAct * 0.02 * Math.Cos(yawAct * Math.PI / 180.0);
        _posY += _vAct * 0.02 * Math.Sin(yawAct * Math.PI / 180.0);
        ch[16] = (float)_dist;
        ch[17] = (float)_posX;
        ch[18] = (float)_posY;

        ch[19] = (float)yawTgt;
        ch[20] = (float)yawAct;
        ch[21] = (float)hdgErr;
        ch[22] = (float)(hdgErr * 0.02);

        ch[23] = (float)(hdgErr * 0.004);
        ch[24] = kp * ch[21];
        ch[25] = Prm("steer_ki") * ch[22];
        ch[26] = Prm("steer_kd") * ch[1];

        // ESKF 估计：故意给一点偏差，方便看出"估计 vs 真值"
        ch[27] = ch[17] * 0.995f;
        ch[28] = ch[18] * 1.004f;
        ch[29] = (float)(yawAct * 0.998);
        ch[30] = 4200f + (float)(_rnd.NextDouble() - 0.5) * 300f;
    }

    private float _steerPrev;
    private float steepFilter(float v, float k)
    {
        _steerPrev = v * (1 - k) + _steerPrev * k;
        return _steerPrev;
    }

    public void Dispose() => Stop();
}
