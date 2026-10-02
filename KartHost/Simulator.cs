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

    // ---- 模拟物理状态（独立字段，不要借用 _ch 存状态）----
    private double _t;
    private float _vAct;
    private double _dist, _posX, _posY;
    private double _lastSendT;
    private double _lastCmdT;                   // 最近一次收到下行帧的时刻（秒）
    private bool   _everRx;                     // 上位机开口说过话没有


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
            // 单电机 / 双电机档案没有 telemetry_hz 这个参数，直接给 50 Hz
            float hz = (Defaults.IsMotor || Defaults.IsDualMotor) ? 50f : Math.Clamp(Prm("telemetry_hz"), 5f, 200f);
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

        if (Defaults.IsMotor) FillMotor(ch);
        else if (Defaults.IsDualMotor) FillDualMotor(ch);
        else FillKart(ch);

        // ---- 组帧 ----
        int cnt = ch.Length;
        int pl = Proto.TELEM_OVERHEAD + cnt * 4;
        var p = new byte[pl];
        Proto.PutU16(p, 0, (ushort)(FramesSent & 0xFFFF));
        Proto.PutU32(p, 2, (uint)(_t * 1000));
        Proto.PutU16(p, 6, 0x0FFF);                 // VALID_MASK：通道数按档案给
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
