using System.IO.Ports;

namespace KartHost;

/// <summary>
/// 路表串口链路（PC 端）。协议与车端 <c>ins_route/code/route_link.c</c> 逐字节对应。
///
/// <para>走哪条线：**DAP 自带的虚拟串口**（车端 <c>UART_0</c>，PC 侧一般显示为 COM33）。
/// 为什么不做在 WiFi 上：车端 WiFi 下行那条路被三层限制卡死
/// （<c>motor_parse_rx()</c> 写死 LEN==8 / <c>motor_rxbuf[256]</c> / 模块单帧 ~200 B），
/// 而串口是自己的字节流，一层都没有。</para>
///
/// <para>帧：<c>A5 5A | type(1) | len(2,LE) | payload | crc16(2,LE)</c>，
/// crc16 = CRC16-MODBUS（0xFFFF / 0xA001）覆盖 type|len|payload。</para>
///
/// <para><b>槽的纪律</b>：槽 0 是"手推录制"的家，**下发只允许写 1..N-1**（车端协议层也会拒）。
/// 所以灌表不可能覆盖手推的那张表。</para>
/// </summary>
public sealed class RouteLink : IDisposable
{
    // ---- 帧类型（与 route_link.h 对齐，别改数值）----
    public const byte Sync0 = 0xA5, Sync1 = 0x5A;
    public const byte TPing = 0x01, TDirReq = 0x02, TLoadBegin = 0x03, TLoadData = 0x04;
    public const byte TLoadCommit = 0x05, TLoadAbort = 0x06, TSlotSel = 0x07, TSlotLoad = 0x08;
    public const byte TSlotRead = 0x09, TStepGo = 0x0A;
    public const byte TPong = 0x81, TDirData = 0x82, TAck = 0x85, TLoadDone = 0x86, TSlotData = 0x87;
    public const byte TStepDone = 0x88;

    public const int PayloadMax = 768;
    public const int ChunkMax = 512;

    public static readonly string[] ResultName =
    {
        "OK", "槽号越界/写了槽0", "尺寸超限", "offset 不连续(丢帧)", "镜像校验不过",
        "没先 LOAD_BEGIN", "正在写 Flash", "状态不允许",
    };

    private SerialPort _sp;
    private readonly List<(byte T, byte[] P)> _pending = new();
    private readonly byte[] _rxRaw = new byte[4096];

    /// <summary>被丢弃的字节（同步不上 / CRC 不过）。>0 说明线上有噪声。</summary>
    public int BadBytes { get; private set; }
    /// <summary>车端报回来的协议版本/槽数等（PING 之后有效）</summary>
    public int ProtoVer, SlotCount = 7, SlotPages = 13, RecSize = 12, MaxBytes = 24516;

    // ---- 能力位（车端 PONG v2 起）----
    //   ★ 为什么需要：命令表加一条命令最阴的失败是「上位机发了、车端不认识」——
    //     车端只回一个笼统的 ERR_STATE（"状态不允许"），现场根本看不出是固件旧。
    //     真实案例：点「上传」→「读槽被拒：状态不允许（cmd=0x09 detail=0）」反复出现，
    //     而 0x09 是后加的 ⇒ 板上跑的是旧 hex。有了能力位，OPEN 那一刻就能点名说清。
    public const uint CapDir = 1u << 0, CapLoad = 1u << 1, CapSlotSel = 1u << 2,
                      CapSlotRead = 1u << 3, CapStep = 1u << 4;
    public uint Caps { get; private set; }
    public bool CapsKnown { get; private set; }          // false = 车端还是 v1 PONG（8 B）
    public bool CanSlotRead => (Caps & CapSlotRead) != 0;
    public bool CanStep => (Caps & CapStep) != 0;

    /// <summary>转向阶跃测试写进哪一槽（与车端 ins_route.h 的 INS_ROUTE_STEP_SLOT 一致）</summary>
    public const int StepSlot = 4;
    /// <summary>基线段长度 ms（与车端 steer_step.h 的 SS_BASE_MS 一致）</summary>
    public const int StepBaseMs = 300;

    /// <summary>车端固件太旧时的统一话术（README/文档里的路径也写在这里，免得现场找不到）</summary>
    public const string StaleFirmwareHint =
        "车端固件比上位机旧。重刷 8.ins_route_cyt4bb7\\ins_route\\iar\\project_config\\Debug_m7_0\\Exe\\"
        + "cyt4bb7_cm_7_0.hex（★ 两个核都要刷），然后点「关闭」再「打开」让 PONG 重新握手。";

    /// <summary>PONG 是不是带能力位的 v2 格式（12 B）。纯函数 ⇒ 可以离线自检。</summary>
    public static bool PongHasCaps(byte[] pong) => pong != null && pong.Length >= 12;

    /// <summary>
    /// 从 PONG 载荷解析能力位。
    /// <para>⚠ 关键点：**v1 老固件（8 B）没有这个字段**，此时不能返回 0（那会误判成"什么都不会"），
    /// 而要按"1 版当时确实会什么"推断 —— DIR / LOAD / SLOT_SEL 都有，
    /// **SLOT_READ（0x09）没有**（它是之后才加的命令）。这条推断正好能在现场一次点破"固件旧"。</para>
    /// </summary>
    public static uint ParseCaps(byte[] pong)
        => PongHasCaps(pong) ? U32(pong, 8) : (CapDir | CapLoad | CapSlotSel);

    public bool IsOpen => _sp != null && _sp.IsOpen;
    public string Describe => _sp == null ? "未打开" : $"{_sp.PortName} @ {_sp.BaudRate}";

    // ================================================================ 打开 / 关闭
    public static string[] ListPorts()
    {
        var names = SerialPort.GetPortNames();
        Array.Sort(names, (a, b) =>
        {
            int x = int.TryParse(a.Replace("COM", ""), out var v) ? v : 0;
            int y = int.TryParse(b.Replace("COM", ""), out var w) ? w : 0;
            return x.CompareTo(y);
        });
        return names;
    }

    public void Open(string port, int baud)
    {
        Close();
        _sp = new SerialPort(port, baud)
        {
            ReadTimeout = 20,
            WriteTimeout = 1000,
            ReadBufferSize = 8192,
            WriteBufferSize = 8192,
        };
        _sp.Open();
        _pending.Clear();
        BadBytes = 0;
        CapsKnown = false; Caps = 0;      // 每次重开都要重新握手才知道车端会什么
    }

    public void Close()
    {
        try { _sp?.Close(); } catch { /* 拔线时可能抛，忽略 */ }
        _sp?.Dispose();
        _sp = null;
    }

    public void Dispose() => Close();

    // ================================================================ CRC / 组帧 / 拆帧
    public static ushort Crc16(ushort crc, byte[] b, int off, int len)
    {
        // ⚠ 这条算法在工程里有四份实现（车端 rl_crc16/rt_crc16/motor_crc16 + 本处 + tools/route_send.py）。
        //   必须算出同一个值，否则现象是"帧看着发出去了，车端永远不回 ACK"，而两边都自认为正确。
        for (int i = 0; i < len; i++)
        {
            crc ^= b[off + i];
            for (int k = 0; k < 8; k++)
                crc = (crc & 1) != 0 ? (ushort)((crc >> 1) ^ 0xA001) : (ushort)(crc >> 1);
        }
        return crc;
    }

    public static byte[] Build(byte type, byte[] payload)
    {
        int n = payload?.Length ?? 0;
        if (n > PayloadMax) throw new ArgumentException($"载荷 {n} 超过上限 {PayloadMax}");
        var buf = new byte[7 + n];
        buf[0] = Sync0; buf[1] = Sync1; buf[2] = type;
        buf[3] = (byte)(n & 0xFF); buf[4] = (byte)((n >> 8) & 0xFF);
        if (n > 0) Buffer.BlockCopy(payload, 0, buf, 5, n);
        ushort c = Crc16(0xFFFF, buf, 2, 3 + n);
        buf[5 + n] = (byte)(c & 0xFF);
        buf[6 + n] = (byte)(c >> 8);
        return buf;
    }

    private static ushort U16(byte[] b, int o) => (ushort)(b[o] | (b[o + 1] << 8));
    private static uint U32(byte[] b, int o) => (uint)(b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24));
    private static void PutU32(byte[] b, int o, uint v)
    {
        b[o] = (byte)v; b[o + 1] = (byte)(v >> 8); b[o + 2] = (byte)(v >> 16); b[o + 3] = (byte)(v >> 24);
    }

    private void Write(byte type, byte[] payload = null) => _sp.Write(Build(type, payload), 0, 7 + (payload?.Length ?? 0));

    /// <summary>读一次串口并拆帧，把完整的帧塞进 _pending。返回本次新收的帧数。</summary>
    private int Pump()
    {
        int got = 0;
        int avail;
        try { avail = _sp.BytesToRead; }
        catch { return 0; }
        while (avail > 0)
        {
            int n = Math.Min(avail, _rxRaw.Length);
            try { n = _sp.Read(_rxRaw, 0, n); }
            catch (TimeoutException) { break; }
            if (n <= 0) break;
            got += Feed(_rxRaw, n);
            try { avail = _sp.BytesToRead; } catch { break; }
        }
        return got;
    }

    private readonly List<byte> _acc = new();

    private int Feed(byte[] data, int len)
    {
        int frames = 0;
        for (int i = 0; i < len; i++) _acc.Add(data[i]);

        while (true)
        {
            // 找同步头（滑窗）
            int si = -1;
            for (int i = 0; i + 1 < _acc.Count; i++)
                if (_acc[i] == Sync0 && _acc[i + 1] == Sync1) { si = i; break; }

            if (si < 0)
            {
                if (_acc.Count > 1) { BadBytes += _acc.Count - 1; _acc.RemoveRange(0, _acc.Count - 1); }
                break;
            }
            if (si > 0) { BadBytes += si; _acc.RemoveRange(0, si); }
            if (_acc.Count < 5) break;

            int plen = _acc[3] | (_acc[4] << 8);
            if (plen > PayloadMax) { BadBytes++; _acc.RemoveAt(0); continue; }

            int total = 7 + plen;
            if (_acc.Count < total) break;

            var f = _acc.GetRange(0, total).ToArray();
            ushort c = Crc16(0xFFFF, f, 2, 3 + plen);
            if (c != U16(f, 5 + plen)) { BadBytes++; _acc.RemoveAt(0); continue; }

            _acc.RemoveRange(0, total);
            var pl = new byte[plen];
            Buffer.BlockCopy(f, 5, pl, 0, plen);
            _pending.Add((f[2], pl));
            frames++;
        }
        return frames;
    }

    /// <summary>等一帧指定类型。返回 null = 超时。</summary>
    private byte[] Wait(byte type, int timeoutMs)
    {
        var t0 = Environment.TickCount64;
        while (true)
        {
            for (int i = 0; i < _pending.Count; i++)
            {
                if (_pending[i].T == type)
                {
                    var p = _pending[i].P;
                    _pending.RemoveAt(i);
                    return p;
                }
            }
            if (Environment.TickCount64 - t0 >= timeoutMs) return null;
            Pump();
            if (_pending.Count == 0) Thread.Sleep(2);
        }
    }

    /// <summary>等 ACK；返回 (result, detail)。result=-1 超时。</summary>
    private (int Res, int Detail) WaitAck(byte cmd, int timeoutMs)
    {
        var p = Wait(TAck, timeoutMs);
        if (p == null || p.Length < 4) return (-1, 0);
        if (p[0] != cmd) return (-1, 0);
        return (p[1], U16(p, 2));
    }

    private static string AckText(byte cmd, int res, int detail) =>
        res < 0 ? "等 ACK 超时" : $"{(res < ResultName.Length ? ResultName[res] : "?")}（cmd=0x{cmd:X2} detail={detail}）";

    // ================================================================ 命令
    public string Ping()
    {
        Write(TPing);
        var p = Wait(TPong, 1200) ?? throw new Exception("没有 PONG。检查：① 车是否上电 ② 端口是不是 DAP 虚拟串口 ③ 波特率是否与 DEBUG_UART_BAUDRATE 一致 ④ 串口是否被别的程序占着");
        ProtoVer = p[0];
        SlotCount = p[1]; SlotPages = p[2]; RecSize = p[3]; MaxBytes = (int)U32(p, 4);

        // ★ v2 PONG = 12 B（多了 caps）；v1 = 8 B。
        //   v1 老固件没有能力位，但它当时**确实**会 DIR / LOAD / SLOT_SEL 这三组，
        //   而 SLOT_READ（0x09）是之后才加的 ⇒ 按"缺 SLOT_READ"推断，正好能提前拦住。
        CapsKnown = PongHasCaps(p);
        Caps = ParseCaps(p);

        string caps = CapsKnown
            ? (((Caps & CapDir) != 0 ? "DIR " : "") + ((Caps & CapLoad) != 0 ? "LOAD " : "")
             + ((Caps & CapSlotSel) != 0 ? "SEL " : "") + ((Caps & CapSlotRead) != 0 ? "READ " : "")
             + ((Caps & CapStep) != 0 ? "STEP " : "")).Trim()
            : "推断(无能力位)";

        string warn = CanSlotRead ? "" : "  ★ 车端不支持「读槽」⇒ 上传用不了，需重刷固件";
        if (CapsKnown && !CanStep)
        {
            warn += "  ★ 车端不支持「转向阶跃测试」(0x0A)⇒ 那个按钮用不了，需重刷固件";
        }

        return $"车端在：协议 v{ProtoVer}，槽 {SlotCount} 个 × {SlotPages} 页，记录 {RecSize} B，"
             + $"单槽上限 {MaxBytes} B，能力[{caps}]{warn}";
    }

    // ================================================================ 转向阶跃测试（0x0A / 0x88）
    /// <summary>
    /// 启动一次转向阶跃测试（内环辨识，Polaris 七步工序第 ① 步）。
    /// <para>⚠ 车会**真的动**：舵机按 amp 打、电机按 rps 转。先架空前轮或留足直道。</para>
    /// </summary>
    public void StepGo(double amp, int holdMs, double rps)
    {
        if (!CanStep)
            throw new Exception("车端不支持「转向阶跃测试」(0x0A STEP_GO)："
                + (CapsKnown ? $"PONG 的能力位里没有这一项（caps=0x{Caps:X}）。"
                             : "车端回的是 v1 PONG（8 B，没有能力位）⇒ 一定是旧固件。")
                + " " + StaleFirmwareHint);

        var pl = new byte[6];
        short a = (short)Math.Round(amp * 1000.0), s = (short)Math.Round(rps * 10.0);
        pl[0] = (byte)(a & 0xFF); pl[1] = (byte)((a >> 8) & 0xFF);
        pl[2] = (byte)(holdMs & 0xFF); pl[3] = (byte)((holdMs >> 8) & 0xFF);
        pl[4] = (byte)(s & 0xFF); pl[5] = (byte)((s >> 8) & 0xFF);
        Write(TStepGo, pl);

        var (r, d) = WaitAck(TStepGo, 1500);
        if (r != 0)
            throw new Exception("STEP_GO 被拒：" + AckText(TStepGo, r, d)
                + "（detail: 0=启动 1=幅度非法 2=hold 非法 3=正在跑）");
    }

    /// <summary>等车端把阶跃记录写完（0x88 STEP_DONE）。返回记录条数；-1 = 超时。</summary>
    public int WaitStepDone(int timeoutMs)
    {
        var p = Wait(TStepDone, timeoutMs);
        if (p == null) return -1;
        return (int)U32(p, 1);
    }

    public sealed class SlotInfo
    {
        public int Slot, Count, LapMm, DsMm, Tag;
        public uint CrcR;
        public bool Valid, Active;
        public string Text => Valid
            ? $"{Slot,2}   {Count,5} 点   {LapMm / 1000.0,8:F2} m   {DsMm,3} mm   0x{CrcR:X4}{(Active ? "   ← 当前" : "")}"
            : $"{Slot,2}   （空）{(Active ? "                    ← 当前" : "")}";
    }

    public List<SlotInfo> Dir(out int active, out int slotCount)
    {
        Write(TDirReq);
        var p = Wait(TDirData, 1500) ?? throw new Exception("没有 DIR_DATA");
        active = p[0];
        slotCount = p[1];
        var list = new List<SlotInfo>(slotCount);
        for (int k = 0; k < slotCount; k++)
        {
            int o = 2 + k * 20;
            list.Add(new SlotInfo
            {
                Valid = p[o] != 0,
                Slot = p[o + 1],
                Tag = 0,
                Count = (int)U32(p, o + 4),
                LapMm = (int)U32(p, o + 8),
                DsMm = (int)U32(p, o + 12),
                CrcR = U32(p, o + 16),
                Active = p[o + 1] == active,
            });
        }
        return list;
    }

    public void SlotSelect(int slot, bool persist, bool load)
    {
        Write(TSlotSel, new[] { (byte)slot, (byte)(persist ? 1 : 0), (byte)(load ? 1 : 0) });
        var (res, det) = WaitAck(TSlotSel, 4000);
        if (res != 0) throw new Exception("选槽被拒：" + AckText(TSlotSel, res, det));
    }

    public void SlotLoad(int slot)
    {
        Write(TSlotLoad, new[] { (byte)slot });
        var (res, det) = WaitAck(TSlotLoad, 4000);
        if (res != 0) throw new Exception("载入被拒：" + AckText(TSlotLoad, res, det));
    }

    /// <summary>下载：PC → 车（会把镜像校验一遍，落 Flash 后才回 LOAD_DONE）。</summary>
    public string Download(int slot, byte[] image, Action<double, string> progress)
    {
        if (slot <= 0 || slot >= SlotCount)
            throw new Exception($"槽 0 是手推录制的家，下发只允许 1..{SlotCount - 1}");
        if (image == null || image.Length < 48) throw new Exception("镜像太小，不像一张表");
        if (image.Length > MaxBytes) throw new Exception($"镜像 {image.Length} B 超过单槽上限 {MaxBytes} B");

        var begin = new byte[5];
        begin[0] = (byte)slot;
        PutU32(begin, 1, (uint)image.Length);
        Write(TLoadBegin, begin);
        var (res, det) = WaitAck(TLoadBegin, 3000);
        if (res != 0) throw new Exception("LOAD_BEGIN 被拒：" + AckText(TLoadBegin, res, det));
        progress?.Invoke(0.02, "已握手");

        int off = 0;
        while (off < image.Length)
        {
            int n = Math.Min(ChunkMax, image.Length - off);
            var pl = new byte[6 + n];
            PutU32(pl, 0, (uint)off);
            pl[4] = (byte)(n & 0xFF); pl[5] = (byte)((n >> 8) & 0xFF);
            Buffer.BlockCopy(image, off, pl, 6, n);

            bool ok = false;
            for (int attempt = 0; attempt < 5 && !ok; attempt++)
            {
                Write(TLoadData, pl);
                var (r2, d2) = WaitAck(TLoadData, 1500);
                if (r2 == 0) ok = true;
                else if (r2 == 3 || r2 == -1) continue;      // offset 不连续 / 超时 ⇒ 重发同一块
                else throw new Exception("LOAD_DATA 被拒：" + AckText(TLoadData, r2, d2));
            }
            if (!ok) throw new Exception($"offset={off} 重发 5 次仍失败（串口在丢字节 ⇒ 把块调小/换根线）");

            off += n;
            progress?.Invoke(0.05 + 0.80 * off / image.Length, $"{off}/{image.Length} B");
        }

        Write(TLoadCommit);
        var (r3, d3) = WaitAck(TLoadCommit, 5000);
        if (r3 != 0) throw new Exception("LOAD_COMMIT 被拒：" + AckText(TLoadCommit, r3, d3));
        progress?.Invoke(0.90, "车端正在写 Flash…");

        var done = Wait(TLoadDone, 25000);
        if (done == null) throw new Exception("COMMIT 已 ACK，但没等到 LOAD_DONE（Flash 写超时/页写失败）");
        progress?.Invoke(1.0, "完成");
        int gotSlot = done[0];
        uint cnt = U32(done, 1), lap = U32(done, 5);
        return $"落盘完成：槽 {gotSlot}，{cnt} 点，{lap / 1000.0:F2} m（已设为当前表 ⇒ 菜单 RtLoadF / Cruise 直接就是它）";
    }

    /// <summary>
    /// 丢弃所有"已解析但没用"的帧，并把串口里还没读走的字节也倒掉。
    /// <para>车端是**无状态拉模式**（"要哪块给哪块"）⇒ 重新发请求之前，任何在途的 SLOT_DATA
    /// 都必然是**上一块**的回复，留着只会让下一轮拿到错块。</para>
    /// </summary>
    public void FlushRx()
    {
        _pending.Clear();
        _acc.Clear();
        try
        {
            int a = _sp?.BytesToRead ?? 0;
            if (a > 0)
            {
                var junk = new byte[Math.Min(a, _rxRaw.Length)];
                _sp.Read(junk, 0, junk.Length);
            }
        }
        catch { /* 拔线/超时都忽略：反正马上要重新发请求 */ }
    }

    /// <summary>上传：车 → PC。分块拉整张镜像（车端无状态，流控全在这边）。</summary>
    public byte[] ReadSlot(int slot, Action<double, string> progress)
    {
        // ★ 先按能力位拦一刀 —— 不然会拿到一句笼统的"状态不允许"，看不出是固件旧。
        if (!CanSlotRead)
            throw new Exception("车端不支持「读槽」(0x09 SLOT_READ)："
                + (CapsKnown ? "PONG 的能力位里没有这一项（caps=0x" + Caps.ToString("X") + "）。"
                             : "车端回的是 v1 PONG（8 B，没有能力位）⇒ 一定是旧固件。")
                + " " + StaleFirmwareHint);

        var acc = new List<byte>(4096);
        int off = 0;
        int expected = -1;
        int retries = 0;

        while (true)
        {
            var pl = new byte[7];
            pl[0] = (byte)slot;
            PutU32(pl, 1, (uint)off);
            pl[5] = (byte)(ChunkMax & 0xFF); pl[6] = (byte)((ChunkMax >> 8) & 0xFF);

            byte[] data = null;
            string lastWhy = "?";

            // ★★ 每一块都要**独立重试** —— 这是这一版最重要的改动。
            //   车端无状态 ⇒ 重问同一个 offset 永远安全（不会重复追加、不会错位）。
            //   为什么非做不可：整张表 41 块，旧代码**任何一次抖动就让整条传输作废**，
            //   而"41 块里出一块岔子"的概率随长度线性增长 ⇒ **越长的表越读不出来**
            //   （现场：157 点的圆环一读就过，1742 点的整圈必失败）。
            //   ⚠ 另外，旧代码那句「车端返回的 offset=512 与请求 0 不一致」的直接报错，
            //     其实就是这个缺口的副产品：上一块迟到的回复还在队列里，下一轮直接把它当答案。
            for (int attempt = 0; attempt < 6; attempt++)
            {
                if (attempt > 0)
                {
                    FlushRx();
                    retries++;
                    Thread.Sleep(10);
                }

                Write(TSlotRead, pl);
                var d = Wait(TSlotData, 1200);

                if (d == null) { lastWhy = "超时"; continue; }
                if (d.Length < 7) { lastWhy = "返回太短"; continue; }

                int gotOff0 = (int)U32(d, 1);
                int gotN0 = U16(d, 5);
                if (gotOff0 != off) { lastWhy = $"车端返回 offset={gotOff0}（请求 {off}）"; continue; }
                if (gotN0 == 0 || gotN0 > d.Length - 7) { lastWhy = "返回长度异常"; continue; }

                data = d;
                break;
            }

            if (data == null)
            {
                // 6 次全挂 ⇒ 再确认一下是不是车端**明确拒绝**（越界 / 槽空 / 不认识这条命令）
                var (r, d2) = WaitAck(TSlotRead, 200);
                if (r == 7)      // ERR_STATE：9 成是"车端压根没有 0x09"这条 case
                    throw new Exception("读槽被拒：状态不允许。★ 最可能的原因是车端固件旧、"
                        + "命令表里没有 0x09 SLOT_READ（车端把它当未知命令，回的就是这个笼统错误）。"
                        + " " + StaleFirmwareHint);
                if (r != 0) throw new Exception("读槽被拒：" + AckText(TSlotRead, r, d2));

                throw new Exception($"offset={off} 连续 6 次没读回来（最近一次：{lastWhy}）。"
                    + "串口在长传输里丢字节 ⇒ 换根线 / 换个 USB 口；还不行就把波特率降到 57600 再试。");
            }

            int n = U16(data, 5);       // 进到这里 data 一定已经通过 offset/长度 校验

            acc.AddRange(new ArraySegment<byte>(data, 7, n));
            off += n;

            if (expected < 0 && acc.Count >= 12)
            {
                uint cnt = U32(acc.ToArray(), 8);
                if (cnt > 0 && cnt <= 2040) expected = 36 + (int)cnt * 12;
            }
            double frac = expected > 0 ? Math.Min(1.0, (double)off / expected) : 0.0;
            progress?.Invoke(frac, $"{off}{(expected > 0 ? "/" + expected : "")} B"
                                   + (retries > 0 ? $"   重试 {retries}" : ""));

            if (expected > 0 && off >= expected) break;
        }
        progress?.Invoke(1.0, "完成");
        return acc.ToArray();
    }

    /// <summary>校验一张车端镜像（与车端 <c>ins_route_load_image()</c> 同一套判定）。</summary>
    public static string VerifyImage(byte[] img)
    {
        if (img == null || img.Length < 36) return "太短";
        if (U32(img, 0) != 0x4B525443) return "magic 不是 'CTRK'（不是车端镜像，别拿 asc_to_route.py 的 route.bin 来灌）";
        if (U16(img, 4) != 1) return "版本不是 1";
        if (U16(img, 6) != 12) return "记录长度不是 12";
        uint cnt = U32(img, 8), ds = U32(img, 12);
        if (cnt == 0) return "点数为 0";
        if (cnt > 2040) return $"点数 {cnt} 超过单槽 2040";
        if (ds == 0) return "点距为 0";
        if (Crc16(0xFFFF, img, 0, 34) != U16(img, 34)) return "表头 CRC 不过";
        int need = 36 + (int)cnt * 12;
        if (img.Length < need) return $"记录区不足（需要 {need} B，只有 {img.Length} B）";
        if (Crc16(0xFFFF, img, 36, (int)cnt * 12) != U16(img, 32)) return "记录区 CRC 不过";
        return null;
    }
}
