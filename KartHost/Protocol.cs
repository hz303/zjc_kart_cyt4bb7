using System.Text;

namespace KartHost;

/// <summary>上行帧类型（车 → PC）。下行只有 Command 一种。</summary>
public enum FrameType : byte
{
    Telemetry = 0x01,
    ParamMeta = 0x02,
    ParamValue = 0x03,
    Ack = 0x04,
    Nak = 0x05,
    Event = 0x06,
    Command = 0x01,   // 仅下行

    // ---- ★ 路表 dump（车 → PC）：与车端 motor_link.c 的 MOTOR_TYPE_RT_* 一一对应 ----
    RouteHead = 0x10,   // 32 B 表头
    RouteData = 0x11,   // first(4) + n(2) + n×12 B 记录
    RouteEnd = 0x12,    // count(4) + lap_mm(4)
}

/// <summary>路表 dump 相关的常量（与车端 ins_route.c / motor_link.c 必须一致）</summary>
public static class RouteProto
{
    public const int RecSize = 12;          // int32 x_mm | int32 y_mm | int16 yaw_cdeg | int16 v_cmps
    public const int HeadBytes = 32;        // magic(4) ver(2) rec_len(2) count(4) ds_mm(4) lap_mm(4)
                                            // origin_x(4) origin_y(4) origin_yaw(2) flags(2)
    public const uint Magic = 0x4B525443;   // "CTRK"
    public const ushort Ver = 1;

    public struct Rec
    {
        public int Xmm, Ymm;                 // 相对录制起点的坐标（mm）
        public short YawCdeg;                // 相对航向（0.01°）
        public short VCmps;                  // 车速（cm/s）
        public float X => Xmm / 1000f;
        public float Y => Ymm / 1000f;
        public float YawDeg => YawCdeg / 100f;
        public float VMps => VCmps / 100f;
    }

    public struct Head
    {
        public uint Magic2, Count, DsMm, LapMm;
        public ushort Ver2, RecLen, OriginYawCdeg, Flags;
        public int OriginXmm, OriginYmm;
        public float OriginX => OriginXmm / 1000f;
        public float OriginY => OriginYmm / 1000f;
        public float OriginYawDeg => OriginYawCdeg / 100f;
        public float DsM => DsMm / 1000f;
        public float LapM => LapMm / 1000f;
        public bool Full => (Flags & 1) != 0;

        /// <summary>
        /// ★ 槽头 flags bit1 = 这是一份**巡航记录**（`Cruise` 实际走过的轨迹），
        /// 不是路表。此时第 4 个字段是**舵机 PWM duty** 而不是车速，
        /// `DsMm` 存的是**采样周期 ms**、`LapMm` 存的是**总时长 ms**。
        /// （定义见车端 `ins_route.h` 的 `INS_ROUTE_HDR_FL_LOG`。）
        /// </summary>
        public bool IsLog => (Flags & 2) != 0;

        /// <summary>
        /// ★ 槽头 flags bit2 = 这是一份**转向阶跃测试**记录（内环辨识，见车端 steer_step.h），
        /// 不是路表也不是巡航记录。12 B 在这份记录里是这样解释的：
        ///     int32 t_ms | int16 cmd_permill | int16 duty | int16 gz_cdps | int16 v_cmps
        /// 即第 1 个字段是**毫秒时刻**（不是 x_mm），第 2 个是**施加的归一舵量×1000**
        /// （不是 y_mm），第 3 个是**陀螺 z ×100**（不是航向），第 4 个还是 duty（在这里是车速 cm/s）。
        /// ⇒ 画图没有意义，导出时按这三列数字出来就好。
        /// </summary>
        public bool IsStep => (Flags & 4) != 0;
    }

    public static Rec ReadRec(byte[] b, int o) => new()
    {
        Xmm = (int)Proto.GetU32(b, o),
        Ymm = (int)Proto.GetU32(b, o + 4),
        YawCdeg = (short)Proto.GetU16(b, o + 8),
        VCmps = (short)Proto.GetU16(b, o + 10),
    };

    public static Head ReadHead(byte[] b, int o) => new()
    {
        Magic2 = Proto.GetU32(b, o),
        Ver2 = Proto.GetU16(b, o + 4),
        RecLen = Proto.GetU16(b, o + 6),
        Count = Proto.GetU32(b, o + 8),
        DsMm = Proto.GetU32(b, o + 12),
        LapMm = Proto.GetU32(b, o + 16),
        OriginXmm = (int)Proto.GetU32(b, o + 20),
        OriginYmm = (int)Proto.GetU32(b, o + 24),
        OriginYawCdeg = Proto.GetU16(b, o + 28),
        Flags = Proto.GetU16(b, o + 30),
    };
}

/// <summary>下行命令码</summary>
public enum Cmd : byte
{
    Ping = 0x01,
    GetParam = 0x02,
    SetParam = 0x03,
    SetEnable = 0x04,
    SaveParams = 0x05,
    LoadParams = 0x06,
    GetTable = 0x07,
    RouteDump = 0x08,   // ★ 读路表：车端把整张表分帧 dump 上来（见 FrameType.RouteHead/Data/End）
    RecordStart = 0x10,
    RecordStop = 0x11,
    MotionStart = 0x12,
    MotionStop = 0x13,
}

/// <summary>使能位定义（SET_ENABLE 的 VALUE 按位解释）</summary>
[Flags]
public enum EnableBits : ushort
{
    None = 0,
    Steer = 1 << 0,
    Motor = 1 << 1,
    Master = 1 << 2,
    Telemetry = 1 << 3,
}

/// <summary>参数元数据（来自车端 PARAM_META 帧，或本地默认表）</summary>
public sealed class ParamMeta
{
    public byte Id;
    public string Name = "";     // 英文名（遥测通道绑定用）
    public string Label = "";    // 中文名（UI 显示）
    public string Group = "";    // 分组（UI 折叠用）
    public float Lo, Hi, Def, Step;
    public string Unit = "";
    public float Value;          // 当前值
    public bool Dirty;           // 已改未存 Flash

    public double Span => Math.Abs(Hi - Lo) < 1e-9 ? 1.0 : (Hi - Lo);

    /// <summary>滑条整数 0..1000 → 物理值</summary>
    public float SliderToValue(int s) => Lo + (s / 1000f) * (Hi - Lo);

    /// <summary>物理值 → 滑条整数 0..1000</summary>
    public int ValueToSlider(float v)
    {
        if (Span < 1e-9) return 0;
        int s = (int)Math.Round((v - Lo) / Span * 1000.0);
        return Math.Clamp(s, 0, 1000);
    }

    /// <summary>按步长吸附 + 量程钳位</summary>
    public float Snap(float v)
    {
        if (Step > 1e-9f) v = (float)(Math.Round(v / Step) * Step);
        return Math.Clamp(v, Math.Min(Lo, Hi), Math.Max(Lo, Hi));
    }
}

public static class Proto
{
    // ---- 帧常量 ----
    public const byte MAGIC_UP_0 = 0xA5;
    public const byte MAGIC_UP_1 = 0x5A;
    public const byte MAGIC_DN_0 = 0x5A;
    public const byte MAGIC_DN_1 = 0xA5;

    /// <summary>上行帧固定开销：MAGIC(2)+TYPE(1)+LEN(2)+CRC(2)+TAIL(4) = 11</summary>
    public const int UP_OVERHEAD = 11;
    /// <summary>下行帧固定开销：MAGIC(2)+TYPE(1)+LEN(2)+CRC(2) = 7</summary>
    public const int DN_OVERHEAD = 7;
    /// <summary>下行命令载荷固定 8 字节</summary>
    public const int DN_PAYLOAD = 8;
    /// <summary>遥测载荷固定开销：SEQ(2)+TS(4)+MASK(2)+COUNT(2) = 10</summary>
    public const int TELEM_OVERHEAD = 10;

    public static readonly byte[] TAIL = { 0x00, 0x00, 0x80, 0x7F };

    public const int MAX_CHANNELS = 64;
    /// <summary>LEN 的合理上限，用于快速否决误判的帧头</summary>
    public const int MAX_PAYLOAD = 4096;

    // ---- CRC16-MODBUS（多项式 0xA001，与逐飞虚拟示波器一致）----
    public static ushort Crc16(byte[] buf, int off, int len)
    {
        ushort crc = 0xFFFF;
        for (int i = 0; i < len; i++)
        {
            crc ^= buf[off + i];
            for (int j = 0; j < 8; j++)
                crc = (crc & 1) != 0 ? (ushort)((crc >> 1) ^ 0xA001) : (ushort)(crc >> 1);
        }
        return crc;
    }

    // ---- 小端读写 ----
    public static ushort GetU16(byte[] b, int o) => (ushort)(b[o] | (b[o + 1] << 8));
    public static uint GetU32(byte[] b, int o) =>
        (uint)(b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24));
    /// <summary>Windows x64 本身是小端，BitConverter 可直接用</summary>
    public static float GetF32(byte[] b, int o) => BitConverter.ToSingle(b, o);

    public static void PutU16(byte[] b, int o, ushort v) { b[o] = (byte)v; b[o + 1] = (byte)(v >> 8); }
    public static void PutU32(byte[] b, int o, uint v)
    {
        b[o] = (byte)v; b[o + 1] = (byte)(v >> 8);
        b[o + 2] = (byte)(v >> 16); b[o + 3] = (byte)(v >> 24);
    }
    public static void PutF32(byte[] b, int o, float v) => Buffer.BlockCopy(BitConverter.GetBytes(v), 0, b, o, 4);

    // ---- 定长字符串写入载荷（不足补 0）----
    public static void PutStr(byte[] b, int o, int cap, string s)
    {
        var raw = Encoding.UTF8.GetBytes(s ?? "");
        int n = Math.Min(raw.Length, cap - 1);
        Array.Clear(b, o, cap);
        Array.Copy(raw, 0, b, o, n);
    }

    public static string GetStr(byte[] b, int o, int cap)
    {
        int n = 0;
        while (n < cap && b[o + n] != 0) n++;
        return Encoding.UTF8.GetString(b, o, n);
    }

    // ---- 下行命令帧构造（总长 15）----
    public static byte[] BuildCommand(Cmd cmd, byte paramId, float value, ushort token)
    {
        var f = new byte[DN_OVERHEAD + DN_PAYLOAD];
        f[0] = MAGIC_DN_0; f[1] = MAGIC_DN_1;
        f[2] = (byte)FrameType.Command;
        PutU16(f, 3, DN_PAYLOAD);
        f[5] = (byte)cmd;
        f[6] = paramId;
        PutF32(f, 7, value);
        PutU16(f, 11, token);
        PutU16(f, 13, Crc16(f, 2, 3 + DN_PAYLOAD));   // 覆盖 TYPE~PAYLOAD
        return f;
    }

    /// <summary>
    /// 构造"设置使能位图"帧。
    /// ★ 约定：VALUE 这 4 个字节在 Enable 命令里按**数值**解释（6 → 6.0f → 接收方取整得 6）。
    ///   不要把位图当 IEEE754 位模式塞进去 —— 那样接收端 read_u32 再截 16 位会得到
    ///   0x0000 / 0xC000 之类的垃圾值，症状是"使能怎么都传不下去"且不报错（纯静默失效）。
    /// </summary>
    public static byte[] BuildSetEnable(EnableBits bits, ushort token)
        => BuildCommand(Cmd.SetEnable, 0, (ushort)bits, token);

    public static byte[] BuildSetParam(byte id, float v, ushort token)
        => BuildCommand(Cmd.SetParam, id, v, token);

    public static byte[] BuildSimple(Cmd cmd, ushort token)
        => BuildCommand(cmd, 0, 0f, token);
}
