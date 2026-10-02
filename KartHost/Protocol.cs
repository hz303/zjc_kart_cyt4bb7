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
