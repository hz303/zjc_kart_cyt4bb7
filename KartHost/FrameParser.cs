namespace KartHost;

/// <summary>
/// 帧同步状态机。TCP 是字节流，一次接收可能给出「半个帧」或「两个半帧」，
/// 必须自己组帧。七个参考工程里没有一个正确处理这件事。
///
/// 两条纪律（这是会不会踩坑的分水岭）：
///   1) CRC 失败时只前移 1 字节，不是跳过整帧 —— 帧头可能是数据里的巧合字节。
///   2) 缓冲里始终保留不足一帧的尾巴 —— 帧头可能跨包。
/// </summary>
public sealed class FrameParser
{
    private readonly byte[] _buf;
    private int _len;

    /// <summary>统计</summary>
    public long FramesOk, FramesBadCrc, BytesIn;

    /// <summary>回调参数：(帧类型, 承载缓冲, payload 起始偏移)。★ 回调内必须立即消费，缓冲会被复用。</summary>
    public Action<byte, byte[], int> OnFrame;

    /// <summary>回调参数：(错误描述)</summary>
    public Action<string> OnBadFrame;

    public FrameParser(int capacity = 1 << 16)
    {
        _buf = new byte[capacity];
    }

    public void Feed(byte[] data, int count)
    {
        if (count <= 0) return;
        BytesIn += count;

        // 缓冲要满：丢掉最旧的一半，保住最近的数据
        if (_len + count > _buf.Length)
        {
            int keep = _buf.Length / 2;
            int drop = _len - keep;
            if (drop > 0)
            {
                Buffer.BlockCopy(_buf, drop, _buf, 0, keep);
                _len = keep;
            }
            // 还是放不下（单次投喂就超过缓冲）→ 只保留尾部
            if (count > _buf.Length)
            {
                Buffer.BlockCopy(data, count - _buf.Length, _buf, 0, _buf.Length);
                _len = _buf.Length;
                Parse();
                return;
            }
        }

        Buffer.BlockCopy(data, 0, _buf, _len, count);
        _len += count;
        Parse();
    }

    private void Parse()
    {
        int i = 0;
        while (i + Proto.UP_OVERHEAD <= _len)
        {
            // 1) 找帧头
            if (_buf[i] != Proto.MAGIC_UP_0 || _buf[i + 1] != Proto.MAGIC_UP_1) { i++; continue; }

            byte type = _buf[i + 2];
            int pl = Proto.GetU16(_buf, i + 3);

            // 明显的非法长度 → 认定这是巧合字节
            if (pl > Proto.MAX_PAYLOAD) { i++; continue; }

            int total = Proto.UP_OVERHEAD + pl;

            // 2) 半包：数据还没到齐，保留等待下次
            if (i + total > _len) break;

            // 3) 校验帧尾（JustFloat 兼容标记）
            int t = i + 7 + pl;
            if (_buf[t] != 0x00 || _buf[t + 1] != 0x00 || _buf[t + 2] != 0x80 || _buf[t + 3] != 0x7F)
            {
                i++;   // ★ 只前移 1 字节
                continue;
            }

            // 4) 校验 CRC（覆盖 TYPE ~ PAYLOAD）
            ushort crc = Proto.GetU16(_buf, i + 5 + pl);
            if (Proto.Crc16(_buf, i + 2, 3 + pl) != crc)
            {
                FramesBadCrc++;
                OnBadFrame?.Invoke($"CRC 校验失败 (type=0x{type:X2}, len={pl})");
                i++;   // ★ 只前移 1 字节
                continue;
            }

            // 5) 交付
            FramesOk++;
            try { OnFrame?.Invoke(type, _buf, i + 5); }
            catch (Exception ex) { OnBadFrame?.Invoke("解析回调异常: " + ex.Message); }

            i += total;
        }

        // 搬移剩余（不足一帧的尾巴必须留下）
        if (i > 0)
        {
            if (i < _len) Buffer.BlockCopy(_buf, i, _buf, 0, _len - i);
            _len -= i;
        }
    }

    public void Reset() { _len = 0; FramesOk = FramesBadCrc = BytesIn = 0; }

    /// <summary>当前缓冲里滞留的字节数（长时间不归零说明帧同步有问题）</summary>
    public int PendingBytes => _len;
}
