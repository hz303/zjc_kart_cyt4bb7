namespace KartHost;

/// <summary>
/// 通道数据仓库：每通道一个定长环形数组。
/// 接收线程只往里写，UI 定时器只从中取快照 —— 两侧解耦，UI 速度不影响接收。
/// </summary>
public sealed class ChannelStore
{
    public const int Capacity = 6000;

    private float[,] _vals;          // [通道, 环形索引]
    private double[] _time;          // 各点时间戳（秒，PC 本地时间轴）
    private int _n;                  // 已分配的通道数
    private int _w;                  // 下一个写入位置
    private int _fill;               // 有效点数
    private readonly object _lock = new();

    /// <summary>本帧哪些通道有效（bit 位图）；无效通道 UI 不更新曲线，避免画假竖线</summary>
    public ushort ValidMask { get; private set; } = 0xFFFF;

    public int ChannelCount { get { lock (_lock) return _n; } }
    public int Count { get { lock (_lock) return _fill; } }

    public void Reset()
    {
        lock (_lock) { _w = 0; _fill = 0; }
    }

    /// <summary>按需扩展通道数（车端 COUNT 可能变化）</summary>
    public void Ensure(int n)
    {
        if (n <= 0) return;
        if (n > Proto.MAX_CHANNELS) n = Proto.MAX_CHANNELS;
        lock (_lock)
        {
            if (_n >= n) return;
            var nv = new float[n, Capacity];
            if (_vals != null)
                for (int c = 0; c < _n; c++)
                    for (int k = 0; k < Capacity; k++) nv[c, k] = _vals[c, k];
            _vals = nv;
            _time ??= new double[Capacity];
            _n = n;
        }
    }

    public void Append(float[] ch, int count, double ts, ushort validMask)
    {
        lock (_lock)
        {
            if (_vals == null) return;
            if (count > _n) count = _n;
            for (int c = 0; c < count; c++) _vals[c, _w] = ch[c];
            _time[_w] = ts;
            ValidMask = validMask;
            _w = (_w + 1) % Capacity;
            if (_fill < Capacity) _fill++;
        }
    }

    /// <summary>取某通道最近 n 个点，按时间递增顺序写入 dv/dt，返回实际点数。</summary>
    public int Snapshot(int ch, float[] dv, double[] dt, int maxPoints)
    {
        lock (_lock)
        {
            if (_vals == null || ch < 0 || ch >= _n) return 0;
            int n = Math.Min(_fill, Math.Min(maxPoints, dv.Length));
            int start = (_w - n + Capacity) % Capacity;
            for (int k = 0; k < n; k++)
            {
                int idx = (start + k) % Capacity;
                dv[k] = _vals[ch, idx];
                dt[k] = _time[idx];
            }
            return n;
        }
    }

    /// <summary>取某通道最新值（无数据返回 NaN）</summary>
    public float Latest(int ch)
    {
        lock (_lock)
        {
            if (_vals == null || ch < 0 || ch >= _n || _fill == 0) return float.NaN;
            int idx = (_w - 1 + Capacity) % Capacity;
            return _vals[ch, idx];
        }
    }

    /// <summary>最新时间戳（秒）</summary>
    public double LatestTime()
    {
        lock (_lock)
        {
            if (_time == null || _fill == 0) return 0;
            return _time[(_w - 1 + Capacity) % Capacity];
        }
    }
}
