using System.Globalization;
using System.Text;

namespace KartHost;

/// <summary>
/// ★ CSV 数据记录 / 导出模块（2026-10-13 新增）。
///
/// 为什么要有它：**实车落地标定时车接不上串口**（车要跑开），所以 VOFA+ 那条路走不通。
/// 唯一能用的通道就是已经跑通的那条 WiFi 遥测 —— 于是把"上位机收到的通道值按时间戳落盘"，
/// 再离线拿 Excel / Python 分析，做开环阶跃的辨识与结果留档。
///
/// 设计取舍：
///   · **只写盘、不参与任何控制**。断开/异常都不影响链路，最多是这一条曲线缺几个点。
///   · 表头带通道名 + 单位（`v_l[rps]`），并且**写 UTF-8 BOM** —— 否则 Excel 打开中文表头是乱码。
///   · 采样周期是**上位机侧的抽稀**（车端 20 ms 一帧、这里可以记 20/50/100 ms），
///     这样"记多密"和"车发多快"解耦：要看细节就把车端 tel_period 调小，
///     要记很长时间就把本机周期调大。
///   · `note` 列专门留给"阶跃标记"：按一次按钮就在那一行写一句，
///     离线时按 note 非空的行去对齐时间轴，比靠肉眼看拐点可靠得多。
/// </summary>
public sealed class CsvRecorder
{
    private readonly ChannelStore _store;

    private StreamWriter _w;
    private int[] _channels = Array.Empty<int>();
    private int _periodMs = 20;
    private double _t0 = double.NaN;      // 记录起点（ms，来自传入的同一时钟）
    private double _next = double.NaN;    // 下一次落盘的时刻
    private long _rows;
    private string _pendingNote = "";

    public CsvRecorder(ChannelStore store) { _store = store; }

    public bool IsRecording => _w != null;
    public string Path { get; private set; } = "";
    public long Rows => _rows;
    public int PeriodMs => _periodMs;

    /// <summary>开始录制。<paramref name="nowMs"/> 用调用方的同一时钟（一般就是主界面的秒表）。</summary>
    public bool Start(string path, int periodMs, IReadOnlyList<int> channels, double nowMs, out string err)
    {
        err = null;

        if (IsRecording) { err = "已经在录制中"; return false; }
        if (channels == null || channels.Count == 0) { err = "没有可记录的通道"; return false; }
        if (periodMs < 1) periodMs = 1;

        try
        {
            string dir = System.IO.Path.GetDirectoryName(path);
            if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);

            // UTF-8 **带 BOM**：Excel 直接双击打开时中文表头才不会乱码
            _w = new StreamWriter(path, false, new UTF8Encoding(true));

            _channels = channels.ToArray();
            _periodMs = periodMs;
            _t0 = nowMs;
            _next = nowMs;
            _rows = 0;
            Path = path;

            WriteHeader();
            return true;
        }
        catch (Exception ex)
        {
            _w?.Dispose();
            _w = null;
            err = ex.Message;
            return false;
        }
    }

    private void WriteHeader()
    {
        var sb = new StringBuilder();
        sb.Append("t_ms,note");
        foreach (int ch in _channels)
        {
            string unit = Defaults.ChannelUnit(ch);
            sb.Append(',');
            sb.Append(Defaults.ChannelName(ch));
            if (!string.IsNullOrEmpty(unit)) sb.Append('[').Append(unit).Append(']');
        }
        _w.WriteLine(sb.ToString());
    }

    /// <summary>主界面定时器每圈调一次；内部按 periodMs 抽稀。掉帧太多会重新对齐，不会追债式补点。</summary>
    public void Tick(double nowMs)
    {
        if (_w == null) return;

        if (double.IsNaN(_t0)) { _t0 = nowMs; _next = nowMs; }

        if (nowMs < _next) return;

        _next += _periodMs;
        if (_next < nowMs) _next = nowMs + _periodMs;   // 落后太多 -> 重新对齐（不补假点）

        WriteRow(nowMs);
    }

    /// <summary>
    /// 打一条标记：**立刻**落一行（不等采样周期），note 列写上这句话。
    /// 阶跃按钮点下去时调它 —— 这样"指令发出的那一刻"在 CSV 里是一个明确的锚点。
    /// </summary>
    public void Note(double nowMs, string note)
    {
        if (_w == null) return;
        if (double.IsNaN(_t0)) { _t0 = nowMs; _next = nowMs; }

        _pendingNote = note ?? "";
        WriteRow(nowMs);
        _pendingNote = "";
    }

    private void WriteRow(double nowMs)
    {
        var sb = new StringBuilder(64 + _channels.Length * 12);

        sb.Append(((long)Math.Round(nowMs - _t0)).ToString(CultureInfo.InvariantCulture));
        sb.Append(',');
        sb.Append(Escape(_pendingNote));

        foreach (int ch in _channels)
        {
            float v = _store.Latest(ch);
            sb.Append(',');
            if (float.IsNaN(v))
                sb.Append("");                                    // 没数据就留空，别写 NaN（Excel 会当文本）
            else
                sb.Append(v.ToString("0.######", CultureInfo.InvariantCulture));
        }

        _w.WriteLine(sb.ToString());
        _rows++;
    }

    private static string Escape(string s)
    {
        if (string.IsNullOrEmpty(s)) return "";
        if (s.IndexOfAny(new[] { ',', '"', '\n', '\r' }) < 0) return s;
        return "\"" + s.Replace("\"", "\"\"") + "\"";
    }

    /// <summary>停止并收尾，返回写了多少行（不含表头）。</summary>
    public long Stop()
    {
        if (_w == null) return _rows;

        long n = _rows;
        try { _w.Flush(); _w.Dispose(); } catch { /* 收尾失败不再抛：文件已经写到这一步了 */ }

        _w = null;
        _rows = 0;
        _channels = Array.Empty<int>();
        _t0 = double.NaN;
        _next = double.NaN;
        return n;
    }
}
