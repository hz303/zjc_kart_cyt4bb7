using System.Globalization;
using System.Text;

namespace KartHost;

/// <summary>
/// ★ 波形快照导出（2026-10-13 新增）。
///
/// 与 <see cref="CsvRecorder"/> 的分工 —— **两个都是为了"事后能读数字"，但时机不同**：
///   · `CsvRecorder` = **从现在起往后记**。按下"开始录制"之后的每一拍落一行，
///     用于已经知道要做什么实验（实车阶跃标定）的场合；
///   · `CsvExport`   = **把缓冲里已经收到的那一段倒出来**。用于"事后才想起来要看数字"：
///     屏幕上那条波形长什么样，导出来的 CSV 就是什么样，一个点都不少。
///
/// 为什么必须有它：**看截图读波形是不可靠的** —— y 轴自动量程、像素分辨率、
/// 抗锯齿混色，随便哪一条都能让"读到"的数字差出几个百分点。通道值在
/// <see cref="ChannelStore"/> 里本来就有（每通道 6000 点环形缓冲），
/// 把它倒成 CSV 是既有的数据，不做任何猜测。
///
/// 两条硬约束：
///   1) **表头与 CsvRecorder 逐字节相同**（`t_ms,note,&lt;名&gt;[&lt;单位&gt;],...`）——
///      下游的解析工具（`tools/calib_step_ident.py`）一套代码吃两种文件。快照没有锚点，`note` 列恒为空。
///   2) **只读缓冲、不动链路**。导出期间即使有数据在进来也照样导（快照取的是同一时刻的连续区间）。
/// </summary>
public static class CsvExport
{
    /// <summary>
    /// 把 <paramref name="channels"/> 在缓冲里的**全部有效点**倒成一个 CSV。
    ///
    /// 时间轴 = `store` 里那条 PC 本地时钟（与波形、与 CsvRecorder 是**同一条**），
    /// 输出时减掉第一点、单位换算成毫秒。
    /// <paramref name="info"/> 返回一句人话（导了几个通道 / 多少点 / 时间跨度），可直接打进日志。
    /// </summary>
    /// <returns>写成的数据行数（不含表头）。缓冲里没数据时返回 0。</returns>
    public static int Dump(string path, ChannelStore store, IEnumerable<int> channels, out string info)
    {
        if (store == null) throw new ArgumentNullException(nameof(store));

        var list = new List<int>();
        foreach (int c in channels)
        {
            if (c >= 0 && c < store.ChannelCount && !list.Contains(c)) list.Add(c);
        }
        list.Sort();

        if (list.Count == 0) throw new InvalidOperationException("没有可导出的通道");
        if (store.Count == 0) { info = "缓冲里还没有数据"; return 0; }

        int n = store.Count;
        var cols = new float[list.Count][];
        var t = new double[n];

        for (int k = 0; k < list.Count; k++)
        {
            cols[k] = new float[n];
            store.Snapshot(list[k], cols[k], t, n);      // 每列都覆盖 t，但写入的是同一根时间轴
        }

        double t0 = t[0];
        double tEnd = t[n - 1];

        string dir = System.IO.Path.GetDirectoryName(path);
        if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);

        // UTF-8 **带 BOM**：Excel 双击打开时中文表头不乱码（与 CsvRecorder 同一取舍）
        using (var w = new StreamWriter(path, false, new UTF8Encoding(true)))
        {
            var sb = new StringBuilder(64 + list.Count * 12);
            sb.Append("t_ms,note");
            foreach (int ch in list)
            {
                string unit = Defaults.ChannelUnit(ch);
                sb.Append(',');
                sb.Append(Defaults.ChannelName(ch));
                if (!string.IsNullOrEmpty(unit)) sb.Append('[').Append(unit).Append(']');
            }
            w.WriteLine(sb.ToString());

            sb.Clear();
            for (int i = 0; i < n; i++)
            {
                sb.Append(((long)Math.Round((t[i] - t0) * 1000.0)).ToString(CultureInfo.InvariantCulture));
                sb.Append(",,");                          // note 列留空（快照没有锚点）
                for (int k = 0; k < list.Count; k++)
                {
                    float v = cols[k][i];
                    if (!float.IsNaN(v) && !float.IsInfinity(v))
                        sb.Append(v.ToString("0.######", CultureInfo.InvariantCulture));
                    sb.Append(',');                        // 空值就留空，别写 NaN（Excel 会当文本）
                }
                sb.Length--;                               // 去掉行尾多出来的那个逗号
                w.WriteLine(sb.ToString());
                sb.Clear();
            }
        }

        info = string.Format(CultureInfo.InvariantCulture,
            "{0} 个通道 × {1} 点，跨度 {2:0.00} s（{3:0.#} ms/点）",
            list.Count, n, (tEnd - t0), n > 1 ? (tEnd - t0) * 1000.0 / (n - 1) : 0.0);
        return n;
    }

    /// <summary>
    /// 默认保存路径：与本工程的实时记录同一个 `logs/` 目录（`publish/logs/`），
    /// 文件名带时间戳，随手导不会互相覆盖。
    /// </summary>
    public static string DefaultPath()
    {
        string dir = System.IO.Path.Combine(AppContext.BaseDirectory, "logs");
        return System.IO.Path.Combine(dir, "wave_" + DateTime.Now.ToString("yyyyMMdd_HHmmss") + ".csv");
    }
}
