using System.Text;

namespace KartHost;

/// <summary>
/// 布局体检：走一遍控件树，**拿 PreferredSize（文字想占多大）比 Size（实际给了多大）**。
///
/// 为什么需要它：高 DPI 下"字被吃掉"这类问题编译不报错、断言"没崩"也全过，
/// 只在 125%/150% 缩放的机器上现形。做成机器可判的就再也躲不掉。
/// </summary>
public static class LayoutAudit
{
    public sealed record Issue(Control C, string Kind, string Detail);

    private static bool IsTextControl(Control c) => c is Button or Label or CheckBox or RadioButton;

    private static string Trim(string s) => s.Length <= 14 ? s : s[..14] + "…";

    public static List<Issue> Walk(Control root)
    {
        var list = new List<Issue>();
        Visit(root, list);
        return list;
    }

    public static string Report(Control root, string title, int deviceDpi)
    {
        var issues = Walk(root);
        var sb = new StringBuilder();
        sb.AppendLine($"== 布局体检：{title} ==");
        sb.AppendLine($"DPI = {deviceDpi}（{Ui.Dpi.Scale:0.###}x 缩放）   控件总数 = {Count(root)}");
        if (issues.Count == 0)
        {
            sb.AppendLine("  0 处问题（无剪字、无越界）");
        }
        else
        {
            foreach (var g in issues.GroupBy(i => i.Kind))
            {
                sb.AppendLine($"  [{g.Key}] {g.Count()} 处");
                foreach (var i in g.Take(12))
                    sb.AppendLine("     - " + i.Detail);
            }
        }
        return sb.ToString();
    }

    private static int Count(Control root)
    {
        int n = 1;
        foreach (Control c in root.Controls) n += Count(c);
        return n;
    }

    /// <summary>
    /// 兄弟控件互相压。
    /// ⚠ 这是原版的盲区：只比"控件 vs 父容器"的话，两个文字控件在父容器内互相叠着
    ///   是查不出来的（Bounds 都在父容器里），但用户看得见 —— 标定面板第一次就栽在这。
    /// 只查同一个父容器下、都是文字类控件的两两组合，避免误伤"标签压在面板上"这种正常布局。
    /// </summary>
    private static void CheckSiblingOverlap(Control parent, List<Issue> list)
    {
        var sibs = new List<Control>();
        foreach (Control k in parent.Controls)
        {
            if (IsTextControl(k) && k.Visible && !string.IsNullOrEmpty(k.Text)) sibs.Add(k);
        }

        for (int i = 0; i < sibs.Count; i++)
        {
            for (int j = i + 1; j < sibs.Count; j++)
            {
                var a = sibs[i];
                var b = sibs[j];
                var r = Rectangle.Intersect(a.Bounds, b.Bounds);
                // 两个方向都要压得住才算 —— 1~2px 的细缝是布局取整造成的，肉眼看不见，
                // 报出来只会变噪音（判据太灵 = 没人看，这条吃过亏）。
                if (r.Width > 6 && r.Height > 6)
                {
                    list.Add(new Issue(a, "重叠",
                        $"{a.GetType().Name}(\"{Trim(a.Text)}\") 与 {b.GetType().Name}(\"{Trim(b.Text)}\") 重叠 {r.Width}x{r.Height}"));
                }
            }
        }
    }

    private static void Visit(Control parent, List<Issue> list)
    {
        CheckSiblingOverlap(parent, list);

        foreach (Control c in parent.Controls)
        {
            if (IsTextControl(c) && !string.IsNullOrEmpty(c.Text))
            {
                var pref = c.PreferredSize;
                if (pref.Width > c.Width + 1 || pref.Height > c.Height + 1)
                {
                    list.Add(new Issue(c, "剪字",
                        $"{c.GetType().Name}(\"{Trim(c.Text)}\") 需要 {pref.Width}x{pref.Height}，实际 {c.Width}x{c.Height}"));
                }
            }

            bool scrollable = parent is ScrollableControl { AutoScroll: true };
            if (c.Right > parent.ClientSize.Width + 1)
            {
                list.Add(new Issue(c, "横向越界",
                    $"{c.GetType().Name}(\"{Trim(c.Text)}\") Right={c.Right} > 父可视宽 {parent.ClientSize.Width}"));
            }
            else if (!scrollable && c.Bottom > parent.ClientSize.Height + 1)
            {
                list.Add(new Issue(c, "纵向越界",
                    $"{c.GetType().Name}(\"{Trim(c.Text)}\") Bottom={c.Bottom} > 父可视高 {parent.ClientSize.Height}"));
            }

            Visit(c, list);
        }
    }
}
