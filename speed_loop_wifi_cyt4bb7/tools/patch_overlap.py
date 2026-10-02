"""① LayoutAudit 补"兄弟控件互相压"检查（原版只比"控件 vs 父容器"，漏掉了这种重叠）
② TravelCalPanel 缩文字、加高，消除重叠
"""
import os

ROOT = r"D:/jisuyueye9car/KartHost"


def load(p):
    raw = open(p, "rb").read()
    for enc in ("utf-8-sig", "utf-8"):
        try:
            t = raw.decode(enc)
            break
        except UnicodeDecodeError:
            continue
    else:
        raise SystemExit("解码失败 " + p)
    eol = "\r\n" if "\r\n" in t else "\n"
    return t.replace("\r\n", "\n"), enc, eol


def save(p, t, enc, eol):
    if eol == "\r\n":
        t = t.replace("\n", "\r\n")
    open(p, "wb").write(t.encode(enc))


def sub(t, old, new, tag):
    n = t.count(old)
    assert n == 1, (tag, "命中 %d 次" % n)
    print("  ✔", tag)
    return t.replace(old, new, 1)


# ---------------- ① LayoutAudit：加兄弟重叠检查 ----------------
p = os.path.join(ROOT, "LayoutAudit.cs")
t, enc, eol = load(p)

t = sub(t, "    private static void Visit(Control parent, List<Issue> list)\n    {\n        foreach (Control c in parent.Controls)",
        """    /// <summary>
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
                if (r.Width > 2 && r.Height > 2)
                {
                    list.Add(new Issue(a, "重叠",
                        $"{a.GetType().Name}(\\"{Trim(a.Text)}\\") 与 {b.GetType().Name}(\\"{Trim(b.Text)}\\") 重叠 {r.Width}x{r.Height}"));
                }
            }
        }
    }

    private static void Visit(Control parent, List<Issue> list)
    {
        CheckSiblingOverlap(parent, list);

        foreach (Control c in parent.Controls)""",
        "LayoutAudit 加重叠检查")
save(p, t, enc, eol)

# ---------------- ② TravelCalPanel：缩文字 + 加高 ----------------
p = os.path.join(ROOT, "Ui/TravelCalPanel.cs")
t, enc, eol = load(p)

t = sub(t, "            Height = Dpi.Px(214);", "            Height = Dpi.Px(230);", "面板加高")

t = sub(t, "                MaximumSize = new Size(Dpi.Px(206), 0),   // 够宽不换行，又不越出侧栏",
        "                MaximumSize = new Size(Dpi.Px(224), 0),   // 够宽不换行，又不越出侧栏", "body 最大宽度")

t = sub(t, "                Location = new Point(Dpi.Px(6), Dpi.Px(192)),\n                Text = \"当前角 --\",",
        "                Location = new Point(Dpi.Px(6), Dpi.Px(204)),\n                Text = \"当前角 --\",", "当前角下移")

t = sub(t, """            if (!Cal.Complete)
                return "先把转向推到左硬限位 →「记左限位」\\n再推到右硬限位 →「记右限位」\\n跨度 --\\n映射 --";

            if (!Cal.SpanOk)
                return $"左   {F(Cal.LeftSum, " mrad")}\\n右   {F(Cal.RightSum, " mrad")}\\n"
                     + "跨度太小(<200mrad) → 重记，要顶到硬限位\\n映射 --";""",
        """            if (!Cal.Complete)
                // 每行都短，保证不换行 —— 换行会把下面的"当前角"顶掉（踩过）
                return "左硬限位 →「记左限位」\\n右硬限位 →「记右限位」\\n跨度 --\\n映射 --";

            if (!Cal.SpanOk)
                return $"左   {F(Cal.LeftSum, " mrad")}\\n右   {F(Cal.RightSum, " mrad")}\\n"
                     + "跨度太小 → 重记（要顶到硬限位）\\n映射 --";""",
        "提示文字缩短")
save(p, t, enc, eol)
print("两处修完")
