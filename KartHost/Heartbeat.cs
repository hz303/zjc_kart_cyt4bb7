namespace KartHost;

/// <summary>
/// 心跳：车端 motor_link 用「距上次下行帧 > lost_ms」判上位机是否还在，
/// 判失联就把电机输出清零。而上位机只在用户操作时发命令 ——
/// **不补心跳的话，现象是"拖到 40% 电机转一下，松手 0.6 秒自己停"，而且完全静默。
/// 这个缺陷不会抛异常、也不会让任何"值对不对"的断言变红，所以单独抽成一个可测的小单元。
/// </summary>
public static class Heartbeat
{
    /// <summary>
    /// 心跳周期 ms。必须**明显小于**车端 lost_ms（参数表默认 600ms，最小可设 100ms）。
    /// 100ms 时对默认值有 6 倍余量；实测每帧只有 15 字节，车端 ACK 也不写日志，代价可忽略。
    /// </summary>
    public const int IntervalMs = 100;

    /// <summary>发一次心跳。链路没开时返回 false（不抛异常）。</summary>
    public static bool Beat(Link link, Func<ushort> nextToken)
    {
        if (link == null || !link.IsOpen) return false;
        return link.Send(Proto.BuildCommand(Cmd.Ping, 0, 0f, nextToken()));
    }
}
