using System.Net;
using System.Net.Sockets;

namespace KartHost;

/// <summary>
/// 网络层：只做"收字节 → 交给解析器"，不解析、不碰 UI。
/// 上行走 TCP（可靠）或 UDP；下行统一走同一 socket。
/// </summary>
public sealed class Link : IDisposable
{
    public enum Mode { TcpClient, Udp }

    private TcpClient _tcp;
    private UdpClient _udp;
    private IPEndPoint _udpRemote;
    private Thread _rx;
    private volatile bool _run;
    private readonly FrameParser _parser = new();
    private readonly object _sendLock = new();

    public bool IsOpen { get; private set; }
    public Mode CurrentMode { get; private set; }
    public string RemoteDesc { get; private set; } = "";

    public long BytesIn => _parser.BytesIn;
    public long FramesOk => _parser.FramesOk;
    public long FramesBadCrc => _parser.FramesBadCrc;
    public int PendingBytes => _parser.PendingBytes;

    /// <summary>(帧类型, 缓冲, payload 起始偏移) —— 回调内必须立即消费</summary>
    public event Action<byte, byte[], int> FrameReceived;
    public event Action<string> Log;
    public event Action<bool> StateChanged;

    // ------------------------------------------------------------------
    public void ConnectTcp(string ip, int port)
    {
        Close();
        var c = new TcpClient();
        c.NoDelay = true;                       // 遥测是小包，必须关 Nagle
        c.Connect(ip, port);
        _tcp = c;
        CurrentMode = Mode.TcpClient;
        RemoteDesc = $"TCP {ip}:{port}";
        StartRx();
    }

    /// <summary>
    /// UDP 模式：**绑定本地端口收包**，对端地址从收到的第一个包里学（车端主动上报的拓扑）。
    /// ★ 之前这里写成 `new UdpClient()`（随机本地端口）—— 车端发往 PC 的 8086，
    ///   随机端口根本收不到，表现是"车端明明在发，上位机一片空白"。
    /// </summary>
    public void ConnectUdp(int localPort)
    {
        Close();
        // 刻意**不用** ReuseAddress：端口被别人占了（比如逐飞助手也在监听 8086）就让它明确报错，
        // 而不是两个程序各自随机收到一半的包 —— 那种故障最难查。
        var u = new UdpClient(new IPEndPoint(IPAddress.Any, localPort));
        _udp = u;
        _udpRemote = null;                      // 还没收到包 → 还不知道车在哪
        CurrentMode = Mode.Udp;
        RemoteDesc = $"UDP 本地 :{localPort}";
        StartRx();
    }

    private void StartRx()
    {
        _run = true;
        IsOpen = true;
        _parser.OnFrame = (t, b, o) => FrameReceived?.Invoke(t, b, o);
        _parser.OnBadFrame = m => Log?.Invoke(m);

        _rx = new Thread(RxLoop) { IsBackground = true, Name = "KartHost.Rx" };
        _rx.Start();

        StateChanged?.Invoke(true);
        Log?.Invoke("已连接 " + RemoteDesc);
    }

    private void RxLoop()
    {
        var buf = new byte[1 << 16];
        try
        {
            while (_run)
            {
                int n;
                if (CurrentMode == Mode.TcpClient)
                {
                    var s = _tcp.Client;
                    if (!s.Connected) break;
                    n = s.Receive(buf, 0, buf.Length, SocketFlags.None);
                    if (n <= 0) break;          // 对端关闭
                }
                else
                {
                    IPEndPoint any = new(IPAddress.Any, 0);
                    buf = _udp.Receive(ref any);
                    n = buf.Length;
                    // ★ 学到车端地址：之后下行命令就发给它，用户不用手填车端 IP
                    if (_udpRemote == null || !_udpRemote.Equals(any))
                    {
                        _udpRemote = any;
                        Log?.Invoke($"UDP 对端已锁定：{any.Address}:{any.Port}");
                        StateChanged?.Invoke(true);
                    }
                }
                _parser.Feed(buf, n);
            }
        }
        catch (SocketException ex)
        {
            if (_run) Log?.Invoke("接收中断: " + ex.SocketErrorCode);
        }
        catch (ObjectDisposedException) { /* 正常关闭 */ }
        catch (Exception ex) { Log?.Invoke("接收异常: " + ex.Message); }

        if (_run)
        {
            _run = false;
            IsOpen = false;
            Log?.Invoke("连接已断开");
            StateChanged?.Invoke(false);
        }
    }

    public bool Send(byte[] frame)
    {
        if (!IsOpen || frame == null) return false;
        if (CurrentMode == Mode.Udp && _udpRemote == null)
        {
            Log?.Invoke("UDP 还没收到过车端的包，暂时不知道往哪发（下发命令被丢弃）");
            return false;
        }
        try
        {
            lock (_sendLock)
            {
                if (CurrentMode == Mode.TcpClient) _tcp.Client.Send(frame, 0, frame.Length, SocketFlags.None);
                else _udp.Send(frame, frame.Length, _udpRemote);
            }
            return true;
        }
        catch (Exception ex)
        {
            Log?.Invoke("发送失败: " + ex.Message);
            return false;
        }
    }

    public void Close()
    {
        _run = false;
        try { _tcp?.Close(); } catch { }
        try { _udp?.Close(); } catch { }
        _tcp = null; _udp = null; _udpRemote = null;
        _parser.Reset();
        if (IsOpen)
        {
            IsOpen = false;
            StateChanged?.Invoke(false);
        }
    }

    public void Dispose() => Close();
}
