using System;
using System.Collections;
using System.Collections.Generic;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Networking;

/// <summary>
/// Connects to the XrayVision Pi station (HTTP + WebSocket) without ROS.
///
/// Endpoints consumed:
///   GET  http://{host}:{port}/healthz   - liveness check
///   GET  http://{host}:{port}/pose      - static camera pose (one-shot)
///   WS   ws://{host}:{port}/stream      - JSON detections, ~10 Hz
///
/// This script only handles the connection. Detection -> GameObject rendering
/// is a separate concern (next step).
/// </summary>
public class PiClient : MonoBehaviour
{
    [Tooltip("Pi address. Try IP first (e.g. 10.0.0.101), falls back to hostnames.")]
    public string host = "10.0.0.101";

    [Tooltip("Comma-separated fallback hosts tried in order if the primary fails.")]
    public string fallbackHosts = "pi.local,pi";

    [Tooltip("Pi server port (matches station.yaml server.port).")]
    public int port = 8765;

    [Tooltip("Reconnect after this many seconds when the WebSocket drops.")]
    public float reconnectDelaySeconds = 2f;

    [Tooltip("Log every detection frame received. Spammy — leave off in production.")]
    public bool verboseLogging = false;

    public bool IsConnected { get; private set; }
    public string ResolvedHost { get; private set; }
    public string LastPayload { get; private set; }
    public event Action<string> OnDetectionPayload;   // raw JSON from /stream
    public event Action<PoseData> OnPoseReceived;     // /pose snapshot

    [Serializable]
    public struct PoseData
    {
        public float[] position_m;
        public float[] rpy_deg;
        public float[] mount_rpy_deg;
    }

    ClientWebSocket socket;
    CancellationTokenSource cts;
    Task wsLoopTask;

    void Start()
    {
        cts = new CancellationTokenSource();
        StartCoroutine(BootstrapWhenHostReachable());
    }

    void OnDestroy()
    {
        cts?.Cancel();
        try { socket?.Abort(); } catch { /* ignore */ }
        socket?.Dispose();
    }

    IEnumerator BootstrapWhenHostReachable()
    {
        // Try the primary host, then each fallback. First /healthz that returns
        // 200 wins. We then fetch /pose once and start the WebSocket loop.
        var candidates = new List<string> { host };
        if (!string.IsNullOrWhiteSpace(fallbackHosts))
        {
            foreach (var h in fallbackHosts.Split(','))
            {
                var trimmed = h.Trim();
                if (!string.IsNullOrEmpty(trimmed)) candidates.Add(trimmed);
            }
        }

        while (!cts.IsCancellationRequested)
        {
            foreach (var candidate in candidates)
            {
                yield return TryHealthz(candidate, ok =>
                {
                    if (ok) ResolvedHost = candidate;
                });
                if (!string.IsNullOrEmpty(ResolvedHost)) break;
            }

            if (!string.IsNullOrEmpty(ResolvedHost)) break;

            Debug.LogWarning($"[PiClient] no candidate host responded: {string.Join(", ", candidates)} — retrying in {reconnectDelaySeconds}s");
            yield return new WaitForSeconds(reconnectDelaySeconds);
        }

        if (cts.IsCancellationRequested) yield break;

        Debug.Log($"[PiClient] using host {ResolvedHost}:{port}");
        yield return FetchPose();
        wsLoopTask = Task.Run(() => StreamLoopAsync(cts.Token));
    }

    IEnumerator TryHealthz(string candidate, Action<bool> done)
    {
        var url = $"http://{candidate}:{port}/healthz";
        using var req = UnityWebRequest.Get(url);
        req.timeout = 2;
        yield return req.SendWebRequest();
        var ok = req.result == UnityWebRequest.Result.Success && req.responseCode == 200;
        if (verboseLogging) Debug.Log($"[PiClient] healthz {url} -> {req.responseCode}");
        done(ok);
    }

    IEnumerator FetchPose()
    {
        var url = $"http://{ResolvedHost}:{port}/pose";
        using var req = UnityWebRequest.Get(url);
        req.timeout = 2;
        yield return req.SendWebRequest();
        if (req.result != UnityWebRequest.Result.Success)
        {
            Debug.LogWarning($"[PiClient] /pose fetch failed: {req.error}");
            yield break;
        }
        try
        {
            var pose = JsonUtility.FromJson<PoseData>(req.downloadHandler.text);
            Debug.Log($"[PiClient] pose: pos=[{string.Join(",", pose.position_m ?? new float[0])}] rpy=[{string.Join(",", pose.rpy_deg ?? new float[0])}]");
            OnPoseReceived?.Invoke(pose);
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[PiClient] /pose parse failed: {e.Message}");
        }
    }

    async Task StreamLoopAsync(CancellationToken token)
    {
        var uri = new Uri($"ws://{ResolvedHost}:{port}/stream");
        var buffer = new byte[1 << 16];

        while (!token.IsCancellationRequested)
        {
            socket = new ClientWebSocket();
            try
            {
                await socket.ConnectAsync(uri, token);
                IsConnected = true;
                Debug.Log($"[PiClient] websocket connected: {uri}");

                while (socket.State == WebSocketState.Open && !token.IsCancellationRequested)
                {
                    var sb = new StringBuilder();
                    WebSocketReceiveResult result;
                    do
                    {
                        result = await socket.ReceiveAsync(new ArraySegment<byte>(buffer), token);
                        if (result.MessageType == WebSocketMessageType.Close)
                        {
                            await socket.CloseAsync(WebSocketCloseStatus.NormalClosure, "bye", CancellationToken.None);
                            break;
                        }
                        sb.Append(Encoding.UTF8.GetString(buffer, 0, result.Count));
                    } while (!result.EndOfMessage);

                    if (sb.Length == 0) continue;
                    var payload = sb.ToString();
                    LastPayload = payload;
                    if (verboseLogging) Debug.Log($"[PiClient] {payload}");
                    OnDetectionPayload?.Invoke(payload);
                }
            }
            catch (OperationCanceledException) { /* shutdown */ }
            catch (Exception e)
            {
                Debug.LogWarning($"[PiClient] websocket error: {e.Message}");
            }
            finally
            {
                IsConnected = false;
                try { socket.Dispose(); } catch { /* ignore */ }
            }

            if (token.IsCancellationRequested) break;
            await Task.Delay(TimeSpan.FromSeconds(reconnectDelaySeconds), token);
        }
    }
}
