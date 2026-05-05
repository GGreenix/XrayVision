using System;
using UnityEngine;

/// <summary>
/// Spawns colored markers in the scene to diagnose coordinate alignment:
///   - Red   sphere  at world origin (0,0,0)
///   - Green sphere  1m along +X
///   - Blue  sphere  1m along +Y
///   - Yellow sphere 1m along +Z
///   - White sphere  at current camera_pos (raw, no conversion, no scale)
///
/// Place this on any GameObject, wire up PiClient, and press Play.
/// Look where the white sphere lands relative to the mesh to figure out
/// which axis needs flipping/scaling.
/// </summary>
public class PoseDebugVisualizer : MonoBehaviour
{
    public PiClient piClient;

    [Tooltip("Scale applied to raw camera_pos before placing the white marker.")]
    public float scale = 1f;

    [Tooltip("Flip Z axis (negate Z) before placing marker.")]
    public bool flipZ = false;

    [Tooltip("Flip Y axis (negate Y) before placing marker.")]
    public bool flipY = false;

    [Tooltip("Swap Y and Z components.")]
    public bool swapYZ = false;

    GameObject _camMarker;
    readonly object _lock = new();
    Vector3 _rawPos;
    bool _hasPos;

    void Start()
    {
        SpawnMarker("Origin",  Vector3.zero,              0.3f, Color.red);
        SpawnMarker("+X 1m",   Vector3.right,             0.2f, Color.green);
        SpawnMarker("+Y 1m",   Vector3.up,                0.2f, Color.blue);
        SpawnMarker("+Z 1m",   Vector3.forward,           0.2f, Color.yellow);

        _camMarker = SpawnMarker("CameraPos", Vector3.zero, 0.4f, Color.white);

        if (piClient != null)
            piClient.OnDetectionPayload += OnPayload;
    }

    void OnDestroy()
    {
        if (piClient != null)
            piClient.OnDetectionPayload -= OnPayload;
    }

    void OnPayload(string json)
    {
        try
        {
            var f = JsonUtility.FromJson<Frame>(json);
            if (f.camera_pos == null || f.camera_pos.Length < 3) return;
            lock (_lock)
            {
                _rawPos = new Vector3(f.camera_pos[0], f.camera_pos[1], f.camera_pos[2]);
                _hasPos = true;
            }
        }
        catch { /* ignore */ }
    }

    void Update()
    {
        if (!_hasPos || _camMarker == null) return;
        Vector3 p;
        lock (_lock) { p = _rawPos; }

        float x = p.x;
        float y = swapYZ ? p.z : p.y;
        float z = swapYZ ? p.y : p.z;
        if (flipY) y = -y;
        if (flipZ) z = -z;

        _camMarker.transform.position = new Vector3(x, y, z) * scale;
    }

    static GameObject SpawnMarker(string label, Vector3 pos, float size, Color color)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        go.name = $"[Debug] {label}";
        go.transform.position = pos;
        go.transform.localScale = Vector3.one * size;
        Destroy(go.GetComponent<Collider>());
        var mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
        if (mat.shader.name == "Hidden/InternalErrorShader")
            mat = new Material(Shader.Find("Standard"));
        mat.color = color;
        go.GetComponent<Renderer>().material = mat;
        return go;
    }

    [Serializable] class Frame { public float[] camera_pos; }
}
