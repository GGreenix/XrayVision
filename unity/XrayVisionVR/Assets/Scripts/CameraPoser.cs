using System;
using UnityEngine;

/// <summary>
/// Moves a Unity Transform to match the estimated real-world camera pose
/// received from the PC pipeline via WebSocket.
///
/// Assign this to any GameObject. Set Target to your XR rig, camera proxy,
/// or any object you want to track the physical camera.
///
/// Coordinate conversion: Scaniverse OBJ (right-handed, Y-up) → Unity (left-handed, Y-up): negate Z.
/// </summary>
public class CameraPoser : MonoBehaviour
{
    [Tooltip("The PiClient driving the WebSocket connection.")]
    public PiClient piClient;

    [Tooltip("Transform to reposition — e.g. your XR rig root or a camera proxy object. Leave empty to move this GameObject.")]
    public Transform target;

    [Tooltip("Scale factor matching how your Scaniverse mesh was imported into Unity.")]
    public float positionScale = 10f;

    [Tooltip("Lerp factor per frame (0 = snap immediately, higher = more smoothing).")]
    [Range(0f, 20f)]
    public float smoothSpeed = 8f;

    Vector3    _wantPos;
    Quaternion _wantRot = Quaternion.identity;
    bool       _hasPose;
    readonly object _lock = new();

    void OnEnable()
    {
        if (piClient != null)
            piClient.OnDetectionPayload += OnPayload;
    }

    void OnDisable()
    {
        if (piClient != null)
            piClient.OnDetectionPayload -= OnPayload;
    }

    void OnPayload(string json)
    {
        try
        {
            var frame = JsonUtility.FromJson<PoseFrame>(json);
            if (frame.camera_pos == null || frame.camera_pos.Length < 3) return;

            // Scaniverse OBJ is Z-up: (X right, Y forward, Z up)
            // Unity is Y-up:         (X right, Y up,      Z forward)
            // Conversion: swap Y and Z.
            float px = frame.camera_pos[0];
            float py = frame.camera_pos[1];
            float pz = frame.camera_pos[2];
            var pos = new Vector3(px, pz, py) * positionScale;

            var rot = Quaternion.identity;
            if (frame.camera_rot_q != null && frame.camera_rot_q.Length == 4)
            {
                // Swap Y/Z components of the quaternion axis to match Z-up → Y-up
                rot = new Quaternion(
                     frame.camera_rot_q[0],
                     frame.camera_rot_q[2],
                     frame.camera_rot_q[1],
                     frame.camera_rot_q[3]);
            }

            lock (_lock) { _wantPos = pos; _wantRot = rot; _hasPose = true; }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[CameraPoser] parse error: {e.Message}");
        }
    }

    void Update()
    {
        if (!_hasPose) return;

        Vector3    pos;
        Quaternion rot;
        lock (_lock) { pos = _wantPos; rot = _wantRot; }

        var t = target != null ? target : transform;
        float alpha = smoothSpeed <= 0f ? 1f : 1f - Mathf.Exp(-smoothSpeed * Time.deltaTime);
        t.position = Vector3.Lerp(t.position, pos, alpha);
        t.rotation = Quaternion.Slerp(t.rotation, rot, alpha);
    }

    [Serializable]
    class PoseFrame
    {
        public float[] camera_pos;
        public float[] camera_rot_q;
    }
}
