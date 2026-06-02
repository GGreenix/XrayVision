using System;
using System.Collections.Generic;
using UnityEngine;

public class DetectionVisualizer : MonoBehaviour
{
    public PiClient piClient;
    public Material targetMaterial;
    public float scale = 0.2f;

    [Header("AprilTag marker")]
    public Transform cameraTransform;
    public Material apriltagMaterial;
    public float apriltagScale = 0.15f;

    // All Unity API calls must happen on the main thread.
    // WebSocket callbacks cache pending state; Update() applies it.
    private DetectionFrame _pending;
    private readonly object _pendingLock = new object();

    private Dictionary<string, GameObject> _activeTargets = new();
    private GameObject _apriltagMarker;

    void OnEnable()  { if (piClient != null) piClient.OnDetectionPayload += OnPayload; }
    void OnDisable() { if (piClient != null) piClient.OnDetectionPayload -= OnPayload; }

    void OnPayload(string payload)
    {
        try
        {
            var data = JsonUtility.FromJson<DetectionFrame>(payload);
            lock (_pendingLock) _pending = data;
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[DetectionVisualizer] parse error: {e.Message}");
        }
    }

    void Update()
    {
        DetectionFrame data;
        lock (_pendingLock)
        {
            data = _pending;
            _pending = null;
        }
        if (data == null) return;

        ApplyAprilTag(data.apriltag);

        if (data.objects != null && data.objects.Length > 0)
            ApplyDetections(data.objects);
    }

    void ApplyAprilTag(AprilTag tag)
    {
        if (tag == null || tag.id <= 0)
        {
            if (_apriltagMarker != null) _apriltagMarker.SetActive(false);
            return;
        }

        if (_apriltagMarker == null)
        {
            _apriltagMarker = GameObject.CreatePrimitive(PrimitiveType.Cube);
            _apriltagMarker.name = $"apriltag-{tag.id}";
            Destroy(_apriltagMarker.GetComponent<Collider>());
            _apriltagMarker.transform.localScale = Vector3.one * apriltagScale;
            if (apriltagMaterial != null)
                _apriltagMarker.GetComponent<Renderer>().material = apriltagMaterial;
        }

        _apriltagMarker.SetActive(true);
        var pos = new Vector3(tag.x, tag.y, tag.z);
        if (cameraTransform != null)
        {
            _apriltagMarker.transform.SetParent(cameraTransform, false);
            _apriltagMarker.transform.localPosition = pos;
        }
        else
        {
            _apriltagMarker.transform.SetParent(transform, false);
            _apriltagMarker.transform.position = pos;
        }
    }

    void ApplyDetections(Detection[] objects)
    {
        var seen = new HashSet<string>();
        foreach (var obj in objects)
        {
            var id = obj.tracking_id;
            seen.Add(id);
            if (!_activeTargets.ContainsKey(id))
                CreateTarget(id, obj);
            else
                UpdateTarget(id, obj);
        }

        foreach (var id in new List<string>(_activeTargets.Keys))
        {
            if (!seen.Contains(id))
            {
                Destroy(_activeTargets[id]);
                _activeTargets.Remove(id);
            }
        }
    }

    void CreateTarget(string id, Detection obj)
    {
        var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.name = id;
        sphere.transform.SetParent(transform);
        Destroy(sphere.GetComponent<Collider>());
        sphere.transform.localScale = Vector3.one * scale;
        if (targetMaterial != null)
            sphere.GetComponent<Renderer>().material = targetMaterial;
        _activeTargets[id] = sphere;
        UpdateTarget(id, obj);
    }

    void UpdateTarget(string id, Detection obj)
    {
        _activeTargets[id].transform.position = new Vector3(obj.x, obj.z, obj.y);
    }

    [Serializable] public class DetectionFrame
    {
        public double t;
        public Detection[] objects;
        public AprilTag apriltag;
    }

    [Serializable] public class AprilTag
    {
        public int id;
        public float x, y, z;
        public float distance_m;
        public float[] rvec;
    }

    [Serializable] public class Detection
    {
        public string tracking_id;
        public string class_id;
        public float confidence;
        public float x, y, z;
        public float depth_m;
        public BBox bbox;
    }

    [Serializable] public class BBox
    {
        public float u_norm, v_norm, w_norm, h_norm;
    }
}
