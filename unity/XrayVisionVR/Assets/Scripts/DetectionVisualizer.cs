using System;
using System.Collections.Generic;
using UnityEngine;

public class DetectionVisualizer : MonoBehaviour
{
    public PiClient piClient;
    public Material targetMaterial;
    public float scale = 0.2f;

    private Dictionary<string, GameObject> activeTargets = new();
    private Queue<DetectionFrame> frameQueue = new();
    private object lockObj = new();

    void OnEnable()
    {
        if (piClient != null)
            piClient.OnDetectionPayload += HandleDetections;
    }

    void OnDisable()
    {
        if (piClient != null)
            piClient.OnDetectionPayload -= HandleDetections;
    }

    void HandleDetections(string payload)
    {
        try
        {
            var data = JsonUtility.FromJson<DetectionFrame>(payload);
            if (data?.objects == null) return;
            lock (lockObj)
            {
                frameQueue.Enqueue(data);
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[DetectionVisualizer] parse error: {e.Message}");
        }
    }

    void Update()
    {
        lock (lockObj)
        {
            while (frameQueue.Count > 0)
            {
                var frame = frameQueue.Dequeue();
                ProcessDetections(frame);
            }
        }
    }

    void ProcessDetections(DetectionFrame frame)
    {
        var seenIds = new HashSet<string>();
        foreach (var obj in frame.objects)
        {
            seenIds.Add(obj.tracking_id);
            if (!activeTargets.ContainsKey(obj.tracking_id))
                CreateTarget(obj.tracking_id, obj);
            else
                UpdateTarget(obj.tracking_id, obj);
        }

        var toRemove = new List<string>();
        foreach (var id in activeTargets.Keys)
        {
            if (!seenIds.Contains(id))
                toRemove.Add(id);
        }
        foreach (var id in toRemove)
        {
            Destroy(activeTargets[id]);
            activeTargets.Remove(id);
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

        activeTargets[id] = sphere;
        UpdateTarget(id, obj);
    }

    void UpdateTarget(string id, Detection obj)
    {
        if (activeTargets.TryGetValue(id, out var sphere))
            sphere.transform.position = new Vector3(obj.x, obj.z, obj.y);
    }

    [Serializable]
    public class DetectionFrame
    {
        public double t;
        public Detection[] objects;
    }

    [Serializable]
    public class Detection
    {
        public string tracking_id;
        public string class_id;
        public float confidence;
        public float x, y, z;
        public float depth_m;
        public BBox bbox;
    }

    [Serializable]
    public class BBox
    {
        public float u_norm, v_norm, w_norm, h_norm;
    }
}
