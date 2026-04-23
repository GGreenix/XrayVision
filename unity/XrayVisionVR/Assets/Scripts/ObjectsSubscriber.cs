using System.Collections.Generic;
using RosMessageTypes.XrayInterfaces;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class ObjectsSubscriber : MonoBehaviour
{
    [Tooltip("ROS topic publishing xray_interfaces/TrackedObjectArray.")]
    public string topic = "/xray/world/objects";

    [Tooltip("Optional: transform that acts as the world origin. Markers are placed relative to this.")]
    public Transform worldAnchor;

    [Tooltip("Prefab used for each marker. If empty, a primitive sphere is created.")]
    public GameObject markerPrefab;

    [Tooltip("Drop markers whose tracking_id hasn't been seen for this many updates.")]
    public int staleUpdates = 3;

    ROSConnection ros;
    readonly Dictionary<string, GameObject> markersById = new();
    readonly Dictionary<string, int> missesById = new();

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TrackedObjectArrayMsg>(topic, OnArray);
        Debug.Log($"ObjectsSubscriber: subscribed to {topic}");
    }

    void OnArray(TrackedObjectArrayMsg msg)
    {
        var seenThisTick = new HashSet<string>();

        foreach (var obj in msg.objects)
        {
            string id = string.IsNullOrEmpty(obj.tracking_id) ? obj.class_id : obj.tracking_id;
            if (string.IsNullOrEmpty(id)) continue;
            seenThisTick.Add(id);

            Vector3 unityPosition = obj.pose.position.From<FLU>();
            Quaternion unityRotation = obj.pose.orientation.From<FLU>();

            Vector3 unityScale = new Vector3(
                Mathf.Max((float)obj.dimensions.y, 0.1f),
                Mathf.Max((float)obj.dimensions.z, 0.1f),
                Mathf.Max((float)obj.dimensions.x, 0.1f));

            if (!markersById.TryGetValue(id, out var marker) || marker == null)
            {
                marker = CreateMarker(id, obj.class_id);
                markersById[id] = marker;
            }

            if (worldAnchor != null)
            {
                marker.transform.position = worldAnchor.TransformPoint(unityPosition);
                marker.transform.rotation = worldAnchor.rotation * unityRotation;
            }
            else
            {
                marker.transform.position = unityPosition;
                marker.transform.rotation = unityRotation;
            }
            marker.transform.localScale = unityScale;

            missesById[id] = 0;
        }

        var toRemove = new List<string>();
        foreach (var kv in markersById)
        {
            if (seenThisTick.Contains(kv.Key)) continue;
            missesById.TryGetValue(kv.Key, out int misses);
            misses++;
            missesById[kv.Key] = misses;
            if (misses > staleUpdates) toRemove.Add(kv.Key);
        }
        foreach (var id in toRemove)
        {
            if (markersById.TryGetValue(id, out var m) && m != null) Destroy(m);
            markersById.Remove(id);
            missesById.Remove(id);
        }
    }

    GameObject CreateMarker(string trackingId, string classId)
    {
        GameObject marker;
        if (markerPrefab != null)
        {
            marker = Instantiate(markerPrefab, transform);
        }
        else
        {
            marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            marker.transform.SetParent(transform, worldPositionStays: false);
            var renderer = marker.GetComponent<Renderer>();
            if (renderer != null)
            {
                var mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
                mat.color = ColorFromString(classId);
                renderer.material = mat;
            }
            var collider = marker.GetComponent<Collider>();
            if (collider != null) Destroy(collider);
        }
        marker.name = $"marker_{trackingId}";
        return marker;
    }

    static Color ColorFromString(string s)
    {
        if (string.IsNullOrEmpty(s)) return Color.magenta;
        unchecked
        {
            int hash = s.GetHashCode();
            float h = ((hash & 0xFF) / 255.0f);
            float sat = 0.6f + ((hash >> 8) & 0x3F) / 255.0f;
            float v = 0.8f + ((hash >> 16) & 0x1F) / 255.0f;
            return Color.HSVToRGB(h, Mathf.Clamp01(sat), Mathf.Clamp01(v));
        }
    }
}
