using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DroneSubscriber : MonoBehaviour
{
    [Tooltip("ROS topic publishing nav_msgs/Odometry for the drone.")]
    public string topic = "/xray/uav/odom";

    [Tooltip("Optional: transform that acts as the world origin. Drone pose is applied relative to this. Leave empty to use Unity world origin.")]
    public Transform worldAnchor;

    ROSConnection ros;
    bool hasReceivedFirstMessage;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OdometryMsg>(topic, OnOdometry);
        Debug.Log($"DroneSubscriber: subscribed to {topic}");
    }

    void OnOdometry(OdometryMsg msg)
    {
        Vector3 unityPosition = msg.pose.pose.position.From<FLU>();
        Quaternion unityRotation = msg.pose.pose.orientation.From<FLU>();

        if (worldAnchor != null)
        {
            transform.position = worldAnchor.TransformPoint(unityPosition);
            transform.rotation = worldAnchor.rotation * unityRotation;
        }
        else
        {
            transform.position = unityPosition;
            transform.rotation = unityRotation;
        }

        if (!hasReceivedFirstMessage)
        {
            hasReceivedFirstMessage = true;
            Debug.Log($"DroneSubscriber: first odom received, position={unityPosition}");
        }
    }
}
