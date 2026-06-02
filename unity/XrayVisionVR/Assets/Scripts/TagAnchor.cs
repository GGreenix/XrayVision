using UnityEngine;

/// <summary>
/// Marks this GameObject as the physical AprilTag's pose in the room. Place it
/// where the tag sits in your scene and orient +Z (blue arrow) the way the tag
/// faces (out of the wall/surface). AprilTagCameraPoser uses this transform as
/// the world anchor — i.e. "the tag pose in the room".
///
/// Draws a magenta square (tag outline) and a blue forward arrow in the Scene
/// view so it's easy to position and aim.
/// </summary>
[DisallowMultipleComponent]
public class AprilTagAnchor : MonoBehaviour
{
    [Tooltip("AprilTag id this anchor represents (36h11). Must match the server's tag_id.")]
    public int tagId = 1;

    [Tooltip("Physical tag edge length in metres. Only affects the gizmo size.")]
    public float tagSizeMeters = 0.1f;

    /// <summary>The tag's pose in the Unity world.</summary>
    public Pose WorldPose => new Pose(transform.position, transform.rotation);

    void OnDrawGizmos()
    {
        float h = tagSizeMeters * 0.5f;
        Vector3 tl = transform.TransformPoint(new Vector3(-h,  h, 0f));
        Vector3 tr = transform.TransformPoint(new Vector3( h,  h, 0f));
        Vector3 br = transform.TransformPoint(new Vector3( h, -h, 0f));
        Vector3 bl = transform.TransformPoint(new Vector3(-h, -h, 0f));

        Gizmos.color = Color.magenta;
        Gizmos.DrawLine(tl, tr);
        Gizmos.DrawLine(tr, br);
        Gizmos.DrawLine(br, bl);
        Gizmos.DrawLine(bl, tl);

        // +Z = the direction the tag faces.
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(transform.position, transform.position + transform.forward * tagSizeMeters);
    }
}
