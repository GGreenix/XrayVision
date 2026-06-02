using System;
using UnityEngine;

public class AprilTagCameraPoser : MonoBehaviour
{
    public PiClient piClient;

    [Tooltip("The camera (or a parent rig) to move.")]
    public Transform cameraRig;

    [Tooltip("The tag object placed in the room — its transform is the anchor.")]
    public AprilTagAnchor tagAnchor;

    [Tooltip("Fallback raw anchor if no Tag Anchor assigned. Null = world origin.")]
    public Transform anchor;

    Transform AnchorTransform => tagAnchor != null ? tagAnchor.transform : anchor;

    [Tooltip("Apply the tag's rotation too.")]
    public bool applyRotation = true;

    [Range(0f, 30f)] public float smoothing = 10f;

    [Tooltip("Extra rotation offset (Euler degrees) applied after.")]
    public Vector3 rotationOffsetEuler;

    [Header("PhotonVision Pose")]
    [Tooltip("Use the PhotonVision camera world pose from the 'camera' field. Takes priority over the tag pose.")]
    public bool usePhotonVisionPose = false;

    [Header("Debug — Hardcoded Pose")]
    [Tooltip("When true, ignores the server pose and uses the values below.")]
    public bool useHardcodedPose = true;
    public Vector3 hardcodedPosition = new Vector3(-26.83f, 22.7f, -13.6f);
    public Vector3 hardcodedRotationEuler = new Vector3(0f, 0f, 0f);

    [Header("Debug — Camera Marker")]
    [Tooltip("Shows a red sphere at the camera rig position in the scene.")]
    public bool showCameraMarker = true;
    private GameObject _cameraMarker;

    private Vector3 targetPos;
    private Quaternion targetRot = Quaternion.identity;
    private volatile bool hasTarget;
    private bool hasValidTarget;   // never moved the rig until a real pose arrives
    private bool loggedFirstCamera;
    private Vector3 pendingPos;
    private Quaternion pendingRot = Quaternion.identity;

    void Start()
    {
        if (useHardcodedPose)
        {
            targetPos = hardcodedPosition;
            targetRot = Quaternion.Euler(hardcodedRotationEuler);
        }

        if (showCameraMarker)
        {
            _cameraMarker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            _cameraMarker.name = "CameraMarker";
            Destroy(_cameraMarker.GetComponent<Collider>());
            _cameraMarker.transform.localScale = Vector3.one * 0.3f;
            var mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            mat.color = Color.red;
            _cameraMarker.GetComponent<Renderer>().material = mat;
        }
    }

    void OnEnable()  { if (piClient != null) piClient.OnDetectionPayload += OnPayload; }
    void OnDisable() { if (piClient != null) piClient.OnDetectionPayload -= OnPayload; }

    // PhotonVision world pose arrives pre-composed (not tag-relative).
    private volatile bool pendingIsWorld;

    void OnPayload(string payload)
    {
        if (useHardcodedPose) return;

        try
        {
            var data = JsonUtility.FromJson<Frame>(payload);
            if (data == null) return;

            // Preferred: PhotonVision world camera pose from the "camera" field.
            if (usePhotonVisionPose && data.camera != null
                && data.camera.pos != null && data.camera.pos.Length == 3)
            {
                pendingPos = new Vector3(data.camera.pos[0], data.camera.pos[1], data.camera.pos[2]);
                pendingRot = (data.camera.rot != null && data.camera.rot.Length == 4)
                    ? new Quaternion(data.camera.rot[0], data.camera.rot[1], data.camera.rot[2], data.camera.rot[3])
                    : Quaternion.identity;
                pendingIsWorld = true;
                hasTarget = true;
                if (!loggedFirstCamera)
                {
                    Debug.Log($"[AprilTagCameraPoser] first PhotonVision camera pose: {pendingPos}");
                    loggedFirstCamera = true;
                }
                return;
            }

            // Fallback: my own AprilTag pose, composed against the anchor.
            var tag = data.apriltag;
            if (tag == null || tag.id <= 0 || tag.cam_pos == null || tag.cam_pos.Length < 3)
                return;
            if (tagAnchor != null && tag.id != tagAnchor.tagId)
                return;

            pendingPos = new Vector3(tag.cam_pos[0], tag.cam_pos[1], tag.cam_pos[2]);
            pendingRot = (tag.cam_rot != null && tag.cam_rot.Length == 4)
                ? new Quaternion(tag.cam_rot[0], tag.cam_rot[1], tag.cam_rot[2], tag.cam_rot[3])
                : Quaternion.identity;
            pendingIsWorld = false;
            hasTarget = true;
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[AprilTagCameraPoser] parse error: {e.Message}");
        }
    }

    void Update()
    {
        if (cameraRig == null) return;

        if (!useHardcodedPose && hasTarget)
        {
            if (pendingIsWorld)
            {
                // PhotonVision pose is already in Unity world coordinates.
                targetPos = pendingPos;
                targetRot = pendingRot;
            }
            else
            {
                var a = AnchorTransform;
                if (a != null)
                {
                    targetPos = a.TransformPoint(pendingPos);
                    targetRot = a.rotation * pendingRot;
                }
                else
                {
                    targetPos = pendingPos;
                    targetRot = pendingRot;
                }
            }
            targetRot *= Quaternion.Euler(rotationOffsetEuler);
            hasTarget = false;
            hasValidTarget = true;
        }

        // Don't move the rig until we've actually received a pose — otherwise
        // it drifts toward (0,0,0) while waiting for data.
        if (!hasValidTarget && !useHardcodedPose) return;

        float k = smoothing <= 0f ? 1f : 1f - Mathf.Exp(-smoothing * Time.deltaTime);
        cameraRig.position = Vector3.Lerp(cameraRig.position, targetPos, k);
        if (applyRotation)
            cameraRig.rotation = Quaternion.Slerp(cameraRig.rotation, targetRot, k);

        if (_cameraMarker != null)
            _cameraMarker.transform.position = cameraRig.position;
    }

    [Serializable] private class Frame
    {
        public AprilTag apriltag;
        public CameraPose camera;
    }
    [Serializable] private class AprilTag
    {
        public int id;
        public float[] cam_pos;
        public float[] cam_rot;
    }
    [Serializable] private class CameraPose
    {
        public float[] pos;
        public float[] rot;
    }
}
