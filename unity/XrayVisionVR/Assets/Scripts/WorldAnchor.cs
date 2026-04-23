using UnityEngine;
using UnityEngine.InputSystem;

public class WorldAnchor : MonoBehaviour
{
    [Tooltip("Transform tracked to the user's head (usually the XR Origin's Main Camera). If empty, falls back to Camera.main at runtime.")]
    public Transform head;

    [Tooltip("Input action for re-anchoring. Default binding in the script: keyboard Space + XR right-controller secondary button (B on Quest).")]
    public InputActionProperty recenterAction = new InputActionProperty(
        new InputAction(
            name: "Recenter",
            type: InputActionType.Button,
            binding: "<Keyboard>/space",
            interactions: "press"));

    [Tooltip("If true, zero out roll and pitch — the anchor only takes yaw from the head pose. Recommended: keeps ROS coordinates flat.")]
    public bool yawOnly = true;

    [Tooltip("If true, project the anchor onto y=0 so the ROS ground plane lines up with the user's floor.")]
    public bool snapToGround = true;

    [Tooltip("Optional vertical offset applied after snap-to-ground, e.g., to raise the anchor to eye height.")]
    public float groundOffsetY = 0.0f;

    void OnEnable()
    {
        recenterAction.action.AddBinding("<XRController>{RightHand}/secondaryButton");
        recenterAction.action.Enable();
    }

    void OnDisable()
    {
        recenterAction.action.Disable();
    }

    void Update()
    {
        if (recenterAction.action.WasPressedThisFrame())
        {
            Recenter();
        }
    }

    [ContextMenu("Recenter Now")]
    public void Recenter()
    {
        Transform source = head != null ? head : (Camera.main != null ? Camera.main.transform : null);
        if (source == null)
        {
            Debug.LogWarning("WorldAnchor: no head transform and no Camera.main. Set 'head' in the Inspector.");
            return;
        }

        Vector3 position = source.position;
        if (snapToGround)
        {
            position.y = groundOffsetY;
        }

        Quaternion rotation = source.rotation;
        if (yawOnly)
        {
            rotation = Quaternion.Euler(0f, source.eulerAngles.y, 0f);
        }

        transform.SetPositionAndRotation(position, rotation);
        Debug.Log($"WorldAnchor: recentered to position={position}, yaw={transform.eulerAngles.y:F1}°");
    }
}
