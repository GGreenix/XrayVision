#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

/// <summary>
/// Adds "GameObject > XrayVision > AprilTag Anchor" so the tag object can be
/// created with one click and placed in the scene.
/// </summary>
public static class TagAnchorMenu
{
    [MenuItem("GameObject/XrayVision/AprilTag Anchor", false, 10)]
    public static void CreateTagAnchor(MenuCommand cmd)
    {
        var go = new GameObject("AprilTag Anchor (id 1)");
        go.AddComponent<AprilTagAnchor>();
        GameObjectUtility.SetParentAndAlign(go, cmd.context as GameObject);
        Undo.RegisterCreatedObjectUndo(go, "Create AprilTag Anchor");
        Selection.activeObject = go;
    }
}
#endif
