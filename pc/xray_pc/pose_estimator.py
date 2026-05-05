"""Mesh-based camera pose estimation using global registration + ICP.

Pipeline:
  1. Load the .obj room mesh and sample a dense point cloud + precompute FPFH.
  2. First call: FPFH + RANSAC global registration to coarsely align live depth
     cloud to mesh (handles unknown initial pose / mismatched coordinate systems).
  3. Subsequent calls: ICP refinement from the last known pose.
  4. Extract the refined camera pose from the result.
"""

import numpy as np
from pathlib import Path

try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    print("[pose_estimator] open3d not installed — ICP localization disabled", flush=True)

try:
    import trimesh
    TRIMESH_AVAILABLE = True
except ImportError:
    TRIMESH_AVAILABLE = False

from xray_pc.pose import Pose

_GLOBAL_VOXEL = 0.10   # voxel size (m) for FPFH global registration
_COARSE_THRESH = 0.50  # ICP threshold (m) after global reg
_FINE_THRESH   = 0.10  # ICP threshold (m) for subsequent refinement


class MeshLocalizer:
    def __init__(self, mesh_path: str, initial_pose: Pose,
                 n_mesh_points: int = 100_000, icp_threshold: float = 0.1):
        self.pose = initial_pose
        self.enabled = OPEN3D_AVAILABLE
        self.icp_threshold = icp_threshold
        self._global_T = None   # set once global registration succeeds

        if not self.enabled or not mesh_path:
            self.enabled = False
            print("[pose_estimator] disabled — using fixed pose from config", flush=True)
            return

        abs_path = str(Path(mesh_path).resolve())
        print(f"[pose_estimator] loading mesh {abs_path}", flush=True)

        if TRIMESH_AVAILABLE:
            tm = trimesh.load(abs_path, force="mesh")
            mesh = o3d.geometry.TriangleMesh()
            mesh.vertices = o3d.utility.Vector3dVector(np.array(tm.vertices, dtype=np.float64))
            mesh.triangles = o3d.utility.Vector3iVector(np.array(tm.faces, dtype=np.int32))
        else:
            mesh = o3d.io.read_triangle_mesh(abs_path)

        if len(mesh.triangles) == 0:
            print("[pose_estimator] mesh has no triangles — disabled", flush=True)
            self.enabled = False
            return

        mesh.compute_vertex_normals()
        self.mesh_pc = mesh.sample_points_uniformly(number_of_points=n_mesh_points)

        # Precompute downsampled mesh + FPFH features for global registration
        self._mesh_down = self.mesh_pc.voxel_down_sample(_GLOBAL_VOXEL)
        self._mesh_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=_GLOBAL_VOXEL * 2, max_nn=30))
        self._mesh_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            self._mesh_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=_GLOBAL_VOXEL * 5, max_nn=100))

        mesh_pts = np.asarray(self.mesh_pc.points)
        print(f"[pose_estimator] mesh loaded — {n_mesh_points} pts  "
              f"x=[{mesh_pts[:,0].min():.2f},{mesh_pts[:,0].max():.2f}] "
              f"y=[{mesh_pts[:,1].min():.2f},{mesh_pts[:,1].max():.2f}] "
              f"z=[{mesh_pts[:,2].min():.2f},{mesh_pts[:,2].max():.2f}]", flush=True)
        print(f"[pose_estimator] FPFH features ready — will run global registration on first depth frame", flush=True)

    # ------------------------------------------------------------------
    def refine(self, depth: np.ndarray, fx: float, fy: float,
               cx: float, cy: float) -> float:
        """Align live depth cloud to mesh. Returns fitness (0-1)."""
        if not self.enabled:
            return 0.0

        # Build camera-space point cloud
        h, w = depth.shape
        u_idx, v_idx = np.meshgrid(np.arange(w), np.arange(h))
        valid = depth > 0
        xc = (u_idx[valid] - cx) * depth[valid] / fx
        yc = (v_idx[valid] - cy) * depth[valid] / fy
        zc = depth[valid]
        pts_cam = np.stack([xc, yc, zc], axis=1)

        # Camera → world using current pose
        pts_world = (self.pose.R @ pts_cam.T).T + self.pose.position_m

        if len(pts_world) > 20_000:
            idx = np.random.choice(len(pts_world), 20_000, replace=False)
            pts_world = pts_world[idx]

        if len(pts_world) < 500:
            print(f"[pose_estimator] too few depth points ({len(pts_world)}) — skipping", flush=True)
            return 0.0

        live_pc = o3d.geometry.PointCloud()
        live_pc.points = o3d.utility.Vector3dVector(pts_world)

        live_pts = pts_world
        mesh_pts = np.asarray(self.mesh_pc.points)
        print(f"[icp_debug] live  n={len(live_pts)} "
              f"x=[{live_pts[:,0].min():.2f},{live_pts[:,0].max():.2f}] "
              f"y=[{live_pts[:,1].min():.2f},{live_pts[:,1].max():.2f}] "
              f"z=[{live_pts[:,2].min():.2f},{live_pts[:,2].max():.2f}]", flush=True)

        if self._global_T is None:
            return self._global_register_and_update(live_pc, pts_world)
        else:
            return self._icp_refine(live_pc, self._global_T, _FINE_THRESH)

    # ------------------------------------------------------------------
    def _global_register_and_update(self, live_pc, pts_world) -> float:
        print("[pose_estimator] running FPFH + RANSAC global registration…", flush=True)

        live_down = live_pc.voxel_down_sample(_GLOBAL_VOXEL)
        live_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=_GLOBAL_VOXEL * 2, max_nn=30))
        live_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            live_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=_GLOBAL_VOXEL * 5, max_nn=100))

        dist = _GLOBAL_VOXEL * 1.5
        ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            live_down, self._mesh_down, live_fpfh, self._mesh_fpfh,
            mutual_filter=True,
            max_correspondence_distance=dist,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            ransac_n=3,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist),
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100_000, 0.999),
        )
        print(f"[pose_estimator] RANSAC fitness={ransac.fitness:.3f}", flush=True)

        if ransac.fitness < 0.05:
            print("[pose_estimator] RANSAC fitness too low — will retry next cycle", flush=True)
            return ransac.fitness

        # Coarse ICP to tighten the RANSAC result
        fitness = self._icp_refine(live_pc, ransac.transformation, _COARSE_THRESH)
        if fitness > 0.1:
            self._global_T = np.eye(4)   # from now on use identity (pose already updated)
        else:
            print("[pose_estimator] coarse ICP failed after RANSAC — will retry global next cycle", flush=True)
        return fitness

    # ------------------------------------------------------------------
    def _icp_refine(self, live_pc, T_init: np.ndarray, threshold: float) -> float:
        result = o3d.pipelines.registration.registration_icp(
            live_pc, self.mesh_pc,
            threshold,
            T_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        )
        print(f"[pose_estimator] ICP fitness={result.fitness:.3f} rmse={result.inlier_rmse:.4f} "
              f"(threshold={threshold}m)", flush=True)

        if result.fitness > 0.15:
            T = result.transformation
            R_new = T[:3, :3] @ self.pose.R
            t_new = T[:3, :3] @ self.pose.position_m + T[:3, 3]
            self.pose.update(R_new, t_new)
            print(f"[pose_estimator] pose updated → pos={self.pose.position_m}", flush=True)
        else:
            print("[pose_estimator] ICP fitness too low — pose unchanged", flush=True)

        return result.fitness
