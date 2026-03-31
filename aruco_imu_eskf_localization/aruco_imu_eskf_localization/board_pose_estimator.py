from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from aruco_imu_eskf_localization.frame_conventions import vector_leader_rear_from_board


def invert_observation(rvec, tvec) -> tuple[np.ndarray, np.ndarray]:
    rotation_cam_from_board = Rotation.from_rotvec(np.asarray(rvec, dtype=float).reshape(3))
    rotation_board_from_cam = rotation_cam_from_board.inv()
    translation_cam_from_board = np.asarray(tvec, dtype=float).reshape(3)
    translation_board_from_cam = -rotation_board_from_cam.as_matrix() @ translation_cam_from_board
    return rotation_board_from_cam.as_rotvec(), translation_board_from_cam


def board_pose_delta(
    previous_pose: tuple[np.ndarray, np.ndarray],
    current_pose: tuple[np.ndarray, np.ndarray],
) -> tuple[float, float, float]:
    previous_rvec, previous_tvec = previous_pose
    current_rvec, current_tvec = current_pose

    position_delta_m = float(
        np.linalg.norm(np.asarray(current_tvec, dtype=float) - np.asarray(previous_tvec, dtype=float))
    )

    previous_rotation = Rotation.from_rotvec(np.asarray(previous_rvec, dtype=float))
    current_rotation = Rotation.from_rotvec(np.asarray(current_rvec, dtype=float))
    relative_rotation = current_rotation * previous_rotation.inv()
    rotation_delta_deg = float(np.degrees(relative_rotation.magnitude()))

    previous_yaw_deg = float(
        previous_rotation.as_euler('ZYX', degrees=True)[0]
    )
    current_yaw_deg = float(
        current_rotation.as_euler('ZYX', degrees=True)[0]
    )
    yaw_delta_deg = float((current_yaw_deg - previous_yaw_deg + 180.0) % 360.0 - 180.0)
    return position_delta_m, rotation_delta_deg, abs(yaw_delta_deg)


@dataclass(frozen=True)
class BoardPoseEstimate:
    rvec: np.ndarray
    tvec: np.ndarray
    visible_markers: int
    reprojection_rmse_px: float
    image_area_px: float
    used_single_marker_fallback: bool

    @property
    def board_pose(self) -> tuple[np.ndarray, np.ndarray]:
        return invert_observation(self.rvec, self.tvec)


class BoardDefinition:
    def __init__(
        self,
        marker_sizes_m: dict[int, float],
        marker_centers_m: dict[int, np.ndarray],
        description: str = '',
    ) -> None:
        self.marker_sizes_m = marker_sizes_m
        self.marker_centers_m = marker_centers_m
        self.description = description

    @classmethod
    def from_config(cls, config: dict) -> 'BoardDefinition':
        marker_sizes_m = {
            int(marker['id']): float(marker['size']) / 1000.0
            for marker in config['markers']
        }
        board_geometry = config.get('board_geometry') or {}
        marker_centers_m = {
            int(marker_id): np.asarray(position, dtype=float) / 1000.0
            for marker_id, position in (board_geometry.get('marker_positions') or {}).items()
        }
        return cls(
            marker_sizes_m=marker_sizes_m,
            marker_centers_m=marker_centers_m,
            description=str(board_geometry.get('description', '')),
        )

    def marker_object_points(self, marker_id: int) -> np.ndarray:
        center = self.marker_centers_m[int(marker_id)]
        half_size = self.marker_sizes_m[int(marker_id)] * 0.5
        # Board frame: +x right, +y up, +z outward toward the follower vehicle.
        # Marker corners are ordered as top-left, top-right, bottom-right, bottom-left
        # from the follower-facing view of the board.
        return np.array(
            [
                [center[0] - half_size, center[1] + half_size, center[2]],
                [center[0] + half_size, center[1] + half_size, center[2]],
                [center[0] + half_size, center[1] - half_size, center[2]],
                [center[0] - half_size, center[1] - half_size, center[2]],
            ],
            dtype=np.float64,
        )

    def known_detections(
        self,
        corners,
        ids,
    ) -> list[tuple[int, np.ndarray]]:
        if ids is None:
            return []

        detections: list[tuple[int, np.ndarray]] = []
        for index, marker_id in enumerate(ids.flatten()):
            marker_id = int(marker_id)
            if marker_id not in self.marker_sizes_m or marker_id not in self.marker_centers_m:
                continue
            detections.append(
                (
                    marker_id,
                    np.asarray(corners[index], dtype=np.float64).reshape(4, 2),
                )
            )
        return detections

    def estimate_pose(
        self,
        corners,
        ids,
        camera_matrix,
        dist_coeffs,
        previous_board_pose: tuple[np.ndarray, np.ndarray] | None = None,
        min_markers: int = 1,
        min_markers_to_initialize: int = 2,
        max_position_jump_m: float = 0.35,
        max_rotation_jump_deg: float = 55.0,
        max_yaw_jump_deg: float = 40.0,
        front_halfspace_min_z_m: float = 0.05,
        max_view_angle_deg: float = 75.0,
        feasible_x_min_m: float = 0.10,
        feasible_x_max_m: float = 3.50,
        feasible_abs_y_max_m: float = 1.00,
        feasible_z_min_m: float = -0.50,
        feasible_z_max_m: float = 0.80,
    ) -> BoardPoseEstimate | None:
        detections = self.known_detections(corners, ids)
        if len(detections) < max(1, int(min_markers)):
            return None

        if previous_board_pose is None and len(detections) < max(1, int(min_markers_to_initialize)):
            return None

        if len(detections) == 1:
            return self._estimate_single_marker_pose(
                detections[0],
                camera_matrix,
                dist_coeffs,
                previous_board_pose,
                max_position_jump_m=max_position_jump_m,
                max_rotation_jump_deg=max_rotation_jump_deg,
                max_yaw_jump_deg=max_yaw_jump_deg,
                front_halfspace_min_z_m=front_halfspace_min_z_m,
                max_view_angle_deg=max_view_angle_deg,
                feasible_x_min_m=feasible_x_min_m,
                feasible_x_max_m=feasible_x_max_m,
                feasible_abs_y_max_m=feasible_abs_y_max_m,
                feasible_z_min_m=feasible_z_min_m,
                feasible_z_max_m=feasible_z_max_m,
            )

        return self._estimate_multi_marker_pose(
            detections,
            camera_matrix,
            dist_coeffs,
            previous_board_pose,
            max_position_jump_m=max_position_jump_m,
            max_rotation_jump_deg=max_rotation_jump_deg,
            max_yaw_jump_deg=max_yaw_jump_deg,
            front_halfspace_min_z_m=front_halfspace_min_z_m,
            max_view_angle_deg=max_view_angle_deg,
            feasible_x_min_m=feasible_x_min_m,
            feasible_x_max_m=feasible_x_max_m,
            feasible_abs_y_max_m=feasible_abs_y_max_m,
            feasible_z_min_m=feasible_z_min_m,
            feasible_z_max_m=feasible_z_max_m,
        )

    def _estimate_multi_marker_pose(
        self,
        detections: list[tuple[int, np.ndarray]],
        camera_matrix,
        dist_coeffs,
        previous_board_pose: tuple[np.ndarray, np.ndarray] | None,
        max_position_jump_m: float,
        max_rotation_jump_deg: float,
        max_yaw_jump_deg: float,
        front_halfspace_min_z_m: float,
        max_view_angle_deg: float,
        feasible_x_min_m: float,
        feasible_x_max_m: float,
        feasible_abs_y_max_m: float,
        feasible_z_min_m: float,
        feasible_z_max_m: float,
    ) -> BoardPoseEstimate | None:
        object_points = np.vstack(
            [self.marker_object_points(marker_id) for marker_id, _ in detections]
        ).astype(np.float64)
        image_points = np.vstack([marker_corners for _, marker_corners in detections]).astype(np.float64)

        image_area_px = self._image_area_px(detections)
        candidates = self._solve_ippe_candidates(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            image_area_px=image_area_px,
            visible_markers=len(detections),
            used_single_marker_fallback=False,
            flags=cv2.SOLVEPNP_IPPE,
        )
        if not candidates:
            iterative_candidate = self._solve_iterative_candidate(
                object_points,
                image_points,
                camera_matrix,
                dist_coeffs,
                previous_board_pose=previous_board_pose,
                image_area_px=image_area_px,
                visible_markers=len(detections),
            )
            if iterative_candidate is not None:
                candidates = [iterative_candidate]

        return self._select_best_candidate(
            candidates,
            previous_board_pose=previous_board_pose,
            max_position_jump_m=max_position_jump_m,
            max_rotation_jump_deg=max_rotation_jump_deg,
            max_yaw_jump_deg=max_yaw_jump_deg,
            front_halfspace_min_z_m=front_halfspace_min_z_m,
            max_view_angle_deg=max_view_angle_deg,
            feasible_x_min_m=feasible_x_min_m,
            feasible_x_max_m=feasible_x_max_m,
            feasible_abs_y_max_m=feasible_abs_y_max_m,
            feasible_z_min_m=feasible_z_min_m,
            feasible_z_max_m=feasible_z_max_m,
        )

    def _estimate_single_marker_pose(
        self,
        detection: tuple[int, np.ndarray],
        camera_matrix,
        dist_coeffs,
        previous_board_pose: tuple[np.ndarray, np.ndarray] | None,
        max_position_jump_m: float,
        max_rotation_jump_deg: float,
        max_yaw_jump_deg: float,
        front_halfspace_min_z_m: float,
        max_view_angle_deg: float,
        feasible_x_min_m: float,
        feasible_x_max_m: float,
        feasible_abs_y_max_m: float,
        feasible_z_min_m: float,
        feasible_z_max_m: float,
    ) -> BoardPoseEstimate | None:
        if previous_board_pose is None:
            return None

        marker_id, marker_corners = detection
        object_points = self.marker_object_points(marker_id)
        candidates = self._solve_ippe_candidates(
            object_points,
            marker_corners,
            camera_matrix,
            dist_coeffs,
            image_area_px=self._image_area_px([detection]),
            visible_markers=1,
            used_single_marker_fallback=True,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        return self._select_best_candidate(
            candidates,
            previous_board_pose=previous_board_pose,
            max_position_jump_m=max_position_jump_m,
            max_rotation_jump_deg=max_rotation_jump_deg,
            max_yaw_jump_deg=max_yaw_jump_deg,
            front_halfspace_min_z_m=front_halfspace_min_z_m,
            max_view_angle_deg=max_view_angle_deg,
            feasible_x_min_m=feasible_x_min_m,
            feasible_x_max_m=feasible_x_max_m,
            feasible_abs_y_max_m=feasible_abs_y_max_m,
            feasible_z_min_m=feasible_z_min_m,
            feasible_z_max_m=feasible_z_max_m,
        )

    def _solve_ippe_candidates(
        self,
        object_points: np.ndarray,
        image_points: np.ndarray,
        camera_matrix,
        dist_coeffs,
        image_area_px: float,
        visible_markers: int,
        used_single_marker_fallback: bool,
        flags: int,
    ) -> list[BoardPoseEstimate]:
        try:
            result = cv2.solvePnPGeneric(
                object_points,
                image_points.reshape(-1, 1, 2),
                camera_matrix,
                dist_coeffs,
                flags=flags,
            )
        except Exception:
            return []

        if len(result) >= 4:
            success, rvecs, tvecs, reprojection_errors = result[:4]
        elif len(result) == 3:
            success, rvecs, tvecs = result
            reprojection_errors = None
        else:
            return []

        if not success:
            return []

        estimates: list[BoardPoseEstimate] = []
        for index, (candidate_rvec, candidate_tvec) in enumerate(zip(rvecs, tvecs)):
            reprojection_rmse_px = self._reprojection_rmse(
                object_points,
                image_points,
                camera_matrix,
                dist_coeffs,
                candidate_rvec,
                candidate_tvec,
            )
            if reprojection_errors is not None and index < len(reprojection_errors):
                reprojection_rmse_px = max(
                    reprojection_rmse_px,
                    float(np.asarray(reprojection_errors[index]).reshape(-1)[0]),
                )
            estimates.append(
                BoardPoseEstimate(
                    rvec=np.asarray(candidate_rvec, dtype=np.float64).reshape(3),
                    tvec=np.asarray(candidate_tvec, dtype=np.float64).reshape(3),
                    visible_markers=visible_markers,
                    reprojection_rmse_px=reprojection_rmse_px,
                    image_area_px=image_area_px,
                    used_single_marker_fallback=used_single_marker_fallback,
                )
            )
        return estimates

    def _solve_iterative_candidate(
        self,
        object_points: np.ndarray,
        image_points: np.ndarray,
        camera_matrix,
        dist_coeffs,
        previous_board_pose: tuple[np.ndarray, np.ndarray] | None,
        image_area_px: float,
        visible_markers: int,
    ) -> BoardPoseEstimate | None:
        try:
            if previous_board_pose is not None:
                previous_rvec, previous_tvec = previous_board_pose
                previous_rvec, previous_tvec = invert_observation(previous_rvec, previous_tvec)
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points.reshape(-1, 1, 2),
                    camera_matrix,
                    dist_coeffs,
                    rvec=np.asarray(previous_rvec, dtype=np.float64).reshape(3, 1),
                    tvec=np.asarray(previous_tvec, dtype=np.float64).reshape(3, 1),
                    useExtrinsicGuess=True,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
            else:
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points.reshape(-1, 1, 2),
                    camera_matrix,
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
        except Exception:
            return None

        if not success:
            return None

        return BoardPoseEstimate(
            rvec=np.asarray(rvec, dtype=np.float64).reshape(3),
            tvec=np.asarray(tvec, dtype=np.float64).reshape(3),
            visible_markers=visible_markers,
            reprojection_rmse_px=self._reprojection_rmse(
                object_points,
                image_points,
                camera_matrix,
                dist_coeffs,
                rvec,
                tvec,
            ),
            image_area_px=image_area_px,
            used_single_marker_fallback=False,
        )

    def _select_best_candidate(
        self,
        candidates: list[BoardPoseEstimate],
        previous_board_pose: tuple[np.ndarray, np.ndarray] | None,
        max_position_jump_m: float,
        max_rotation_jump_deg: float,
        max_yaw_jump_deg: float,
        front_halfspace_min_z_m: float,
        max_view_angle_deg: float,
        feasible_x_min_m: float,
        feasible_x_max_m: float,
        feasible_abs_y_max_m: float,
        feasible_z_min_m: float,
        feasible_z_max_m: float,
    ) -> BoardPoseEstimate | None:
        ranked_candidates: list[tuple[float, BoardPoseEstimate]] = []
        for candidate in candidates:
            candidate_board_pose = candidate.board_pose
            if not self._passes_front_visibility_gate(
                candidate_board_pose,
                front_halfspace_min_z_m=front_halfspace_min_z_m,
                max_view_angle_deg=max_view_angle_deg,
            ):
                continue
            if not self._passes_feasible_box_gate(
                candidate_board_pose,
                feasible_x_min_m=feasible_x_min_m,
                feasible_x_max_m=feasible_x_max_m,
                feasible_abs_y_max_m=feasible_abs_y_max_m,
                feasible_z_min_m=feasible_z_min_m,
                feasible_z_max_m=feasible_z_max_m,
            ):
                continue
            if previous_board_pose is not None and not self._passes_motion_gate(
                previous_board_pose,
                candidate_board_pose,
                max_position_jump_m=max_position_jump_m,
                max_rotation_jump_deg=max_rotation_jump_deg,
                max_yaw_jump_deg=max_yaw_jump_deg,
            ):
                continue
            score = self._candidate_score(
                previous_board_pose,
                candidate_board_pose,
                candidate.reprojection_rmse_px,
            )
            ranked_candidates.append((score, candidate))

        if not ranked_candidates:
            return None

        ranked_candidates.sort(key=lambda item: item[0])
        return ranked_candidates[0][1]

    def _candidate_score(
        self,
        previous_board_pose: tuple[np.ndarray, np.ndarray] | None,
        candidate_board_pose: tuple[np.ndarray, np.ndarray],
        reprojection_rmse_px: float,
    ) -> float:
        if previous_board_pose is None:
            return reprojection_rmse_px

        position_delta_m, rotation_delta_deg, yaw_delta_deg = board_pose_delta(
            previous_board_pose,
            candidate_board_pose,
        )
        return (
            position_delta_m * 8.0
            + rotation_delta_deg / 30.0
            + yaw_delta_deg / 20.0
            + reprojection_rmse_px * 0.2
        )

    def _passes_front_visibility_gate(
        self,
        candidate_board_pose: tuple[np.ndarray, np.ndarray],
        front_halfspace_min_z_m: float,
        max_view_angle_deg: float,
    ) -> bool:
        _, candidate_tvec = candidate_board_pose
        distance_m = float(np.linalg.norm(candidate_tvec))
        if distance_m <= 1.0e-9:
            return False

        if float(candidate_tvec[2]) <= float(front_halfspace_min_z_m):
            return False

        cos_view_angle = float(candidate_tvec[2]) / distance_m
        min_cos_view_angle = float(np.cos(np.deg2rad(max_view_angle_deg)))
        return cos_view_angle >= min_cos_view_angle

    def _passes_feasible_box_gate(
        self,
        candidate_board_pose: tuple[np.ndarray, np.ndarray],
        feasible_x_min_m: float,
        feasible_x_max_m: float,
        feasible_abs_y_max_m: float,
        feasible_z_min_m: float,
        feasible_z_max_m: float,
    ) -> bool:
        _, candidate_tvec = candidate_board_pose
        leader_rear_position = vector_leader_rear_from_board(candidate_tvec)
        longitudinal_x = float(leader_rear_position[0])
        lateral_y = float(leader_rear_position[1])
        vertical_z = float(leader_rear_position[2])
        if longitudinal_x < float(feasible_x_min_m) or longitudinal_x > float(feasible_x_max_m):
            return False
        if abs(lateral_y) > float(feasible_abs_y_max_m):
            return False
        if vertical_z < float(feasible_z_min_m) or vertical_z > float(feasible_z_max_m):
            return False
        return True

    def _passes_motion_gate(
        self,
        previous_board_pose: tuple[np.ndarray, np.ndarray],
        candidate_board_pose: tuple[np.ndarray, np.ndarray],
        max_position_jump_m: float,
        max_rotation_jump_deg: float,
        max_yaw_jump_deg: float,
    ) -> bool:
        position_delta_m, rotation_delta_deg, yaw_delta_deg = board_pose_delta(
            previous_board_pose,
            candidate_board_pose,
        )
        if position_delta_m > float(max_position_jump_m):
            return False
        if rotation_delta_deg > float(max_rotation_jump_deg):
            return False
        if yaw_delta_deg > float(max_yaw_jump_deg):
            return False
        return True

    def _image_area_px(self, detections: list[tuple[int, np.ndarray]]) -> float:
        all_points = np.concatenate([marker_corners for _, marker_corners in detections], axis=0)
        hull = cv2.convexHull(all_points.astype(np.float32).reshape(-1, 1, 2))
        return float(cv2.contourArea(hull))

    def _reprojection_rmse(
        self,
        object_points: np.ndarray,
        image_points: np.ndarray,
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
    ) -> float:
        projected_points, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            camera_matrix,
            dist_coeffs,
        )
        projected_points = projected_points.reshape(-1, 2)
        errors = projected_points - image_points.reshape(-1, 2)
        return float(np.sqrt(np.mean(np.sum(errors**2, axis=1))))
