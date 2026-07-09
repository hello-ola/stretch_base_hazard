"""Velocity filtering against robot-centric hazard points."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class HazardVelFilterConfig:
    """Geometry used to decide whether a base velocity points into a hazard."""

    lookahead_m: float = 0.50
    cliff_lookahead_m: float = 0.80
    footprint_m: float = 0.35
    obstacle_buffer_m: float = 0.02
    cliff_buffer_m: float = 0.20
    min_linear_speed_mps: float = 1e-4


@dataclass(frozen=True)
class HazardVelFilterResult:
    """Filtered base velocity and the reason it was limited."""

    vx: float
    vy: float
    wz: float
    blocked_by_obstacle: bool = False
    blocked_by_cliff: bool = False
    blocking_obstacle_count: int = 0
    blocking_cliff_count: int = 0


@dataclass(frozen=True)
class HazardHeadingBlockage:
    """Hazard counts for one candidate base translation direction."""

    blocked_by_obstacle: bool = False
    blocked_by_cliff: bool = False
    blocking_obstacle_count: int = 0
    blocking_cliff_count: int = 0


def filter_hazard_velocity(
    vx: float,
    vy: float,
    wz: float,
    obstacle_xy: np.ndarray,
    cliff_xy: np.ndarray,
    config: HazardVelFilterConfig | None = None,
) -> HazardVelFilterResult:
    """
    Remove commanded linear motion that points into known hazards.

    Points are expected in the robot base frame, with +X forward and +Y left.
    Rotation is preserved because the under-base hazard map constrains
    translational motion rather than in-place yaw.
    """
    cfg = config or HazardVelFilterConfig()
    linear = np.asarray([float(vx), float(vy)], dtype=np.float64)
    speed = float(np.linalg.norm(linear))
    if speed <= cfg.min_linear_speed_mps:
        return HazardVelFilterResult(float(vx), float(vy), float(wz))

    heading = linear / speed
    blockage = classify_hazard_heading(heading, obstacle_xy, cliff_xy, cfg)

    if blockage.blocked_by_obstacle or blockage.blocked_by_cliff:
        linear = linear - float(np.dot(linear, heading)) * heading
        linear[np.abs(linear) <= cfg.min_linear_speed_mps] = 0.0

    return HazardVelFilterResult(
        vx=float(linear[0]),
        vy=float(linear[1]),
        wz=float(wz),
        blocked_by_obstacle=blockage.blocked_by_obstacle,
        blocked_by_cliff=blockage.blocked_by_cliff,
        blocking_obstacle_count=blockage.blocking_obstacle_count,
        blocking_cliff_count=blockage.blocking_cliff_count,
    )


def classify_hazard_heading(
    heading_xy: np.ndarray,
    obstacle_xy: np.ndarray,
    cliff_xy: np.ndarray,
    config: HazardVelFilterConfig | None = None,
) -> HazardHeadingBlockage:
    """Classify whether translation along a heading would be blocked."""
    cfg = config or HazardVelFilterConfig()
    heading = np.asarray(heading_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(heading))
    if norm <= cfg.min_linear_speed_mps:
        return HazardHeadingBlockage()
    heading = heading / norm

    obstacle_count = _count_blockers(
        obstacle_xy,
        heading,
        lookahead_m=cfg.lookahead_m,
        lateral_limit_m=cfg.footprint_m + cfg.obstacle_buffer_m,
    )
    cliff_count = _count_blockers(
        cliff_xy,
        heading,
        lookahead_m=cfg.cliff_lookahead_m,
        lateral_limit_m=cfg.footprint_m + cfg.cliff_buffer_m,
    )
    return HazardHeadingBlockage(
        blocked_by_obstacle=obstacle_count > 0,
        blocked_by_cliff=cliff_count > 0,
        blocking_obstacle_count=obstacle_count,
        blocking_cliff_count=cliff_count,
    )


def _count_blockers(
    points_xy: np.ndarray,
    heading: np.ndarray,
    *,
    lookahead_m: float,
    lateral_limit_m: float,
) -> int:
    points = _as_xy(points_xy)
    if len(points) == 0:
        return 0

    lateral_axis = np.asarray([-heading[1], heading[0]], dtype=np.float64)
    along = points @ heading
    lateral = np.abs(points @ lateral_axis)
    finite = np.isfinite(points).all(axis=1)
    blocking = (
        finite
        & (along > 0.0)
        & (along <= max(float(lookahead_m), 0.0))
        & (lateral <= max(float(lateral_limit_m), 0.0))
    )
    return int(np.count_nonzero(blocking))


def _as_xy(points_xy: np.ndarray) -> np.ndarray:
    points = np.asarray(points_xy, dtype=np.float64)
    if points.size == 0:
        return np.zeros((0, 2), dtype=np.float64)
    if points.ndim == 1:
        points = points.reshape(1, -1)
    if points.shape[1] < 2:
        raise ValueError('hazard points must have at least x and y columns')
    return np.ascontiguousarray(points[:, :2], dtype=np.float64)
