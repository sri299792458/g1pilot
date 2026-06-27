#!/usr/bin/env python3
"""Reachability map loading and query utilities for G1 hand targets."""

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class ReachabilityQueryResult:
    side: str
    target: np.ndarray
    reachable: bool
    neighbors: int
    nearest_distance: float
    nearest_position: np.ndarray | None
    nearest_config: np.ndarray | None


class ReachabilityMap:
    """KD-tree-backed 3D reachability map in the pelvis frame."""

    def __init__(self, positions, sides, configs=None, metadata=None):
        self.positions = np.asarray(positions, dtype=np.float64)
        if self.positions.ndim != 2 or self.positions.shape[1] != 3:
            raise ValueError("positions must have shape (N, 3)")

        self.sides = np.asarray(sides).astype(str)
        if self.sides.shape[0] != self.positions.shape[0]:
            raise ValueError("sides length must match positions")

        if configs is None:
            self.configs = None
        else:
            self.configs = np.asarray(configs, dtype=np.float64)
            if self.configs.shape[0] != self.positions.shape[0]:
                raise ValueError("configs length must match positions")

        self.metadata = dict(metadata or {})
        self._trees = {}
        self._indices = {}
        for side in sorted(set(self.sides.tolist())):
            indices = np.flatnonzero(self.sides == side)
            if len(indices) == 0:
                continue
            self._indices[side] = indices
            self._trees[side] = cKDTree(self.positions[indices])

    @classmethod
    def load(cls, path):
        path = Path(path)
        with np.load(path, allow_pickle=False) as data:
            metadata = {}
            if "metadata_json" in data:
                metadata = json.loads(str(data["metadata_json"]))
            configs = data["configs"] if "configs" in data else None
            return cls(
                positions=data["positions"],
                sides=data["sides"],
                configs=configs,
                metadata=metadata,
            )

    def query(self, side, target, radius=0.04, min_neighbors=1):
        side = str(side).lower()
        target = np.asarray(target, dtype=np.float64).reshape(3)
        if side not in self._trees:
            return ReachabilityQueryResult(
                side=side,
                target=target,
                reachable=False,
                neighbors=0,
                nearest_distance=float("inf"),
                nearest_position=None,
                nearest_config=None,
            )

        tree = self._trees[side]
        local_indices = self._indices[side]
        neighbor_local = tree.query_ball_point(target, float(radius))
        nearest_distance, nearest_local = tree.query(target, k=1)
        nearest_global = int(local_indices[int(nearest_local)])
        nearest_config = None
        if self.configs is not None:
            nearest_config = self.configs[nearest_global].copy()

        return ReachabilityQueryResult(
            side=side,
            target=target,
            reachable=len(neighbor_local) >= int(min_neighbors),
            neighbors=len(neighbor_local),
            nearest_distance=float(nearest_distance),
            nearest_position=self.positions[nearest_global].copy(),
            nearest_config=nearest_config,
        )

    def side_count(self, side):
        return int(len(self._indices.get(str(side).lower(), [])))

    def summary(self):
        lines = [f"ReachabilityMap samples={len(self.positions):,}"]
        for side in sorted(self._indices):
            pts = self.positions[self._indices[side]]
            lines.append(
                f"  {side}: {len(pts):,} samples, "
                f"x=[{pts[:, 0].min():.3f}, {pts[:, 0].max():.3f}], "
                f"y=[{pts[:, 1].min():.3f}, {pts[:, 1].max():.3f}], "
                f"z=[{pts[:, 2].min():.3f}, {pts[:, 2].max():.3f}]"
            )
        return "\n".join(lines)
