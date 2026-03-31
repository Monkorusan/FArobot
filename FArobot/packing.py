from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Tuple


@dataclass(frozen=True)
class Box:
    box_id: str
    size_x: float
    size_y: float
    size_z: float


@dataclass(frozen=True)
class Bin:
    size_x: float
    size_y: float
    size_z: float
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_z: float = 0.0


@dataclass(frozen=True)
class Placement:
    box_id: str
    size_x: float
    size_y: float
    size_z: float
    center_x: float
    center_y: float
    center_z: float


def plan_packing(bin_spec: Bin, boxes: Iterable[Box]) -> Tuple[List[Placement], List[Box]]:
    """
    Simple 3D shelf heuristic. Boxes are axis-aligned with no rotation.
    Returns placements and any boxes that did not fit.
    """
    sorted_boxes = sorted(
        boxes,
        key=lambda b: (b.size_x * b.size_y * b.size_z),
        reverse=True,
    )

    placements: List[Placement] = []
    unplaced: List[Box] = []

    x_cursor = bin_spec.origin_x
    y_cursor = bin_spec.origin_y
    z_cursor = bin_spec.origin_z
    row_height = 0.0
    layer_height = 0.0

    max_x = bin_spec.origin_x + bin_spec.size_x
    max_y = bin_spec.origin_y + bin_spec.size_y
    max_z = bin_spec.origin_z + bin_spec.size_z

    for box in sorted_boxes:
        if box.size_x > bin_spec.size_x or box.size_y > bin_spec.size_y or box.size_z > bin_spec.size_z:
            unplaced.append(box)
            continue

        if x_cursor + box.size_x > max_x:
            x_cursor = bin_spec.origin_x
            y_cursor += row_height
            row_height = 0.0

        if y_cursor + box.size_y > max_y:
            y_cursor = bin_spec.origin_y
            z_cursor += layer_height
            layer_height = 0.0

        if z_cursor + box.size_z > max_z:
            unplaced.append(box)
            continue

        center_x = x_cursor + box.size_x * 0.5
        center_y = y_cursor + box.size_y * 0.5
        center_z = z_cursor + box.size_z * 0.5

        placements.append(
            Placement(
                box_id=box.box_id,
                size_x=box.size_x,
                size_y=box.size_y,
                size_z=box.size_z,
                center_x=center_x,
                center_y=center_y,
                center_z=center_z,
            )
        )

        x_cursor += box.size_x
        row_height = max(row_height, box.size_y)
        layer_height = max(layer_height, box.size_z)

    return placements, unplaced
