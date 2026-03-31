from __future__ import annotations

from dataclasses import dataclass
from typing import List

import yaml

from FArobot.packing import Bin, Box


@dataclass(frozen=True)
class SourceSpec:
    origin_x: float
    origin_y: float
    origin_z: float
    spacing_x: float
    spacing_y: float
    cols: int
    rows: int


@dataclass(frozen=True)
class PackingConfig:
    bin_spec: Bin
    source_spec: SourceSpec
    boxes: List[Box]


def load_config(yaml_path: str) -> PackingConfig:
    with open(yaml_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    bin_data = data.get("bin", {})
    bin_size = bin_data.get("size", [0.6, 0.4, 0.3])
    bin_origin = bin_data.get("origin", [0.0, 0.0, 0.0])

    bin_spec = Bin(
        size_x=float(bin_size[0]),
        size_y=float(bin_size[1]),
        size_z=float(bin_size[2]),
        origin_x=float(bin_origin[0]),
        origin_y=float(bin_origin[1]),
        origin_z=float(bin_origin[2]),
    )

    source_data = data.get("source", {})
    source_origin = source_data.get("origin", [-0.4, -0.2, 0.05])
    source_spacing = source_data.get("spacing", [0.12, 0.12])
    source_cols = int(source_data.get("cols", 3))
    source_rows = int(source_data.get("rows", 2))

    source_spec = SourceSpec(
        origin_x=float(source_origin[0]),
        origin_y=float(source_origin[1]),
        origin_z=float(source_origin[2]),
        spacing_x=float(source_spacing[0]),
        spacing_y=float(source_spacing[1]),
        cols=max(source_cols, 1),
        rows=max(source_rows, 1),
    )

    boxes: List[Box] = []
    for box in data.get("boxes", []):
        size = box.get("size", [0.1, 0.1, 0.1])
        box_id = str(box.get("id", f"box_{len(boxes):02d}"))
        boxes.append(
            Box(
                box_id=box_id,
                size_x=float(size[0]),
                size_y=float(size[1]),
                size_z=float(size[2]),
            )
        )

    return PackingConfig(bin_spec=bin_spec, source_spec=source_spec, boxes=boxes)
