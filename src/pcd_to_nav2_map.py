#!/usr/bin/env python3
"""
Offline converter: PCD point cloud -> Nav2 PGM/YAML occupancy map.

Supports PCD DATA ascii and DATA binary (not binary_compressed).
Uses x, y, z fields and optionally estimates the floor height.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from pathlib import Path

import numpy as np

 
PCD_NUMPY_TYPES = {
    ("F", 4): np.dtype("<f4"),
    ("F", 8): np.dtype("<f8"),
    ("I", 1): np.dtype("<i1"),
    ("I", 2): np.dtype("<i2"),
    ("I", 4): np.dtype("<i4"),
    ("I", 8): np.dtype("<i8"),
    ("U", 1): np.dtype("<u1"),
    ("U", 2): np.dtype("<u2"),
    ("U", 4): np.dtype("<u4"),
    ("U", 8): np.dtype("<u8"),
}


def parse_header(path: Path):
    header = {}
    header_lines = []
    data_offset = 0

    with path.open("rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError("PCD header ended before DATA line.")
            data_offset = f.tell()

            text = line.decode("ascii", errors="strict").strip()
            header_lines.append(text)

            if not text or text.startswith("#"):
                continue

            key, *values = text.split()
            header[key.upper()] = values

            if key.upper() == "DATA":
                break

    required = ("FIELDS", "SIZE", "TYPE", "COUNT", "POINTS", "DATA")
    missing = [key for key in required if key not in header]
    if missing:
        raise RuntimeError(f"Missing PCD header entries: {missing}")

    fields = header["FIELDS"]
    sizes = [int(v) for v in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(v) for v in header["COUNT"]]
    points = int(header["POINTS"][0])
    data_mode = header["DATA"][0].lower()

    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise RuntimeError("FIELDS/SIZE/TYPE/COUNT lengths do not match.")

    return fields, sizes, types, counts, points, data_mode, data_offset, header_lines


def build_dtype(fields, sizes, types, counts):
    dtype_fields = []
    for field, size, pcd_type, count in zip(fields, sizes, types, counts):
        base = PCD_NUMPY_TYPES.get((pcd_type.upper(), size))
        if base is None:
            raise RuntimeError(f"Unsupported PCD type: field={field}, TYPE={pcd_type}, SIZE={size}")
        if count == 1:
            dtype_fields.append((field, base))
        else:
            dtype_fields.append((field, base, (count,)))
    return np.dtype(dtype_fields)


def load_xyz(path: Path):
    fields, sizes, types, counts, points, data_mode, data_offset, _ = parse_header(path)

    for name in ("x", "y", "z"):
        if name not in fields:
            raise RuntimeError(f"PCD has no '{name}' field. FIELDS={fields}")

    if data_mode == "binary_compressed":
        raise RuntimeError(
            "DATA binary_compressed is not supported by this script. "
            "Convert it first with pcl_convert_pcd_ascii_binary."
        )

    if data_mode == "binary":
        dtype = build_dtype(fields, sizes, types, counts)
        expected_bytes = points * dtype.itemsize
        with path.open("rb") as f:
            f.seek(data_offset)
            raw = f.read(expected_bytes)
        if len(raw) < expected_bytes:
            raise RuntimeError(
                f"PCD data is truncated: expected {expected_bytes} bytes, got {len(raw)} bytes."
            )
        cloud = np.frombuffer(raw, dtype=dtype, count=points)
        xyz = np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(np.float64, copy=False)

    elif data_mode == "ascii":
        field_columns = {}
        column = 0
        for field, count in zip(fields, counts):
            field_columns[field] = column
            column += count

        data = np.loadtxt(path, comments="#", skiprows=len(parse_header(path)[7]), dtype=np.float64)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        xyz = data[:, [field_columns["x"], field_columns["y"], field_columns["z"]]]

    else:
        raise RuntimeError(f"Unsupported DATA mode: {data_mode}")

    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    if xyz.size == 0:
        raise RuntimeError("No finite XYZ points found.")
    return xyz


def estimate_floor_z(z: np.ndarray, bin_size: float = 0.03) -> float:
    """Estimate the dominant lower horizontal layer using a z histogram."""
    z_lo, z_hi = np.percentile(z, [1.0, 70.0])
    if not np.isfinite(z_lo) or not np.isfinite(z_hi) or z_hi <= z_lo:
        return float(np.percentile(z, 5.0))

    bins = max(10, int(math.ceil((z_hi - z_lo) / bin_size)))
    hist, edges = np.histogram(z[(z >= z_lo) & (z <= z_hi)], bins=bins, range=(z_lo, z_hi))
    centers = 0.5 * (edges[:-1] + edges[1:])

    # Prefer a strong peak in the lower 45% of the inspected z range.
    cutoff = z_lo + 0.45 * (z_hi - z_lo)
    mask = centers <= cutoff
    if not np.any(mask):
        return float(np.percentile(z, 5.0))
    idx_local = int(np.argmax(hist[mask]))
    return float(centers[mask][idx_local])


def dilate(binary: np.ndarray, radius_cells: int) -> np.ndarray:
    if radius_cells <= 0:
        return binary

    h, w = binary.shape
    out = np.zeros_like(binary, dtype=bool)
    offsets = []
    r2 = radius_cells * radius_cells
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= r2:
                offsets.append((dy, dx))

    for dy, dx in offsets:
        src_y0 = max(0, -dy)
        src_y1 = min(h, h - dy)
        src_x0 = max(0, -dx)
        src_x1 = min(w, w - dx)

        dst_y0 = max(0, dy)
        dst_y1 = min(h, h + dy)
        dst_x0 = max(0, dx)
        dst_x1 = min(w, w + dx)

        out[dst_y0:dst_y1, dst_x0:dst_x1] |= binary[src_y0:src_y1, src_x0:src_x1]

    return out


def save_pgm(path: Path, image: np.ndarray):
    if image.dtype != np.uint8 or image.ndim != 2:
        raise ValueError("PGM image must be a 2D uint8 array.")
    h, w = image.shape
    with path.open("wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(image.tobytes(order="C"))


def main():
    parser = argparse.ArgumentParser(
        description="Convert a PCD map into Nav2-compatible PGM/YAML."
    )
    parser.add_argument("input_pcd", type=Path)
    parser.add_argument("output_prefix", type=Path,
                        help="Output prefix, e.g. ~/cpr_auto_ws/maps/fastlio_nav2")
    parser.add_argument("--resolution", type=float, default=0.05,
                        help="Map resolution in meters/cell. Default: 0.05")
    parser.add_argument("--floor-z", type=float, default=None,
                        help="Known floor z in PCD coordinates. Omit to auto-estimate.")
    parser.add_argument("--min-height", type=float, default=0.10,
                        help="Obstacle slice lower height above floor. Default: 0.10 m")
    parser.add_argument("--max-height", type=float, default=1.50,
                        help="Obstacle slice upper height above floor. Default: 1.50 m")
    parser.add_argument("--padding", type=float, default=0.50,
                        help="Free border around cloud bounds. Default: 0.50 m")
    parser.add_argument("--min-points-per-cell", type=int, default=1,
                        help="Points required to mark a cell occupied. Default: 1")
    parser.add_argument("--inflate", type=float, default=0.00,
                        help="Optional obstacle dilation radius in meters. Prefer Nav2 inflation; default 0.")
    
    parser.add_argument("--x-min", type=float, default=None,
                    help="Minimum X coordinate to keep.")
    parser.add_argument("--x-max", type=float, default=None,
                        help="Maximum X coordinate to keep.")
    parser.add_argument("--y-min", type=float, default=None,
                        help="Minimum Y coordinate to keep.")
    parser.add_argument("--y-max", type=float, default=None,
                        help="Maximum Y coordinate to keep.")
    
    args = parser.parse_args()

    if args.resolution <= 0:
        parser.error("--resolution must be > 0")
    if args.max_height <= args.min_height:
        parser.error("--max-height must be greater than --min-height")
    if args.min_points_per_cell < 1:
        parser.error("--min-points-per-cell must be >= 1")

    input_path = args.input_pcd.expanduser().resolve()
    output_prefix = args.output_prefix.expanduser().resolve()
    output_prefix.parent.mkdir(parents=True, exist_ok=True)

    print(f"[LOAD] {input_path}")
    xyz = load_xyz(input_path)
    print(f"[INFO] finite points: {len(xyz):,}")

    # Optional XY crop
    crop_mask = np.ones(len(xyz), dtype=bool)

    if args.x_min is not None:
        crop_mask &= xyz[:, 0] >= args.x_min
    if args.x_max is not None:
        crop_mask &= xyz[:, 0] <= args.x_max
    if args.y_min is not None:
        crop_mask &= xyz[:, 1] >= args.y_min
    if args.y_max is not None:
        crop_mask &= xyz[:, 1] <= args.y_max

    xyz = xyz[crop_mask]

    if len(xyz) == 0:
        raise RuntimeError("No points remain after XY cropping.")

    print(f"[INFO] points after XY crop: {len(xyz):,}")

    print(
        "[INFO] bounds: "
        f"x=[{xyz[:,0].min():.3f}, {xyz[:,0].max():.3f}], "
        f"y=[{xyz[:,1].min():.3f}, {xyz[:,1].max():.3f}], "
        f"z=[{xyz[:,2].min():.3f}, {xyz[:,2].max():.3f}]"
    )
    print(
        "[INFO] z percentiles: "
        + ", ".join(
            f"p{p:g}={np.percentile(xyz[:,2], p):.3f}"
            for p in (1, 5, 25, 50, 75, 95, 99)
        )
    )

    floor_z = args.floor_z if args.floor_z is not None else estimate_floor_z(xyz[:, 2])
    z_min = floor_z + args.min_height
    z_max = floor_z + args.max_height
    print(f"[INFO] floor_z={floor_z:.3f} m")
    print(f"[INFO] obstacle slice z=[{z_min:.3f}, {z_max:.3f}] m")

    selected = xyz[(xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)]
    if len(selected) == 0:
        raise RuntimeError(
            "No points remain after z filtering. Set --floor-z, --min-height, and --max-height manually."
        )
    print(f"[INFO] selected obstacle points: {len(selected):,}")

    min_x = args.x_min if args.x_min is not None else float(xyz[:, 0].min() - args.padding)
    max_x = args.x_max if args.x_max is not None else float(xyz[:, 0].max() + args.padding)
    min_y = args.y_min if args.y_min is not None else float(xyz[:, 1].min() - args.padding)
    max_y = args.y_max if args.y_max is not None else float(xyz[:, 1].max() + args.padding) 

    width = int(math.ceil((max_x - min_x) / args.resolution))
    height = int(math.ceil((max_y - min_y) / args.resolution))
    if width <= 0 or height <= 0:
        raise RuntimeError("Invalid map dimensions.")

    ix = np.floor((selected[:, 0] - min_x) / args.resolution).astype(np.int64)
    iy = np.floor((selected[:, 1] - min_y) / args.resolution).astype(np.int64)
    valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    ix = ix[valid]
    iy = iy[valid]

    counts = np.zeros((height, width), dtype=np.uint32)
    np.add.at(counts, (iy, ix), 1)
    occupied = counts >= args.min_points_per_cell

    inflate_cells = int(math.ceil(args.inflate / args.resolution))
    occupied = dilate(occupied, inflate_cells)

    # Nav map convention: free=254 (white), occupied=0 (black).
    image = np.full((height, width), 254, dtype=np.uint8)
    image[occupied] = 0

    # PGM row 0 is top, while the map origin is bottom-left.
    image = np.flipud(image)

    pgm_path = output_prefix.with_suffix(".pgm")
    yaml_path = output_prefix.with_suffix(".yaml")
    save_pgm(pgm_path, image)

    yaml_text = (
        f"image: {pgm_path.name}\n"
        "mode: trinary\n"
        f"resolution: {args.resolution:.6f}\n"
        f"origin: [{min_x:.6f}, {min_y:.6f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )
    yaml_path.write_text(yaml_text, encoding="utf-8")

    occupied_cells = int(np.count_nonzero(occupied))
    print(f"[SAVE] {pgm_path}")
    print(f"[SAVE] {yaml_path}")
    print(f"[INFO] grid: {width} x {height}, resolution={args.resolution:.3f} m")
    print(f"[INFO] occupied cells: {occupied_cells:,}")
    print("[DONE]")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        sys.exit(1)
