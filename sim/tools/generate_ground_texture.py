#!/usr/bin/env python3
"""Generate the seamless, multi-scale ground texture used by sim/models/flow_ground.

Why: PX4's simulated optical-flow sensor is a real camera + OpenCV feature
tracker. Over the stock worlds' flat grey ground plane it finds nothing to track,
and flow-only estimation diverges. This texture has high-contrast features from
~1 cm to ~1 m so flow works from touchdown (~0.4 m footprint) to ~10 m altitude.

Offline asset tooling (not part of any flight path). Deterministic: same seed,
same PNG. Tileable: the model repeats it every TILE_M metres.
"""
import argparse

import numpy as np
from PIL import Image


def periodic_value_noise(rng, size, cells):
    """Bilinearly-interpolated random grid that wraps seamlessly at the edges."""
    grid = rng.random((cells, cells))
    coords = np.arange(size) * cells / size
    i0 = np.floor(coords).astype(int)
    f = coords - i0
    i1 = (i0 + 1) % cells
    f = f * f * (3 - 2 * f)  # smoothstep, avoids visible grid creases
    top = grid[i0][:, i0] * (1 - f)[None, :] + grid[i0][:, i1] * f[None, :]
    bot = grid[i1][:, i0] * (1 - f)[None, :] + grid[i1][:, i1] * f[None, :]
    return top * (1 - f)[:, None] + bot * f[:, None]


def stamp_shapes(rng, img, count, r_min, r_max):
    """Random filled discs and squares, wrapped around the edges (tileable)."""
    size = img.shape[0]
    yy, xx = np.mgrid[0:size, 0:size]
    for _ in range(count):
        cx, cy = rng.integers(0, size, 2)
        r = rng.integers(r_min, r_max + 1)
        shade = rng.choice([0.05, 0.25, 0.75, 0.95])
        dx = (xx - cx + size // 2) % size - size // 2  # wrapped distance
        dy = (yy - cy + size // 2) % size - size // 2
        if rng.random() < 0.5:
            mask = dx * dx + dy * dy <= r * r
        else:
            mask = (np.abs(dx) <= r) & (np.abs(dy) <= r)
        img[mask] = shade


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("out")
    ap.add_argument("--size", type=int, default=1024)
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    n = args.size
    # Octaves of noise: large patches (~1 m) down to fine grain (~2 cm at 5 m/tile).
    img = (0.45 * periodic_value_noise(rng, n, 8)
           + 0.30 * periodic_value_noise(rng, n, 32)
           + 0.25 * periodic_value_noise(rng, n, 128))
    img = (img - img.min()) / (img.max() - img.min())
    stamp_shapes(rng, img, count=120, r_min=4, r_max=40)
    # Per-pixel grain keeps corners trackable even inside flat shapes.
    img = np.clip(img + rng.normal(0.0, 0.06, img.shape), 0.0, 1.0)
    Image.fromarray((img * 255).astype(np.uint8), mode="L").convert("RGB").save(args.out, optimize=True)
    print(f"wrote {args.out} ({n}x{n}, seed {args.seed})")


if __name__ == "__main__":
    main()
