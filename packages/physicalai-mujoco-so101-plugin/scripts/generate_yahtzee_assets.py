from __future__ import annotations

import struct
import zlib
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent.parent / "urdf" / "scenes" / "yahtzee" / "assets"
OUT.mkdir(parents=True, exist_ok=True)


# ---------------------------------------------------------------------------
# Minimal PNG writer (no external dependencies)
# ---------------------------------------------------------------------------

def _write_png(path: Path, data: np.ndarray) -> None:
    h, w, channels = data.shape
    raw = b""
    for y in range(h):
        raw += b"\x00"  # filter byte (none)
        raw += data[y, :, :].tobytes()
    compressed = zlib.compress(raw)

    def chunk(ctype: bytes, cdata: bytes) -> bytes:
        c = ctype + cdata
        return struct.pack(">I", len(cdata)) + c + struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF)

    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)  # 8-bit RGB
    iend = b""
    path.write_bytes(sig + chunk(b"IHDR", ihdr) + chunk(b"IDAT", compressed) + chunk(b"IEND", iend))


# ---------------------------------------------------------------------------
# Die face textures (white with black dots)
# ---------------------------------------------------------------------------

def _draw_dot(arr: np.ndarray, cx: float, cy: float, r: float, color: tuple[int, ...]) -> None:
    h, w = arr.shape[:2]
    ys, xs = np.ogrid[:h, :w]
    cx_px, cy_px = int(cx * w), int(cy * h)
    r_px = max(1, int(r * min(w, h)))
    mask = (xs - cx_px) ** 2 + (ys - cy_px) ** 2 <= r_px**2
    arr[mask] = color


def _make_die_face(dots: list[tuple[float, float]], size: int = 64) -> np.ndarray:
    img = np.full((size, size, 3), 240, dtype=np.uint8)
    for cx, cy in dots:
        _draw_dot(img, cx, cy, 0.12, (0, 0, 0))
    return img


_DIE_FACES: list[list[tuple[float, float]]] = [
    # 1: centre
    [(0.5, 0.5)],
    # 2: diagonal
    [(0.25, 0.75), (0.75, 0.25)],
    # 3: diagonal
    [(0.25, 0.75), (0.5, 0.5), (0.75, 0.25)],
    # 4: four corners
    [(0.25, 0.25), (0.75, 0.25), (0.25, 0.75), (0.75, 0.75)],
    # 5: four corners + centre
    [(0.25, 0.25), (0.75, 0.25), (0.5, 0.5), (0.25, 0.75), (0.75, 0.75)],
    # 6: two columns of three
    [(0.25, 0.1667), (0.25, 0.5), (0.25, 0.8333),
     (0.75, 0.1667), (0.75, 0.5), (0.75, 0.8333)],
]

print("Generating die face textures ...")
for i, dots in enumerate(_DIE_FACES, 1):
    img = _make_die_face(dots)
    _write_png(OUT / f"die_face_{i}.png", img)
print("  done")


# ---------------------------------------------------------------------------
# Wood floor texture
# ---------------------------------------------------------------------------

def _make_wood_texture(size: int = 512) -> np.ndarray:
    rng = np.random.default_rng(42)
    img = np.zeros((size, size, 3), dtype=np.uint8)

    base = np.array([160, 130, 90], dtype=np.uint8)

    num_rings = 120
    ring_positions = np.sort(rng.uniform(0, size * 1.1, num_rings))
    ring_amplitudes = rng.uniform(2.0, 8.0, num_rings)
    ring_phases = rng.uniform(0, 2 * np.pi, num_rings)

    ys, xs = np.ogrid[:size, :size]
    dist = np.sqrt((xs - size * 0.5) ** 2 + (ys - size * 0.3) ** 2)

    noise = np.zeros((size, size), dtype=np.float32)
    for i in range(num_rings):
        noise += ring_amplitudes[i] * np.sin(dist * 0.3 + ring_positions[i] * 0.05 + ring_phases[i])

    noise += rng.normal(0, 3.0, (size, size)).astype(np.float32)
    noise = np.clip(noise, -30, 30)

    for c in range(3):
        img[:, :, c] = np.clip(base[c] + noise.astype(np.int16), 0, 255).astype(np.uint8)

    return img


print("Generating wood floor texture ...")
_write_png(OUT / "wood_floor.png", _make_wood_texture())
print("  done")


# ---------------------------------------------------------------------------
# Cup STL (binary)
# ---------------------------------------------------------------------------

def _write_binary_stl(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    n = len(faces)
    with path.open("wb") as f:
        f.write(b"\x00" * 80)
        f.write(struct.pack("<I", n))
        for tri in faces:
            v0, v1, v2 = vertices[tri]
            normal = np.cross(v1 - v0, v2 - v0)
            norm = np.linalg.norm(normal)
            if norm > 0:
                normal /= norm
            f.write(struct.pack("<3f", *normal))
            f.write(struct.pack("<3f", *v0))
            f.write(struct.pack("<3f", *v1))
            f.write(struct.pack("<3f", *v2))
            f.write(struct.pack("<H", 0))


def _make_cup_mesh(inner_r: float = 0.035, outer_r: float = 0.045,
                   height: float = 0.06, segments: int = 24) -> tuple[np.ndarray, np.ndarray]:
    verts: list[np.ndarray] = []
    faces: list[list[int]] = []

    base_idx = len(verts)
    for i in range(segments):
        theta = 2.0 * np.pi * i / segments
        ct, st = np.cos(theta), np.sin(theta)
        verts.append(np.array([outer_r * ct, outer_r * st, 0.0]))
        verts.append(np.array([inner_r * ct, inner_r * st, 0.0]))
        verts.append(np.array([outer_r * ct, outer_r * st, height]))
        verts.append(np.array([inner_r * ct, inner_r * st, height]))

    for i in range(segments):
        nxt = (i + 1) % segments
        b0 = base_idx + i * 4
        b1 = base_idx + nxt * 4

        # outer wall
        faces.append([b0 + 2, b1 + 0, b0 + 0])
        faces.append([b0 + 2, b1 + 2, b1 + 0])
        # inner wall (reverse winding)
        faces.append([b0 + 1, b1 + 1, b0 + 3])
        faces.append([b1 + 1, b1 + 3, b0 + 3])
        # top ring
        faces.append([b0 + 3, b1 + 2, b0 + 2])
        faces.append([b0 + 3, b1 + 3, b1 + 2])

    vertices = np.array(verts)
    faces_np = np.array(faces, dtype=np.int32)
    return vertices, faces_np


print("Generating cup STL ...")
verts, faces = _make_cup_mesh()
_write_binary_stl(OUT / "cup.stl", verts, faces)
print("  done")


print(f"\nAll assets written to {OUT}")
