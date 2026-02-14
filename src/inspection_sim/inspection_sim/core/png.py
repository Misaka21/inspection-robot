import struct
import zlib


_PNG_SIG = b"\x89PNG\r\n\x1a\n"


def _chunk(chunk_type: bytes, data: bytes) -> bytes:
    length = struct.pack(">I", len(data))
    crc = zlib.crc32(chunk_type)
    crc = zlib.crc32(data, crc)
    crc_bytes = struct.pack(">I", crc & 0xFFFFFFFF)
    return length + chunk_type + data + crc_bytes


def encode_png_rgb(width: int, height: int, rgb_bytes: bytes) -> bytes:
    """
    Encode raw RGB bytes (row-major, 3 bytes per pixel) into a PNG.
    No external deps (Pillow) required.
    """
    w = int(width)
    h = int(height)
    if w <= 0 or h <= 0:
        raise ValueError("width/height must be > 0")
    expected = w * h * 3
    if len(rgb_bytes) != expected:
        raise ValueError(f"rgb_bytes size mismatch: got {len(rgb_bytes)} expected {expected}")

    # Add filter type 0 for each scanline.
    stride = w * 3
    raw = bytearray()
    off = 0
    for _y in range(h):
        raw.append(0)
        raw.extend(rgb_bytes[off : off + stride])
        off += stride

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    idat = zlib.compress(bytes(raw), level=9)

    out = bytearray()
    out.extend(_PNG_SIG)
    out.extend(_chunk(b"IHDR", ihdr))
    out.extend(_chunk(b"IDAT", idat))
    out.extend(_chunk(b"IEND", b""))
    return bytes(out)


def make_grid_png(width: int, height: int, grid_px: int = 32) -> bytes:
    """
    Generate a simple white grid map for UI debug.
    """
    w = int(width)
    h = int(height)
    g = max(4, int(grid_px))

    rgb = bytearray(w * h * 3)
    cx = w // 2
    cy = h // 2

    for y in range(h):
        for x in range(w):
            is_axis = (x == cx) or (y == cy)
            is_grid = (x % g == 0) or (y % g == 0)

            if is_axis:
                c = 160
            elif is_grid:
                c = 220
            else:
                c = 255

            i = (y * w + x) * 3
            rgb[i + 0] = c
            rgb[i + 1] = c
            rgb[i + 2] = c

    return encode_png_rgb(w, h, bytes(rgb))

