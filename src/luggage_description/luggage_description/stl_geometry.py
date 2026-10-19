"""Shared ROS-free STL decoding for scene publication and boundary queries."""

def _stl_is_binary(data):
    """Binary STL is 84 + n*50 bytes. ASCII may start with ``solid``."""
    import struct
    if len(data) < 84:
        return False
    n_tri = struct.unpack_from("<I", data, 80)[0]
    if n_tri <= 0:
        return False
    expected = 84 + n_tri * 50
    if len(data) == expected:
        return True
    head = data.lstrip()[:5].lower()
    if head == b"solid" and b"facet" in data[:2048].lower():
        return False
    return expected <= len(data) < expected + 256


def read_stl_arrays(path):
    """Vertices (N,3) and triangle indices (M,3) from a binary or ASCII STL."""
    with open(path, "rb") as handle:
        data = handle.read()
    return stl_arrays_from_bytes(data, path)


def stl_arrays_from_bytes(data, path="<bytes>"):
    """Decode previously captured bytes without reopening a mutable resource."""
    import struct
    import numpy as np
    if len(data) < 84:
        raise ValueError("STL %s is too small (%d bytes)" % (path, len(data)))
    if _stl_is_binary(data):
        n_tri = struct.unpack_from("<I", data, 80)[0]
        dtype = np.dtype([
            ("normal", "<f4", (3,)),
            ("v0", "<f4", (3,)),
            ("v1", "<f4", (3,)),
            ("v2", "<f4", (3,)),
            ("attr", "<u2"),
        ])
        records = np.frombuffer(data, dtype=dtype, count=int(n_tri), offset=84)
        vertices = np.empty((int(n_tri) * 3, 3), dtype=np.float64)
        vertices[0::3] = records["v0"]
        vertices[1::3] = records["v1"]
        vertices[2::3] = records["v2"]
        faces = np.arange(int(n_tri) * 3, dtype=np.int64).reshape(int(n_tri), 3)
        return vertices, faces
    text = data.decode("utf-8", "replace")
    vertices = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[0].lower() == "vertex":
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
    if len(vertices) < 3 or len(vertices) % 3:
        raise ValueError(
            "STL %s is neither a binary mesh nor an ASCII facet list" % path)
    n_tri = len(vertices) // 3
    return (
        np.asarray(vertices, dtype=np.float64),
        np.arange(n_tri * 3, dtype=np.int64).reshape(n_tri, 3),
    )


