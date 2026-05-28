"""
Elfin S20 Robot Arm – Forward Kinematics Visualizer
Uses Plotly for a fully interactive 3D view (saved to elfin_s20_visualization.html).
Renders joint frames, links, joint spheres, and the EOF camera mount.
"""

import numpy as np
import plotly.graph_objects as go
import sys

PI = np.pi


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def rpy_to_rot(r: float, p: float, y: float) -> np.ndarray:
    """ZYX intrinsic (URDF convention: apply R_z * R_y * R_x)."""
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r),  np.cos(r)]])
    Ry = np.array([[ np.cos(p), 0, np.sin(p)],
                   [0,          1, 0         ],
                   [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y),  np.cos(y), 0],
                   [0,          0,         1]])
    return Rz @ Ry @ Rx


def rot_axis_angle(axis: np.ndarray, theta: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    c, s = np.cos(theta), np.sin(theta)
    ux, uy, uz = axis
    return np.array([
        [c + ux*ux*(1-c),    ux*uy*(1-c) - uz*s,  ux*uz*(1-c) + uy*s],
        [uy*ux*(1-c) + uz*s, c + uy*uy*(1-c),     uy*uz*(1-c) - ux*s],
        [uz*ux*(1-c) - uy*s, uz*uy*(1-c) + ux*s,  c + uz*uz*(1-c)   ],
    ])


def make_transform(xyz, rpy, axis, theta) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_to_rot(*rpy) @ rot_axis_angle(np.array(axis, dtype=float), theta)
    T[:3, 3] = xyz
    return T


def make_fixed_transform(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_to_rot(*rpy)
    T[:3, 3] = xyz
    return T


# ---------------------------------------------------------------------------
# Elfin S20 – joint chain from S20.urdf.xacro
# ---------------------------------------------------------------------------

JOINTS = [
    ("world_base",   "fixed",    [0, 0, 0],          [0, 0, 0],           None),
    ("elfin_base_j", "fixed",    [0, 0, 0],          [0, 0, 0],           None),
    ("joint1",  "revolute", [0,       0,       0.171  ], [0,       0,      0   ], [0,0,1]),
    ("joint2",  "revolute", [0,      -0.2295,  0      ], [PI/2,    0,      0   ], [0,0,1]),
    ("joint3",  "revolute", [-0.85,   0,      -0.1885 ], [0,       0,      PI  ], [0,0,1]),
    ("joint4",  "revolute", [0.712,   0,       0      ], [0,       0,      0   ], [0,0,1]),
    ("joint5",  "revolute", [0,       0,       0.138  ], [-PI/2,   0,      0   ], [0,0,1]),
    ("joint6",  "revolute", [0,       0,       0.138  ], [PI/2,    0,      0   ], [0,0,1]),
    ("end_joint","fixed",   [0,       0,       0.1257 ], [0,       0,      PI  ], None),
]

LINK_NAMES = ["World", "Base", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6", "EOF"]


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------

def forward_kinematics(joint_angles):
    q = iter(joint_angles)
    T = np.eye(4)
    frames = [T.copy()]
    for _, jtype, xyz, rpy, axis in JOINTS:
        if jtype == "fixed":
            T = T @ make_fixed_transform(xyz, rpy)
        else:
            T = T @ make_transform(xyz, rpy, axis, next(q))
        frames.append(T.copy())
    return frames


def positions(frames):
    return np.array([f[:3, 3] for f in frames])


# ---------------------------------------------------------------------------
# Plotly geometry helpers
# ---------------------------------------------------------------------------

def tube_mesh(p1, p2, radius=0.03, n=16):
    """Return (x, y, z, i, j, k) for a cylindrical tube mesh between p1 and p2."""
    vec = p2 - p1
    L = np.linalg.norm(vec)
    if L < 1e-6:
        return None
    z = vec / L
    ref = np.array([0, 0, 1]) if abs(z[2]) < 0.9 else np.array([1, 0, 0])
    x = np.cross(ref, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)

    angles = np.linspace(0, 2*PI, n, endpoint=False)
    cos_a = np.cos(angles); sin_a = np.sin(angles)
    ring = np.outer(cos_a, x) + np.outer(sin_a, y)  # (n, 3)

    verts = np.vstack([p1 + ring * radius,   # indices 0..n-1  (bottom ring)
                       p2 + ring * radius,   # indices n..2n-1 (top ring)
                       [p1], [p2]])          # indices 2n, 2n+1 (caps)

    tri_i, tri_j, tri_k = [], [], []
    for i in range(n):
        j = (i + 1) % n
        # Side quad → 2 triangles
        tri_i += [i, i]; tri_j += [j, n+i]; tri_k += [n+i, n+j]
        tri_i += [i]; tri_j += [n+j]; tri_k += [n+i]
        # Bottom cap
        tri_i.append(2*n);     tri_j.append(i);   tri_k.append(j)
        # Top cap
        tri_i.append(2*n+1);   tri_j.append(n+i); tri_k.append(n+j)

    return (verts[:, 0], verts[:, 1], verts[:, 2],
            tri_i, tri_j, tri_k)


def arrow_traces(origin, R, scale=0.12, opacity=0.9):
    """RGB XYZ frame arrows as three Scatter3d traces."""
    colors = ["red", "limegreen", "dodgerblue"]
    labels = ["X", "Y", "Z"]
    traces = []
    for i, (c, lbl) in enumerate(zip(colors, labels)):
        tip = origin + R[:3, i] * scale
        traces.append(go.Scatter3d(
            x=[origin[0], tip[0]], y=[origin[1], tip[1]], z=[origin[2], tip[2]],
            mode="lines+text",
            line=dict(color=c, width=5),
            text=["", lbl], textfont=dict(color=c, size=10),
            opacity=opacity,
            showlegend=False,
            hoverinfo="skip",
        ))
    return traces


def sphere_mesh(center, radius=0.045, n_lat=12, n_lon=20):
    lat = np.linspace(-PI/2, PI/2, n_lat)
    lon = np.linspace(0, 2*PI, n_lon)
    la, lo = np.meshgrid(lat, lon)
    x = center[0] + radius * np.cos(la) * np.cos(lo)
    y = center[1] + radius * np.cos(la) * np.sin(lo)
    z = center[2] + radius * np.sin(la)
    return x, y, z


def box_mesh(center, R, dims):
    """8-corner box at `center` oriented by R with half-dims `dims`."""
    w, h, d = dims
    corners_local = np.array([
        [-w, -h, -d], [w, -h, -d], [w, h, -d], [-w, h, -d],
        [-w, -h,  d], [w, -h,  d], [w, h,  d], [-w, h,  d],
    ]) * 0.5
    corners = center + (R @ corners_local.T).T

    def f(a, b, c, d_):
        return ([corners[a, 0], corners[b, 0], corners[c, 0], corners[d_, 0], corners[a, 0]],
                [corners[a, 1], corners[b, 1], corners[c, 1], corners[d_, 1], corners[a, 1]],
                [corners[a, 2], corners[b, 2], corners[c, 2], corners[d_, 2], corners[a, 2]])

    faces = [f(0,1,2,3), f(4,5,6,7), f(0,1,5,4),
             f(2,3,7,6), f(0,3,7,4), f(1,2,6,5)]
    traces = []
    for face_x, face_y, face_z in faces:
        traces.append(go.Scatter3d(
            x=face_x, y=face_y, z=face_z,
            mode="lines",
            line=dict(color="#1a252f", width=2),
            showlegend=False,
            hoverinfo="skip",
        ))
    return traces


# ---------------------------------------------------------------------------
# Camera mount geometry
# ---------------------------------------------------------------------------

def camera_mount_traces(T_eof, mount_z=0.05):
    traces = []
    origin = T_eof[:3, 3]
    R = T_eof[:3, :3]
    z_axis = R[:, 2]

    cam_pos = origin + z_axis * mount_z

    # Bracket tube
    t = tube_mesh(origin, cam_pos, radius=0.012, n=12)
    if t:
        vx, vy, vz, ti, tj, tk = t
        traces.append(go.Mesh3d(
            x=vx, y=vy, z=vz, i=ti, j=tj, k=tk,
            color="#3498db", opacity=0.9, flatshading=False,
            name="Camera bracket", showlegend=True,
            hovertemplate="Camera bracket",
        ))

    # Camera body (RealSense D435 – 90×25×25 mm)
    cam_dims = (0.090, 0.025, 0.025)
    traces += box_mesh(cam_pos, R, cam_dims)

    # Filled camera face with Mesh3d
    w, h, d = [x/2 for x in cam_dims]
    body_corners_local = np.array([
        [-w, -h, -d], [ w, -h, -d], [ w, h, -d], [-w, h, -d],
        [-w, -h,  d], [ w, -h,  d], [ w, h,  d], [-w, h,  d],
    ])
    bc = cam_pos + (R @ body_corners_local.T).T
    traces.append(go.Mesh3d(
        x=bc[:, 0], y=bc[:, 1], z=bc[:, 2],
        i=[0,0,4,4,0,0,2,2,0,0,1,1],
        j=[1,2,5,6,1,4,3,7,3,4,2,5],
        k=[2,3,6,7,4,5,7,6,7,3,6,6],
        color="#2c3e50", opacity=0.88, flatshading=True,
        name="Camera body", showlegend=True,
        hovertemplate="Intel RealSense D435",
    ))

    # Lens circles
    for lx in [-0.028, 0, 0.028]:
        t_a = np.linspace(0, 2*PI, 32)
        lens_r = 0.008
        pts_lens = np.array([
            cam_pos + R @ np.array([lx + lens_r*np.cos(a),
                                    lens_r*np.sin(a), d+0.001])
            for a in t_a
        ])
        traces.append(go.Scatter3d(
            x=pts_lens[:, 0], y=pts_lens[:, 1], z=pts_lens[:, 2],
            mode="lines",
            line=dict(color="#00aaff", width=3),
            showlegend=False, hoverinfo="skip",
        ))

    return traces, cam_pos


# ---------------------------------------------------------------------------
# Link radii (visual, scaled to robot size)
# ---------------------------------------------------------------------------

LINK_RADII = [0.0, 0.0, 0.055, 0.07, 0.055, 0.045, 0.035, 0.030, 0.020]
LINK_COLORS_HEX = [
    "#888888", "#6c6c6c", "#d0d0d0", "#a0a0a0",
    "#d0d0d0", "#a0a0a0", "#d0d0d0", "#a0a0a0", "#e8a020"
]


# ---------------------------------------------------------------------------
# Main visualize function
# ---------------------------------------------------------------------------

def visualize(joint_angles=None, show_mount=True, output="elfin_s20_visualization.html"):
    if joint_angles is None:
        joint_angles = [0.0] * 6

    frames = forward_kinematics(joint_angles)
    pts = positions(frames)

    traces = []

    # ── 1. Link tubes
    for i in range(2, len(pts)):
        p1, p2 = pts[i-1], pts[i]
        r = LINK_RADII[i-1] if i-1 < len(LINK_RADII) else 0.02
        if np.linalg.norm(p2 - p1) < 1e-4:
            continue
        t = tube_mesh(p1, p2, radius=r, n=20)
        if t:
            vx, vy, vz, ti, tj, tk = t
            traces.append(go.Mesh3d(
                x=vx, y=vy, z=vz, i=ti, j=tj, k=tk,
                color=LINK_COLORS_HEX[i-1], opacity=0.85, flatshading=False,
                name=f"Link {i-2}",
                showlegend=(i == 3),
                legendgroup="links",
                hovertemplate=f"Link {i-2}",
            ))

    # ── 2. Joint spheres
    joint_frame_idx = [2, 3, 4, 5, 6, 7]
    for k, idx in enumerate(joint_frame_idx):
        sx, sy, sz = sphere_mesh(pts[idx], radius=0.048)
        traces.append(go.Surface(
            x=sx, y=sy, z=sz,
            colorscale=[[0, "#c0392b"], [1, "#e74c3c"]],
            showscale=False, opacity=0.95,
            name=f"Joint {k+1}",
            showlegend=(k == 0),
            legendgroup="joints",
            hovertemplate=f"Joint {k+1}<br>pos: ({pts[idx,0]:.3f}, {pts[idx,1]:.3f}, {pts[idx,2]:.3f})",
        ))

    # ── 3. Coordinate frames at Base, J3, J6, EOF
    for idx in [2, 4, 7, 8]:
        traces += arrow_traces(pts[idx], frames[idx], scale=0.10)

    # ── 4. Camera mount
    cam_pos = None
    if show_mount:
        mt, cam_pos = camera_mount_traces(frames[-1], mount_z=0.055)
        traces += mt

    # ── 5. Ground plane ring
    ring_t = np.linspace(0, 2*PI, 80)
    base_z = pts[2, 2]
    traces.append(go.Scatter3d(
        x=0.18 * np.cos(ring_t), y=0.18 * np.sin(ring_t),
        z=np.full(80, base_z),
        mode="lines",
        line=dict(color="#7777aa", width=4),
        name="Base plate", showlegend=True,
        hoverinfo="skip",
    ))

    # ── Layout
    all_pts = pts if cam_pos is None else np.vstack([pts, cam_pos])
    pad = 0.20
    rng = [[all_pts[:, i].min() - pad, all_pts[:, i].max() + pad] for i in range(3)]

    # Build annotation table
    eof_pos = frames[-1][:3, 3]
    q_deg = [f"{np.degrees(a):.1f}°" for a in joint_angles]
    annot_text = (
        "<b>Elfin S20  –  Elfin S20 FK</b><br>"
        + "".join(f"  J{k+1}: {q_deg[k]}<br>" for k in range(6))
        + f"<br><b>EOF position (m)</b><br>"
        + f"  X={eof_pos[0]:.4f}  Y={eof_pos[1]:.4f}  Z={eof_pos[2]:.4f}<br>"
        + "<br><b>Camera mount</b><br>"
        + "  Bracket offset: 55 mm along EOF-Z<br>"
        + "  Camera: Intel RealSense D435<br>"
        + "  Flange: 4× M6 on Ø63 mm PCD"
    )

    fig = go.Figure(data=traces)
    fig.update_layout(
        title=dict(
            text="<b>Elfin S20 Robot  ·  Camera Mount at EOF</b>",
            font=dict(size=18, color="white"),
            x=0.5,
        ),
        scene=dict(
            xaxis=dict(title="X (m)", range=rng[0],
                       backgroundcolor="#16213e", gridcolor="#2a2a5a",
                       color="white", showbackground=True),
            yaxis=dict(title="Y (m)", range=rng[1],
                       backgroundcolor="#16213e", gridcolor="#2a2a5a",
                       color="white", showbackground=True),
            zaxis=dict(title="Z (m)", range=rng[2],
                       backgroundcolor="#1a1a3e", gridcolor="#2a2a5a",
                       color="white", showbackground=True),
            bgcolor="#0d0d1f",
            camera=dict(eye=dict(x=1.4, y=-1.4, z=0.8)),
            aspectmode="cube",
        ),
        paper_bgcolor="#0d0d1f",
        plot_bgcolor="#0d0d1f",
        font=dict(color="white"),
        legend=dict(
            bgcolor="#1a1a3a", bordercolor="#334466", borderwidth=1,
            font=dict(color="white"),
        ),
        annotations=[dict(
            x=0.01, y=0.99, xref="paper", yref="paper",
            text=annot_text, showarrow=False, align="left",
            font=dict(size=10, color="#aaccff"),
            bgcolor="#1a2a40", bordercolor="#334466", borderpad=6,
        )],
        margin=dict(l=0, r=0, t=50, b=0),
        width=1200, height=800,
    )

    fig.write_html(output)
    print(f"Saved: {output}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    poses = {
        "home":    [0, 0, 0, 0, 0, 0],
        "reach":   [0, -PI/4, PI/4, 0, PI/3, 0],
        "folded":  [0, -PI/2, PI/2, 0, -PI/2, 0],
    }

    pose_name = sys.argv[1] if len(sys.argv) > 1 else "home"
    q = poses.get(pose_name, poses["home"])
    print(f"Pose: {pose_name}  →  {[f'{np.degrees(a):.0f}°' for a in q]}")
    visualize(q, show_mount=True)
