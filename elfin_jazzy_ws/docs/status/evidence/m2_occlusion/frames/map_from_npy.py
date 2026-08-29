import numpy as np

OUT = "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/m2_occlusion/frames"
dep = np.load(OUT + "/frame_00_depth.npy")
K = [337.22194822727283, 0.0, 320.0, 0.0, 337.22194822727283, 240.0, 0.0, 0.0, 1.0]
fx, cx, fy, cy = K[0], K[2], K[4], K[5]
# TF world<-camera_depth_optical_frame at pickup_observe (measured):
# pos (-0.8,0,1.7); optical +X=(1,0,0) +Y=(0,-1,0) +Z=(0,0,-1)
R = np.array([[1,0,0],[0,-1,0],[0,0,-1]], dtype=np.float64)
T = np.array([-0.8, 0.0, 1.7])
H, W = dep.shape
vv, uu = np.mgrid[0:H, 0:W]
z = dep
x = (uu - cx) / fx * z
y = (vv - cy) / fy * z
fin = np.isfinite(dep)
pts = np.stack([x, y, np.where(fin, z, 0)], axis=-1)
sel = pts[fin]
w = sel.dot(R.T) + T
zw = w[:, 2]
cls = np.full(zw.shape, '.', dtype='<U1')
cls[np.abs(zw-1.115)<0.02] = 'B'
cls[np.abs(zw-0.860)<0.02] = 'P'
cls[(zw>=1.13)&(zw<1.45)] = 'M'
cls[(zw>=1.45)&(zw<1.8)] = 'n'
cls[zw<0.5] = 'g'
cls[(zw>=0.88)&(zw<1.13)&(np.abs(zw-0.86)>=0.02)] = 's'
gx, gy = 64, 24
lines = ["M region location map (64x24 grid, matches frame_00_depth.png):",
         "B=box_top(1.115) P=platform(0.86) M=mystery(1.13-1.45, the phantom to verify)",
         "n=panel(1.45-1.8, known self-occlusion) s=box-side/other g=ground .=no-hit", ""]
img = cls.reshape(H//40, 40, W//64, 64).mean(axis=(1,3)) if False else None
# majority per block
for j in range(gy):
    row = []
    for i in range(gx):
        block = cls.reshape(H, W)[j*H//gy:(j+1)*H//gy, i*W//gx:(i+1)*W//gx].ravel() \
            if False else None
    # simpler: reindex cls into image shape via fin mask positions
# rebuild 2D map directly:
img2 = np.full((H, W), '.', dtype='<U1')
img2[fin] = cls
for j in range(gy):
    row = []
    for i in range(gx):
        block = img2[j*H//gy:(j+1)*H//gy, i*W//gx:(i+1)*W//gx].ravel()
        vals, counts = np.unique(block[block != '.'], return_counts=True)
        row.append(vals[np.argmax(counts)] if len(vals) else '.')
    lines.append("".join(row))
open(OUT + "/classification_map.txt", "w").write("\n".join(lines) + "\n")
print("\n".join(lines[-24:]))
