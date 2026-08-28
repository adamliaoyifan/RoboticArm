# vendor/ — offline CLIP for YOLO-World semantic backends

This folder is **not committed** — it is populated by
[`scripts/setup_clip_vendor.sh`](../scripts/setup_clip_vendor.sh). Run that
script once on a host with network access; the folder then travels with the
workspace into any container that mounts it, so `semantic_backend:=yolo_world`
works offline without ultralytics' slow `pip install git+.../CLIP.git`
auto-install.

```
vendor/
  clip_pkg/                  ultralytics/CLIP python package  (import clip)
  deps/                      ftfy, regex, wcwidth             (CLIP runtime deps)
  clip_models/ViT-B-32.pt    ~338 MB CLIP ViT-B/32 checkpoint (clip.load target)
```

`semantic_segmenter._setup_clip_vendor()` (called from each YOLO-World backend
before `set_classes`) prepends `clip_pkg/` and `deps/` to `sys.path` and
symlinks `clip_models/ViT-B-32.pt` into `~/.cache/clip/` so `clip.load`
skips its download. Override the folder location with
`$LUGGAGE_CLIP_VENDOR_DIR` (e.g. when the workspace is mounted at a different
path inside a container). When the folder is absent the segmenter falls back
to ultralytics' default auto-install behavior.

Regenerate anytime: `bash scripts/setup_clip_vendor.sh`
