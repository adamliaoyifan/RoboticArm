# Committed baseline snapshot (read-only)

Exact copies of production estimator sources at commit
`14c23038d0bdc0e211588d65cfb40c1cce7869a2` (task/plan revision for
PF-R6-RANSAC-RESEARCH generation 2). The dirty PF-R6 working checkpoint is
deliberately NOT reflected here; the baseline under test is the committed one.

| file | sha256 |
|---|---|
| `luggage_perception/box_geometry.py` | `d3a431189358d713bfcdc0d6201795fa7b7f8c565f04f18bd81cbec0af2c7786` |
| `luggage_perception/luggage_box_estimator.py` | `01afe27b7287c111fde0d7c99f7c64cda46ce1397698354b2f5d3bf768cf112b` |
| `luggage_perception/top_support_estimator.py` | `d7bc4749a204b0b848907d504c32611f176cbb53c77788e64eacf18117f3ec5a` |

Reproduction:

```bash
for f in top_support_estimator.py luggage_box_estimator.py box_geometry.py; do
  git show 14c23038d0bdc0e211588d65cfb40c1cce7869a2:src/luggage_perception/luggage_perception/$f
done
```

These files are imported read-only by the benchmark harness. No comparator
edits them; the baseline comparator `methods.baseline_ransac_plane` is the
verbatim committed `_ransac_horizontal_plane` re-exported from this snapshot.
