# PF-R5B rerun-boundary mechanical check (c5921d5 -> a3dba5e)

## 1. mesh_observable_reference core computation
```
+#: ``mesh_observable_reference`` semantics (band, bin size, plateau rule)
+    "mesh_observable_reference/v1(top_band_frac=0.25,z_bin=0.001,"
+    "lid=densest-bin-median,extent=plateau_band)")
```

## 2. evaluator source
```
(empty = zero diff)
```

## 3. online detection path
```
(detector/filter TF-wedge fix was in c5921d5 itself; only empty = zero diff since)
```

## 4. six pinned values (test-anchored)
```
10 passed in 0.09s
```
