# LRF-P1 evidence

- subtask: LRF-P1
- recommendation: continue-research
- tests: `python3 -m unittest discover -s research/lrf_p1/tests -p 'test_*.py'` -- 13 passed
- spike: `python3 research/lrf_p1/run_spike.py --tests-result pass`
- metrics.json sha256: `5203df94be196b2d8009934fe52c8bdb28b8fb7144c9e5d05c5050931f9be2ff`
- dirty files at this folder: recorded after the evidence commit; expect 0 on a clean checkout of that commit

## Files

- `metrics.json` (aggregates, seed summaries, per-sample seed-1 rows, gates)
- `manifest.json`
- `FAILURE_GALLERY.md`
- `COMMANDS.md`
- `research/lrf_p1/REPORT.md`
- `research/lrf_p1/METHOD.md`
