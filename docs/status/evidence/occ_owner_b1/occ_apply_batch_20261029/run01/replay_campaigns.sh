#!/usr/bin/env bash
set -euo pipefail
root=/home/adamliao/work/elfin_humble_ws
product=/tmp/elfin-occ-octomap-perf-cursor-g1
evidence="$root/docs/status/evidence/occ_owner_b1/occ_apply_batch_20261029/run01"
runner="$evidence/harness/fusion_benchmark.py"
fixtures="$root/docs/status/evidence/occ_owner_b1/occ_octomap_perf_diag_g1_20261026/run01/fixtures"
common=(--fixture-root "$fixtures" --geometry "$evidence/config/geometry.yaml" --config "$evidence/config/config.yaml" --source-root "$product" --timeout-s 900)
run_campaign() {
  local label="$1"; shift
  local remaining
  remaining=$(python3 - "$evidence" <<'PY'
import json, pathlib, sys
root=pathlib.Path(sys.argv[1])
used=sum(json.loads(p.read_text()).get('measurement_wall_s',0) for name in ('baseline_to_a','a_to_b','b_stages','memory') for p in (root/name).glob('campaign_report.json'))
if sum(p.stat().st_size for p in root.rglob('*') if p.is_file()) >= 8*1024**3:
    raise SystemExit('evidence cap exceeded before campaign')
print(max(0, int(10800-used)))
PY
)
  python3 "$runner" "${common[@]}" --budget-s "$remaining" --out "$evidence/$label" "$@" > "$evidence/${label}_console.log" 2>&1
  python3 - "$evidence/$label/campaign_report.json" <<'PY'
import json,sys
r=json.load(open(sys.argv[1]));print(r['campaign'],r['status'],r['measurement_wall_s'],flush=True)
if r['status'] != 'complete': raise SystemExit('campaign incomplete; preserve evidence before continuing')
PY
}
run_campaign baseline_to_a --campaign baseline --repetitions 3 --require-content --baseline-cli /tmp/occ-apply-baseline-build/observer_profile_cli --candidate-cli /tmp/occ-apply-a-frozen-build/observer_profile_cli
run_campaign a_to_b --campaign baseline --repetitions 3 --require-content --baseline-cli /tmp/occ-apply-a-frozen-build/observer_profile_cli --candidate-cli /tmp/occ-apply-b-frozen-build/observer_profile_cli
run_campaign b_stages --campaign stages --repetitions 1 --require-content --baseline-cli /tmp/occ-apply-b-frozen-build/observer_profile_cli --candidate-cli /tmp/occ-apply-b-frozen-build/observer_profile_cli
run_campaign memory --campaign memory --repetitions 1 --candidate-cli /tmp/occ-apply-b-frozen-build/observer_profile_cli
