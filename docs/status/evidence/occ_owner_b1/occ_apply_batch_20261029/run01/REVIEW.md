# Read-only implementation review

Independent helper review of frozen A/B: no confirmed normal-path product defect. Unknown/zero distinction, coarse-leaf updates, official float integration, parent maintenance/pruning and Off diagnostics consistent.

Focused ApplyBatch regression currently seeds all sibling keys with hits and the extra free key as known zero. Existing keyset reference test exercises first-frame unknown free rays, but the new focused class should explicitly exercise an unknown miss itself. Add that focused test after timing, without modifying measured product code; record a separate test-only SHA.

Allocation failures during apply are non-transactional: partial tree writes and stale blocks/identity may exist, and lazy parents may be unfinished. Neither this task nor existing behavior provides recover-and-continue after allocation failure. No allocation-failure recovery success is claimed; document reset/restore requirement. Completed scans and endpoint-validation failures remain separately tested before mutation.

Closeout completed:4c8f1862 adds the explicit unknown-miss/hit flags and documents required reset/restore after failed apply. Final CTest4/4 and pytest45/45. Runtime code unchanged after B freeze.
