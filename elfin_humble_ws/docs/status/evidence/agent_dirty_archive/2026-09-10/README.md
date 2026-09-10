# Agent dirty-work preservation snapshot

This directory preserves tracked diffs that were still uncommitted when the
workspace moved to `ros2_humble` as its future baseline.

## Bases

- WIP branch base: `e25c7c908270ca2e3ef95456e4aedcf42a2edd46`
- Primary workspace patch base: `7076891561f5e88c1b52a184e6678d156e15fc22`
- LRF-P1 patch base: `5c08312192f65a1bd75a301c927b2a557faa0ab4`
- TCIG-7 post-close patch base: `c499e818d754e845e3840edc45a03a3b421da496`

The primary workspace's untracked source, documents, and derived evidence are
also present at their normal paths on this WIP branch. Runtime lease files were
excluded.

## Patches

- `patches/primary-master.patch`: PF-R10 g3 diagnostics/refinement and shared
  coordination changes still tracked as dirty against local `master`.
- `patches/lrf-p1.patch`: residual clipping, locked-pose fallback, regression
  test, and research stop note.
- `patches/tcig7-post-close.patch`: executable-bit corrections for the two
  ROS 2 atlas command wrappers.
- `patches/df9c7a2-pfr10-refine.patch`: format-patch for the PF-R10 commit
  created on local `master` while this archive was being assembled.
- `patches/primary-after-df9c7a2.patch`: the remaining primary-workspace diff
  after `df9c7a2`, captured after the first WIP snapshot was pushed.

Apply a patch only after checking its recorded base and intended destination;
the primary patch overlaps files subsequently changed in `ros2_humble` and
must be merged semantically rather than applied blindly.

## External raw bag

The following raw site bag is intentionally not committed because it exceeds
GitHub's normal single-file limit by two orders of magnitude:

- path: `docs/hardware/record_site_20260908_210234/record_site_20260908_210234_0.mcap`
- size: `10600611212` bytes
- SHA-256: `512a76bd07d7aa289c63c2ca9ffbd6621f963781c0bc170db69881541d2af416`
- modified: `2026-09-09 09:46:58.837791837 +0800`

Its small `metadata.yaml` is included. Move the MCAP to durable external or
LFS/object storage before any command that removes untracked files.

## Verification

- All three patches pass `git apply --reverse --check` in their source
  worktrees.
- `df9c7a2-pfr10-refine.patch` passes `git apply --reverse --check` at
  `master@df9c7a2`; `primary-after-df9c7a2.patch` passes the same check
  against the post-commit dirty workspace.
- Primary PF-R10/pendant focused suite: 110 passed, 1 skipped.
- LRF-P1 OOD suite: 9 passed.
- No Gazebo stack and no robot motion were started.
