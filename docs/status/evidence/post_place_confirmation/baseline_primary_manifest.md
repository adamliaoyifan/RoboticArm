# Primary worktree preservation manifest

- captured_at: `2026-09-29T12:06:21+08:00`
- revision: `763361d339d2a8df11363d1d1f76118c1b6183b9`
- tracked_dirty_files: `53`
- tracked_binary_diff_sha256: `205910f5b0df1e7a0deb01d6363abd0def3103212407776fad91f2c2b7a630f8`
- untracked_files: `56`
- sorted_untracked_manifest_sha256: `10f11e4a160a35c61c2f1c03352cd743b3bdaedfd697b44eae0ce688374ab1b5`
- isolation: product implementation must occur in a separate worktree based on
  the revision above; these pre-existing primary-worktree changes are not to be
  staged, modified, or incorporated implicitly.

The hashes were computed with `git diff --binary | sha256sum` and
`git ls-files --others --exclude-standard | sha256sum`; the file lists remain
available from the preserved primary worktree rather than duplicated into a
large evidence dump.
