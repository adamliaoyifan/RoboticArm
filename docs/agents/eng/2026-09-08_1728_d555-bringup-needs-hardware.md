# 2026-09-08 -- D555 HB-1/2/3 need live hardware

- role: eng
- agent: cursor
- model: grok-4.6
- cli: cursor
- status: done

## Summary

HB-1, HB-2, and HB-3 cannot close from notes or sim. All three require the
installed D555 PoE online. HB-2 also needs a high-contrast target at three
known depths. HB-3 also needs Mid-360, three static arm poses, and an
operator. This host is on `192.168.2.125/24` with no route to the cell
(`192.168.11.55` D555, `192.168.1.120` Mid-360); both pings dropped, no ROS
graph, Humble only. The 2026-09-02 `NOTES.md` snapshot is not a substitute
for live `camera_info`, `/extrinsics/depth_to_color`, QoS, `topic hz`, or a
Humble-versus-Jazzy DDS answer. Item 9 of HB-1 is already given by the
operator. Rows left unclaimed so a later live session can start them.

## Pointers

- `docs/plans/d555_hardware_bringup_verification.md`
- `docs/agents/discuss/2026-09-08_1714_d555-bringup-hb-1.md`
- `docs/agents/discuss/2026-09-08_1714_d555-bringup-hb-2.md`
- `docs/agents/discuss/2026-09-08_1714_d555-bringup-hb-3.md`
- `src/luggage_description/config/backups/20260902_183500_eef_livox_d555/NOTES.md`
