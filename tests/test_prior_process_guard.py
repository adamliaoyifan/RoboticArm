from pathlib import Path
import importlib.util

path=Path(__file__).parents[1]/'scripts/placement_prior/process_guard.py'
spec=importlib.util.spec_from_file_location('prior_process_guard',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_guard_includes_orphans_and_ignores_other_domains_and_ancestors(tmp_path):
    for pid,parent,domain,command in [
        (99,98,'7','python rehearsal.py'),(98,1,'7','bash'),
        (20,1,'7','robot_state_publisher'),(21,1,'7','ros2 bag record'),
        (22,1,'0','move_group'),(23,1,'78','test probe')]:
        entry=tmp_path/str(pid);entry.mkdir()
        (entry/'status').write_text('PPid:\t%d\n'%parent)
        (entry/'environ').write_bytes(('ROS_DOMAIN_ID='+domain).encode()+b'\0')
        (entry/'cmdline').write_text(command)
    rows=module.dedicated_domain_processes(proc_root=tmp_path,current_pid=99)
    assert [row['pid'] for row in rows]==[20,21]
