"""Read-only dedicated-domain guard, including orphan capture processes."""
import os
from pathlib import Path


def dedicated_domain_processes(domain='7', *, proc_root=Path('/proc'), current_pid=None):
    current_pid=os.getpid() if current_pid is None else current_pid
    ancestors=set();pid=current_pid
    while pid and pid not in ancestors:
        ancestors.add(pid)
        try:
            status=(proc_root/str(pid)/'status').read_text()
            pid=int(next(line.split()[1] for line in status.splitlines() if line.startswith('PPid:')))
        except (OSError,StopIteration,ValueError):break
    rows=[]
    expected=('ROS_DOMAIN_ID='+str(domain)).encode()
    for entry in proc_root.iterdir():
        if not entry.name.isdigit() or int(entry.name) in ancestors:continue
        try:
            if expected not in (entry/'environ').read_bytes().split(b'\0'):continue
            command=(entry/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
            if command:rows.append(dict(pid=int(entry.name),command=command))
        except (OSError,ProcessLookupError):continue
    return sorted(rows,key=lambda row:row['pid'])
