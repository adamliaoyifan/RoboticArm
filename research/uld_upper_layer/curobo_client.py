"""One private, bounded GPU worker per source scene; no changes to ROS Python.

Responses use a dedicated pipe because Warp/cuRobo write startup text to stdout.
Timeout closes the worker, so a late response can never satisfy a later query.
Linux parent-death signalling also covers an enclosing planning-worker kill.
"""
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import tempfile
import time
import weakref


class CuroboClient:
    def __init__(self,python=''):
        self.directory=tempfile.TemporaryDirectory(prefix='upper-curobo-')
        self.log_path=Path(self.directory.name)/'worker.log'
        read_fd,write_fd=os.pipe()
        try:
            with self.log_path.open('wb') as log:
                self.process=subprocess.Popen([python or sys.executable,str(Path(__file__).with_name('curobo_worker.py')),
                    '--response-fd',str(write_fd),'--parent-pid',str(os.getpid())],
                    stdin=subprocess.PIPE,stdout=log,stderr=log,pass_fds=(write_fd,),text=True,bufsize=1)
        except BaseException:
            os.close(read_fd);self.directory.cleanup();raise
        finally:os.close(write_fd)
        self.response=os.fdopen(read_fd,'rb',buffering=0)
        # Do not retain self through atexit; source scene disposal releases CUDA.
        self.finalizer=weakref.finalize(self,self._cleanup,self.process,self.response,self.directory)

    @staticmethod
    def _cleanup(process,response,directory):
        if process.poll() is None:
            process.kill()
        process.wait();response.close()
        if process.stdin:process.stdin.close()
        directory.cleanup()

    def close(self):self.finalizer()

    def query(self,request,timeout):
        try:
            self.process.stdin.write(json.dumps(request,allow_nan=False)+'\n');self.process.stdin.flush()
            deadline=time.monotonic()+max(0.,timeout);buffer=bytearray()
            while b'\n' not in buffer:
                ready,_,_=select.select([self.response],[],[],max(0.,deadline-time.monotonic()))
                if not ready:raise TimeoutError('GPU_WORKER_TIMEOUT')
                chunk=os.read(self.response.fileno(),65536)
                if not chunk:raise RuntimeError('GPU_WORKER_EXITED')
                buffer.extend(chunk)
                if len(buffer)>32*1024*1024:raise ValueError('GPU_RESPONSE_TOO_LARGE')
            line=buffer.split(b'\n',1)[0]
            result=json.loads(line)
            if result.get('identity')!=request['identity']:raise ValueError('GPU_RESPONSE_IDENTITY_MISMATCH')
            if not result.get('success'):
                result['worker_log_tail']=self.log_tail()
            return result
        except (OSError,ValueError,RuntimeError,TimeoutError) as exc:
            # Preserve the bounded failure tail before releasing the temporary log.
            tail=self.log_tail()
            self.close()
            return dict(success=False,reason=str(exc),worker_log_tail=tail,identity=request['identity'])

    def log_tail(self):
        with self.log_path.open('rb') as log:
            log.seek(max(0,self.log_path.stat().st_size-12000))
            return log.read().decode(errors='replace')
