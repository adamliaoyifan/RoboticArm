"""Bounded six-joint controller reference/feedback evidence, without commands."""
import gzip,json,time


def state_record(message):
    def point(value):
        return {name:list(getattr(value,name)) for name in ('positions','velocities','accelerations','effort')}
    reference=message.reference if message.reference.positions else message.desired
    feedback=message.feedback if message.feedback.positions else message.actual
    return dict(stamp_s=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
        joint_names=list(message.joint_names),reference=point(reference),
        feedback=point(feedback),error=point(message.error),output=point(message.output))


class ControllerTrace:
    def __init__(self,path):
        self._stream=gzip.open(path,'wt',compresslevel=3)
        self.count=0;self.latest=None
    def observe(self,message,topic=None):
        self.latest=dict(state_record(message),topic=topic)
        self._stream.write(json.dumps(self.latest)+'\n');self._stream.flush();self.count+=1
    def close(self):self._stream.close()


def record_command(runtime,event,**fields):
    native=runtime.snapshot().get('vacuum') or {}
    record=dict(event=event,wall_time_s=time.time(),native_stamp_s=native.get('sim_time_s'),**fields)
    with (runtime.output/'trajectory_commands.jsonl').open('a') as stream:
        stream.write(json.dumps(record,allow_nan=False)+'\n')


def check_tracking(runtime,threshold_rad,max_age_s):
    state=runtime.controller_trace.latest if runtime.controller_trace else None
    native=runtime.snapshot().get('vacuum') or {}
    if state is None or abs(native.get('sim_time_s',0)-state['stamp_s'])>max_age_s:
        (runtime.output/'controller_feedback_failure.json').write_text(json.dumps(dict(
            max_age_s=max_age_s,native_stamp_s=native.get('sim_time_s'),state=state),indent=2)+'\n')
        raise RuntimeError('CONTROLLER_FEEDBACK_NOT_CURRENT')
    names=state['joint_names'];reference=state['reference']['positions'];feedback=state['feedback']['positions']
    if len(names)!=6 or len(reference)!=6 or len(feedback)!=6:
        raise RuntimeError('CONTROLLER_JOINT_FEEDBACK_INCOMPLETE')
    error=max(abs(a-b) for a,b in zip(reference,feedback))
    if error>threshold_rad:
        (runtime.output/'controller_tracking_failure.json').write_text(json.dumps(dict(
            threshold_rad=threshold_rad,max_error_rad=error,state=state),indent=2)+'\n')
        raise RuntimeError('CONTROLLER_TRACKING_ERROR')
