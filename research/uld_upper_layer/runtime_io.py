"""ROS/Gazebo adapter for the explicitly ideal-input research experiment.

Capture uses a private partition and owned launch. No observation callback moves
the robot or cargo. All raw transport traces survive failures before teardown.
"""
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import threading
import time
from collections import deque


class RuntimeIO:
    def __init__(self,repo,evidence,scene_dir=None):
        self.repo=Path(repo);self.output=Path(evidence)
        self.env=dict(os.environ,ROS_DOMAIN_ID='7',IGN_PARTITION='uld_upper_layer_closed_loop',
                      ROS_LOG_DIR=str(self.output/'ros_logs'))
        self.contact_topic='/upper_layer_payload/contacts'
        self.vacuum_spec=None;self.vacuum_capture=None;self.vacuum_state=None
        self._native_history=deque(maxlen=256)
        if scene_dir:
            self.env['ULD_UPPER_SCENE']=str(scene_dir)
            fixture=json.loads((Path(scene_dir)/'fixture.json').read_text())
            self.vacuum_spec=fixture.get('ideal_vacuum')
            self.contact_topic=fixture.get('contact_observation_topic',self.contact_topic)
        self.launch=None;self.pose_capture=None;self.contact_capture=None
        self.clock_capture=None
        self.contacts=None;self.contact_stamp=None;self.contact_count=0
        self.positive_contact_messages=0;self.complete_contact_messages=0
        self.native_contact_samples=0;self.native_empty_contact_samples=0
        self.pose_lock=threading.Lock();self.poses={};self.pose_stamps={};self.pose_stamp=None
        self.pose_count=0;self.q=None;self.q_stamp=None;self.clocks=[]
        self.first_clock=None
        self.capture_error=None

    def start(self):
        # launch_scene performs the machine-wide exclusivity check itself.
        if self.env.get('ELFIN_GZ_STRIP_PANEL_COLLISION','').strip():
            raise RuntimeError('PANEL_COLLISION_STRIPPING_INCOMPATIBLE_WITH_PHYSICAL_PUSH')
        self.launch_log=(self.output/'launch.log').open('w')
        self.launch=subprocess.Popen(['bash',str(self.repo/'research/uld_upper_layer/launch_scene.sh'),
            '--headless'],env=self.env,stdout=self.launch_log,stderr=subprocess.STDOUT,start_new_session=True)
        (self.output/'outer_launch.pid').write_text(str(self.launch.pid))
        deadline=time.monotonic()+60
        while time.monotonic()<deadline:
            log=(self.output/'launch.log').read_text(errors='replace')
            clean=re.sub(r'\x1b\[[0-9;]*m','',log)
            if 'Configured and activated elfin_arm_controller' in clean and 'You can start planning now!' in clean:break
            if self.launch.poll() is not None:raise RuntimeError('SIM_STARTUP_EXITED')
            time.sleep(.2)
        else:raise TimeoutError('SIM_STARTUP_READINESS')
        from sim_model_contract import panel_contact_identity
        spawned=Path(self.env.get('TMPDIR','/tmp'))/'elfin_s20_gazebo.sdf'
        shutil.copy2(spawned,self.output/'spawned_robot.sdf')
        parent=self.vacuum_spec['parent_model'] if self.vacuum_spec else 'S20'
        self.panel_contact=panel_contact_identity(self.output/'spawned_robot.sdf',parent)
        (self.output/'panel_contact_identity.json').write_text(json.dumps(self.panel_contact,indent=2)+'\n')
        os.environ['ROS_DOMAIN_ID']='7'
        import rclpy
        from sensor_msgs.msg import JointState
        from rclpy.qos import qos_profile_sensor_data
        rclpy.init()
        self.node=rclpy.create_node('upper_layer_ideal_experiment')
        self.joint_trace=(self.output/'joint_states.jsonl').open('w')
        def joints(msg):
            names=[f'elfin_joint{i}' for i in range(1,7)]
            values=dict(zip(msg.name,msg.position))
            if all(n in values for n in names):
                self.q=[values[n] for n in names]
                self.q_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                self.joint_trace.write(json.dumps(dict(stamp=self.q_stamp,q=self.q))+'\n')
                self.joint_trace.flush()
        self.node.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data)
        from ros_clock_capture import RosClockCapture
        self.clock_capture=RosClockCapture(self.output/'clock.jsonl.gz',self._accept_clock,self._native_stamp)
        self.clock_capture.start()
        from transport_capture import JsonTopicCapture
        self.pose_capture=JsonTopicCapture(self.env,self.output,'poses',
            '/world/airport_loading/pose/info',self._accept_poses)
        self.pose_capture.start()
        self.contact_capture=JsonTopicCapture(self.env,self.output,'payload_contacts',
            self.contact_topic,self._accept_contacts)
        self.contact_capture.start()
        if self.vacuum_spec:
            self.vacuum_capture=JsonTopicCapture(self.env,self.output,'vacuum_state',
                self.vacuum_spec['state_topic'],self._accept_vacuum)
            self.vacuum_capture.start()
        deadline=time.monotonic()+25
        while time.monotonic()<deadline:
            rclpy.spin_once(self.node,timeout_sec=.1)
            with self.pose_lock:ready='upper_layer_payload' in self.poses
            if self.q is not None and ready and len(self.clocks)>20:
                if self.clocks[-1]-self.first_clock>=5.:break
        else:raise TimeoutError('IDEAL_OBSERVATION_NOT_READY')
        count=self.node.count_publishers('/clock')
        if count!=1:raise RuntimeError(f'CLOCK_PUBLISHER_COUNT:{count}')

    def _accept_clock(self,stamp):
        with self.pose_lock:
            self.clocks.append(stamp)
            if self.first_clock is None:self.first_clock=stamp
            if len(self.clocks)>500:self.clocks=self.clocks[-500:]

    def _native_stamp(self):
        with self.pose_lock:return self.vacuum_state['sim_time_s'] if self.vacuum_state else None

    def _accept_poses(self,row):
        header=row.get('header',{}).get('stamp',{})
        stamp=float(header.get('sec',0))+float(header.get('nsec',0))*1e-9
        with self.pose_lock:
            for item in row.get('pose',[]):
                if item.get('name'):
                    self.poses[item['name']]=item;self.pose_stamps[item['name']]=stamp
            self.pose_stamp=stamp;self.pose_count+=1

    def _accept_contacts(self,row):
        header=row.get('header',{}).get('stamp',{})
        stamp=float(header.get('sec',0))+float(header.get('nsec',0))*1e-9
        with self.pose_lock:
            if not self.vacuum_spec:self.contacts=row;self.contact_stamp=stamp
            self.contact_count+=1
            contacts=row.get('contact',[])
            if contacts:self.positive_contact_messages+=1
            if contacts and all(len(c.get('normal',[]))==len(c.get('position',[]))>0
                                and len(c.get('depth',[]))==len(c['position']) for c in contacts):
                self.complete_contact_messages+=1

    def _accept_vacuum(self,row):
        state=json.loads(row['data'])
        with self.pose_lock:
            self.vacuum_state=state;self._native_history.append(state)
            if state.get('contacts_component_available') and 'payload_contacts' in state:
                self.contacts=state['payload_contacts'];self.contact_stamp=state['sim_time_s']
                self.native_contact_samples+=1
                if not self.contacts.get('contact'):self.native_empty_contact_samples+=1
            elif self.vacuum_spec:
                self.contacts=None;self.contact_stamp=None

    def native_samples_after(self,stamp_s):
        with self.pose_lock:
            return tuple(state for state in self._native_history if state['sim_time_s']>stamp_s)

    def toggle_vacuum(self,attached,timeout_s=5.,monitor=None):
        """Queue ideal attachment; wait for physics-step state acknowledgement.

        Caller verifies the ideal cup seal-frame gap and footprint before attaching.
        The rigid plate lies behind the seal frame; CAD plate contact is not a
        prerequisite of this explicitly ideal cup model. Never used
        to move a released payload; native pose-command count proves this.
        """
        if not self.vacuum_spec:raise RuntimeError('IDEAL_VACUUM_NOT_CONFIGURED')
        before=self.snapshot()['vacuum']
        previous_commands=before['processed_commands'] if before else -1
        process=subprocess.Popen(['ign','service','-s',self.vacuum_spec['service'],
            '--reqtype','ignition.msgs.Boolean','--reptype','ignition.msgs.Boolean',
            '--timeout','2000','--req','data: '+str(bool(attached)).lower()],
            env=self.env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        import rclpy
        deadline=time.monotonic()+timeout_s
        try:
            while process.poll() is None:
                rclpy.spin_once(self.node,timeout_sec=.01)
                if monitor:
                    reason=monitor(self.snapshot())
                    if reason:raise RuntimeError(reason)
                if time.monotonic()>deadline:raise TimeoutError('IDEAL_VACUUM_SERVICE_TIMEOUT')
            stdout,stderr=process.communicate()
        except BaseException:
            if process.poll() is None:process.terminate()
            process.communicate(timeout=3.)
            raise
        with (self.output/'vacuum_commands.jsonl').open('a') as trace:
            trace.write(json.dumps(dict(attached=attached,wall=time.time(),
                returncode=process.returncode,stdout=stdout,stderr=stderr))+'\n')
        if process.returncode or 'data: true' not in stdout:
            raise RuntimeError('IDEAL_VACUUM_SERVICE_REJECTED')
        while time.monotonic()<deadline:
            rclpy.spin_once(self.node,timeout_sec=.01)
            if monitor:
                reason=monitor(self.snapshot())
                if reason:raise RuntimeError(reason)
            state=self.snapshot()['vacuum']
            if (state and state['ready'] and state['attached']==attached
                    and state['processed_commands']>previous_commands):return state
        raise TimeoutError('IDEAL_VACUUM_STATE_ACK')

    def snapshot(self):
        with self.pose_lock:
            return dict(vacuum=self.vacuum_state,poses=dict(self.poses),pose_stamps=dict(self.pose_stamps),pose_stamp=self.pose_stamp,
                        messages=self.pose_count,q=self.q,q_stamp=self.q_stamp,
                        clock=self.clocks[-1] if self.clocks else None,
                        first_clock=self.first_clock,
                        capture_error=self.capture_error or (self.pose_capture.error if self.pose_capture else None)
                            or (self.contact_capture.error if self.contact_capture else None)
                            or (self.vacuum_capture.error if self.vacuum_capture else None)
                            or (self.clock_capture.error if self.clock_capture else None),
                        contact_topic=self.contact_topic,contacts=self.contacts,contact_stamp=self.contact_stamp,contact_messages=self.contact_count,positive_contact_messages=self.positive_contact_messages,
                        complete_contact_messages=self.complete_contact_messages,
                        contact_source='native_physics_component' if self.vacuum_spec else 'legacy_contact_topic',
                        native_contact_samples=self.native_contact_samples,
                        native_empty_contact_samples=self.native_empty_contact_samples)

    def stop(self):
        if self.clock_capture:self.clock_capture.stop()
        if self.pose_capture:self.pose_capture.stop()
        if self.contact_capture:self.contact_capture.stop()
        if self.vacuum_capture:self.vacuum_capture.stop()
        if hasattr(self,'node'):
            import rclpy
            self.node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
        if hasattr(self,'joint_trace'):self.joint_trace.close()
        if self.launch:
            # launch_scene has its own inner setsid launch; use its owned pidfile,
            # then terminate the outer waiting shell if still necessary.
            pidfile=Path('/tmp/elfin_uld_upper_scene.pid')
            if pidfile.exists():
                pid=pidfile.read_text().strip()
                result=subprocess.run(['/home/adamliao/work/elfin_humble_ws/scripts/stop_sim.sh',
                    '--owned-pgid',pid,'--pidfile',str(pidfile),'--wait','3'],
                    capture_output=True,text=True,timeout=25)
                (self.output/'teardown.log').write_text(result.stdout+result.stderr)
            try:self.launch.wait(timeout=10)
            except subprocess.TimeoutExpired:self.launch.terminate();self.launch.wait(timeout=5)
            self.launch_log.close()
