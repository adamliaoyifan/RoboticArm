"""ROS/Gazebo adapter for the explicitly ideal-input research experiment.

Capture uses a private partition and owned launch. No observation callback moves
the robot or cargo. All raw transport traces survive failures before teardown.
"""
import json
import os
from pathlib import Path
import re
import subprocess
import threading
import time


class RuntimeIO:
    def __init__(self,repo,evidence,scene_dir=None):
        self.repo=Path(repo);self.output=Path(evidence)
        self.env=dict(os.environ,ROS_DOMAIN_ID='7',IGN_PARTITION='uld_upper_layer_closed_loop',
                      ROS_LOG_DIR=str(self.output/'ros_logs'))
        self.contact_topic='/upper_layer_payload/contacts'
        if scene_dir:
            self.env['ULD_UPPER_SCENE']=str(scene_dir)
            fixture=json.loads((Path(scene_dir)/'fixture.json').read_text())
            self.contact_topic=fixture.get('contact_observation_topic',self.contact_topic)
        self.launch=None;self.pose_capture=None;self.contact_capture=None
        self.contacts=None;self.contact_stamp=None;self.contact_count=0
        self.positive_contact_messages=0;self.complete_contact_messages=0
        self.pose_lock=threading.Lock();self.poses={};self.pose_stamp=None
        self.pose_count=0;self.q=None;self.q_stamp=None;self.clocks=[]
        self.first_clock=None
        self.capture_error=None

    def start(self):
        # launch_scene performs the machine-wide exclusivity check itself.
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
        os.environ['ROS_DOMAIN_ID']='7'
        import rclpy
        from sensor_msgs.msg import JointState
        from rosgraph_msgs.msg import Clock
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
        def clock(msg):
            self.clocks.append(msg.clock.sec+msg.clock.nanosec*1e-9)
            if self.first_clock is None:self.first_clock=self.clocks[-1]
            if len(self.clocks)>500:self.clocks=self.clocks[-500:]
        self.node.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data)
        self.node.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data)
        from transport_capture import JsonTopicCapture
        self.pose_capture=JsonTopicCapture(self.env,self.output,'poses',
            '/world/airport_loading/pose/info',self._accept_poses)
        self.pose_capture.start()
        self.contact_capture=JsonTopicCapture(self.env,self.output,'payload_contacts',
            self.contact_topic,self._accept_contacts)
        self.contact_capture.start()
        deadline=time.monotonic()+25
        while time.monotonic()<deadline:
            rclpy.spin_once(self.node,timeout_sec=.1)
            with self.pose_lock:ready='upper_layer_payload' in self.poses
            if self.q is not None and ready and len(self.clocks)>20:
                if self.clocks[-1]-self.first_clock>=5.:break
        else:raise TimeoutError('IDEAL_OBSERVATION_NOT_READY')
        count=self.node.count_publishers('/clock')
        if count!=1:raise RuntimeError(f'CLOCK_PUBLISHER_COUNT:{count}')

    def _accept_poses(self,row):
        header=row.get('header',{}).get('stamp',{})
        stamp=float(header.get('sec',0))+float(header.get('nsec',0))*1e-9
        with self.pose_lock:
            for item in row.get('pose',[]):
                if item.get('name'):self.poses[item['name']]=item
            self.pose_stamp=stamp;self.pose_count+=1

    def _accept_contacts(self,row):
        header=row.get('header',{}).get('stamp',{})
        stamp=float(header.get('sec',0))+float(header.get('nsec',0))*1e-9
        with self.pose_lock:
            self.contacts=row;self.contact_stamp=stamp;self.contact_count+=1
            contacts=row.get('contact',[])
            if contacts:self.positive_contact_messages+=1
            if contacts and all(len(c.get('normal',[]))==len(c.get('position',[]))>0
                                and len(c.get('depth',[]))==len(c['position']) for c in contacts):
                self.complete_contact_messages+=1

    def snapshot(self):
        with self.pose_lock:
            return dict(poses=dict(self.poses),pose_stamp=self.pose_stamp,
                        messages=self.pose_count,q=self.q,q_stamp=self.q_stamp,
                        clock=self.clocks[-1] if self.clocks else None,
                        first_clock=self.first_clock,
                        capture_error=self.capture_error or (self.pose_capture.error if self.pose_capture else None)
                            or (self.contact_capture.error if self.contact_capture else None),
                        contact_topic=self.contact_topic,contacts=self.contacts,contact_stamp=self.contact_stamp,contact_messages=self.contact_count,positive_contact_messages=self.positive_contact_messages,
                        complete_contact_messages=self.complete_contact_messages)

    def stop(self):
        if self.pose_capture:self.pose_capture.stop()
        if self.contact_capture:self.contact_capture.stop()
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
