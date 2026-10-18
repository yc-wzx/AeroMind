"""Validate real inputs before forwarding unchanged messages to a separate LIO."""
import argparse, json, os, sys, time
import numpy as np
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent/'mid360'))
from core import inspect_cloud, imu_values, stamp_ns
from input_health import InputHealth, PoseHealth
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry

RAW = {'points':'/hardware/mid360/points', 'imu':'/hardware/mid360/imu_si'}
CHECKED = {'points':'/hardware/mid360/checked/points', 'imu':'/hardware/mid360/checked/imu_si'}

class Guard(Node):
    def __init__(self, output, observe_only=False):
        super().__init__('mid360_input_guard')
        self.output = Path(output)
        self.observe_only = observe_only
        self.engine = InputHealth()
        self.pose_health = PoseHealth()
        self.forwarded = dict(points=0, imu=0)
        self.sequence = 0
        self.last_status = 0
        self.sources_verified = False
        self.sources = {}
        self.events = (self.output/'input_events.jsonl').open('x', buffering=1)
        self.publishers_ = {}
        if not observe_only:
            self.publishers_['points'] = self.create_publisher(PointCloud2, CHECKED['points'], QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
            self.publishers_['imu'] = self.create_publisher(Imu, CHECKED['imu'], qos_profile_sensor_data)
        self.subscriptions_ = [
            self.create_subscription(PointCloud2, RAW['points'], lambda m:self.receive('points',m), QoSProfile(depth=10)),
            self.create_subscription(Imu, RAW['imu'], lambda m:self.receive('imu',m), qos_profile_sensor_data)]
        self.timer = self.create_timer(.02, self.tick)
        self.odom_subscription = self.create_subscription(Odometry,
            '/hardware/mid360/lio/odometry',self.receive_pose,10)

    def receive_pose(self, message):
        p,q=message.pose.pose.position,message.pose.pose.orientation
        try:
            if not self.pose_health.accept(stamp_ns(message),time.monotonic_ns(),
                    [p.x,p.y,p.z],[q.x,q.y,q.z,q.w]):
                self.engine.fail(self.pose_health.failure)
        except Exception as error:self.engine.fail('LIO output: '+str(error))

    def discover(self):
        self.sources = {k:[dict(name=p.node_name, namespace=p.node_namespace, gid=list(p.endpoint_gid))
                       for p in self.get_publishers_info_by_topic(t)] for k,t in RAW.items()}
        expected = {'points':'mid360_real_driver', 'imu':'mid360_imu_unit_adapter'}
        self.sources_verified = all(len(self.sources[k])==1 and self.sources[k][0]['name']==v and self.sources[k][0]['namespace']=='/' for k,v in expected.items())
        return self.sources_verified

    def receive(self, kind, message):
        receipt, wall = time.monotonic_ns(), time.time_ns()
        self.sequence += 1
        details, accepted, stamp = {}, False, None
        try:
            if self.engine.failure or not self.sources_verified:
                return
            stamp = stamp_ns(message)
            if kind == 'points':
                details = inspect_cloud(message)
                field=next(f for f in message.fields if f.name=='timestamp')
                times=np.ndarray((message.height,message.width),dtype=('>f8' if message.is_bigendian else '<f8'),buffer=message.data,offset=field.offset,strides=(message.row_step,message.point_step))
                details['minimum_point_stamp_ns']=int(np.min(times))
                details['maximum_point_stamp_ns']=int(np.max(times))
                # MID360 packets can interleave point times; upstream deskew sorts them.
                # Keep/count this fact without editing payloads. Span and message-time
                # continuity remain mandatory; interleaving alone is not a bad scan.
            else:
                details = imu_values(message)
                if message.header.frame_id != 'mid360_imu':
                    raise ValueError('Wrong physical IMU frame')
            accepted = self.engine.accept(kind, stamp, receipt,details.get('maximum_point_stamp_ns'))
            if accepted and not self.observe_only:
                self.publishers_[kind].publish(message)
                self.forwarded[kind] += 1
        except Exception as error:
            self.engine.fail(kind + ': ' + str(error))
        finally:
            self.events.write(json.dumps(dict(sequence=self.sequence,kind=kind,receive_monotonic_ns=receipt,
                receive_wall_ns=wall,stamp_ns=stamp,accepted=accepted,forwarded=accepted and not self.observe_only,
                details=details,failure=self.engine.failure),allow_nan=False)+'\n')

    def status(self, closed=False):
        now = time.monotonic_ns()
        ready = self.engine.ready(now)
        return dict(scope='INPUT VALIDITY GUARD; NOT NAVIGATION ACCEPTANCE',state='FAULT_LATCHED' if self.engine.failure else 'READY' if ready else 'WARMUP',
                    failure=self.engine.failure,counts=self.engine.counts,forwarded=self.forwarded,latest=self.engine.latest,
                    sources=self.sources,source_identity_verified=self.sources_verified,monotonic_ns=now,wall_ns=time.time_ns(),
                    closed=closed,observe_only=self.observe_only,payload_or_timestamp_edited=False,automatic_restart=False,
                    pose_sanity_failure=self.pose_health.failure)

    def tick(self):
        now = time.monotonic_ns()
        self.engine.tick(now)
        if now-self.last_status >= 100_000_000:
            self.last_status = now
            tmp = self.output/'guard_status.tmp'
            tmp.write_text(json.dumps(self.status(),allow_nan=False)+'\n')
            os.replace(tmp, self.output/'guard_status.json')

def main():
    p=argparse.ArgumentParser();p.add_argument('--output-dir',required=True);p.add_argument('--observe-only',action='store_true');a=p.parse_args()
    out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=Guard(out,a.observe_only);exit_code=0
    try:
        deadline=time.monotonic()+8
        while not node.sources_verified:
            rclpy.spin_once(node,timeout_sec=.02)
            if node.discover():break
            if time.monotonic()>deadline:raise RuntimeError('Expected unique real radar/IMU sources unavailable')
        while rclpy.ok() and not node.engine.failure:
            rclpy.spin_once(node,timeout_sec=.02)
        if node.engine.failure:exit_code=2
    except KeyboardInterrupt:pass
    except Exception as error:node.engine.fail(str(error));exit_code=2
    finally:
        (out/'guard_status.json').write_text(json.dumps(node.status(closed=True),indent=2,allow_nan=False)+'\n')
        node.events.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return exit_code
if __name__=='__main__':raise SystemExit(main())
