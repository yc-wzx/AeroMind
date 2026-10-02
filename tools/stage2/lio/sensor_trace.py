"""Lossless per-received-message CDR archive, bounded asynchronous writer."""
import base64,time
from rclpy.serialization import serialize_message
from provincial_native_trace import NativeTrace,seconds

class SensorTrace(NativeTrace):
    def record(self,topic,msg):
        received=time.monotonic_ns();wall=time.time_ns()
        self.counts[topic]+=1
        row={'topic':topic,'receive_sequence':self.counts[topic],
            'receive_monotonic_ns':received,'receive_wall_ns':wall,'receive_sim_s':self.sim_s,
            'message_stamp_s':seconds(msg.header.stamp),
            'ros_type':msg.__class__.__module__.split('.')[0]+'/msg/'+msg.__class__.__name__,
            'encoding':'ROS2-CDR/base64','cdr':base64.b64encode(serialize_message(msg)).decode('ascii')}
        self._enqueue('message',row)
