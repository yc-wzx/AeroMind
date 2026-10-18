"""Read-only recorder with independently decoded corrected-pose health."""
import sys, threading
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'mid360_gimbal_check'))
from record import Recorder,deserialize_message,Odometry,stamp_ns,pose_row
from input_health import PoseHealth

class HealthyRecorder(Recorder):
    def __init__(self,path):
        self.pose_lock=threading.Lock()
        self.pose_health=PoseHealth()
        super().__init__(path)

    def receive(self,kind,payload):
        super().receive(kind,payload)
        if kind=='corrected':
            try:
                m=deserialize_message(payload,Odometry)
                pose=pose_row(m)
                with self.lock:receipt=self.latest[kind][0]
                with self.pose_lock:
                    self.pose_health.accept(stamp_ns(m),receipt,pose['position'],pose['quaternion'])
            except Exception as error:
                with self.pose_lock:self.pose_health.failure=str(error)

    def fresh(self):
        with self.pose_lock:good=self.pose_health.failure is None
        return good and super().fresh()

    def static_report(self,start_ns=None):
        with self.pose_lock:return self.pose_health.static_report(start_ns=start_ns)
