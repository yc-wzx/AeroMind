"""Exercise the actual new executable in an isolated DDS domain, no sensors."""
import json
import os
from pathlib import Path
import resource
import signal
import subprocess
import tempfile
import time
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[3]
BUILD=ROOT/'tools/results/stage2_step4_lio_optimization_20261002/candidate_build'


class Tests(unittest.TestCase):
    def run_value(self,value,expected,good):
        with tempfile.TemporaryDirectory(prefix='lio_variance_') as tmp:
            config=Path(tmp)/'params.yaml'
            params={} if value is None else {'mapping.laser_point_covariance':value}
            config.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
            env=dict(os.environ,ROS_DOMAIN_ID='74')
            env['LD_LIBRARY_PATH']=str(BUILD)+':'+env.get('LD_LIBRARY_PATH','')
            def no_core(): resource.setrlimit(resource.RLIMIT_CORE,(0,0))
            p=subprocess.Popen([str(BUILD/'spark_lio_mapping'),'--ros-args','-r','__node:=variance_isolated_test',
                '--params-file',str(config)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                text=True,env=env,start_new_session=True,preexec_fn=no_core)
            try:
                time.sleep(1.0)
                if good:
                    self.assertIsNone(p.poll())
                # Generated component executables may keep an empty executor
                # alive after refusing construction. Test the actual refusal,
                # not an assumed process exit convention.
                if p.poll() is None:
                    os.killpg(p.pid,signal.SIGINT)
                try:
                    text,_=p.communicate(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGTERM)
                    try:
                        text,_=p.communicate(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(p.pid,signal.SIGKILL)
                        text,_=p.communicate(timeout=2)
            finally:
                if p.poll() is None:
                    os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
            if good:
                self.assertIn('POINT_VARIANCE_ACTUAL',text)
                actual=float(text.split('POINT_VARIANCE_ACTUAL ')[1].split()[0])
                self.assertEqual(actual,expected)
            else:
                self.assertIn('must be finite and positive',text)
                self.assertNotIn('POINT_VARIANCE_ACTUAL',text)

    def test_original_default(self): self.run_value(None,.001,True)
    def test_explicit_candidate(self): self.run_value(.0001,.0001,True)
    def test_zero_rejected(self): self.run_value(0.,None,False)
    def test_negative_rejected(self): self.run_value(-.0001,None,False)
    def test_nonfinite_rejected(self): self.run_value(float('nan'),None,False)


if __name__=='__main__':unittest.main(verbosity=2)
