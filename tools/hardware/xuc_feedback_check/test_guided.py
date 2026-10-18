import copy,math,unittest
from guided_yaw import GuidedYaw,summarize,PHASES
from inspect_wire import inspect_bytes
from tests import frame

def fixture(origin_angle=.01,direction=1.,missing_gap=False,invalid=False):
    origin=1_000_000_000_000;raw=bytearray();reads=[]
    phases=[dict(phase=p,start_monotonic_ns=origin+i*5_000_000_000,end_monotonic_ns=origin+(i+1)*5_000_000_000) for i,p in enumerate(PHASES)]
    for i in range(1250):
        t=i*.02
        if missing_gap and 6<t<6.4:continue
        a=origin_angle+direction*(0 if t<5 else .8*(t-5)/5 if t<10 else .8 if t<15 else .8*(20-t)/5 if t<20 else 0)
        wrapped=math.atan2(math.sin(a),math.cos(a));b=frame(poly=0x8408,angle=wrapped,flags=7 if invalid and i==400 else 15)
        offset=len(raw);raw.extend(b);stamp=origin+round(t*10**9)
        reads.append(dict(sequence=len(reads)+1,offset=offset,length=len(b),read_enter_monotonic_ns=stamp,read_return_monotonic_ns=stamp+1,receive_wall_ns=stamp+1000))
    result,frames=inspect_bytes(bytes(raw),reads,'0x8408')
    return result,frames,phases

class GuidedTests(unittest.TestCase):
    def test_explicit_crc_profile_required_without_fallback(self):
        from tests import receipts
        raw=frame(poly=0x8408)*4
        old,_=inspect_bytes(raw,receipts(raw));new,_=inspect_bytes(raw,receipts(raw),'0x8408')
        self.assertFalse(old['receive_profile_matches']);self.assertTrue(new['receive_profile_matches'])
        self.assertEqual(new['format_status'],'NAV39_CRC8408_OBSERVED')
    def test_left_and_return_and_nonzero_start_preserved(self):
        r,f,p=fixture();v=summarize(f,'0x8408',p,True,r['receive_profile_matches'])
        self.assertTrue(v['motion_check_complete']);self.assertAlmostEqual(v['hold_to_hold_left_deg'],math.degrees(.8),places=5)
        self.assertAlmostEqual(v['operator_declared_forward_encoder_candidate_rad'],.01,places=6)
        self.assertFalse(v['navigation_ready']);self.assertFalse(v['body_yaw_computed'])
    def test_wrap_does_not_reverse_direction(self):
        r,f,p=fixture(origin_angle=3.);v=summarize(f,'0x8408',p,True,True)
        self.assertTrue(v['motion_check_complete']);self.assertAlmostEqual(v['encoder_return_closure_deg'],0,places=4)
    def test_wrong_direction_not_passed(self):
        r,f,p=fixture(direction=-1.);self.assertFalse(summarize(f,'0x8408',p,True,True)['motion_check_complete'])
    def test_missing_gap_and_invalid_flag_rejected(self):
        for args in [dict(missing_gap=True),dict(invalid=True)]:
            r,f,p=fixture(**args)
            with self.assertRaises(ValueError):summarize(f,'0x8408',p,True,True)
    def test_incomplete_phases_or_capture_rejected(self):
        r,f,p=fixture()
        with self.assertRaises(ValueError):summarize(f,'0x8408',p[:-1],True,True)
        with self.assertRaises(ValueError):summarize(f,'0x8408',p,False,True)
    def test_no_input_does_not_prompt_operator_to_move(self):
        g=GuidedYaw('0x8408');g.start(1_000_000_000)
        with self.assertRaises(RuntimeError):g.tick(6_000_000_000)
        self.assertEqual(len(g.phases),1)

if __name__=='__main__':unittest.main(verbosity=2)
