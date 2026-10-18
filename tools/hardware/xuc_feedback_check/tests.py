import copy,json,os,pty,select,struct,tempfile,threading,time,unittest
from pathlib import Path
from inspect_wire import crc16,inspect_bytes,audit
from capture import capture_session

def frame(poly=0xa001,angle=.3,flags=15):
    b=bytearray(39);b[:2]=b'SP';b[22]=1
    struct.pack_into('<Hfff',b,23,flags,.1,.2,angle)
    struct.pack_into('<H',b,37,crc16(b[:37],poly));return bytes(b)

def receipts(raw):
    return [dict(sequence=1,offset=0,length=len(raw),read_enter_monotonic_ns=100,read_return_monotonic_ns=200,receive_wall_ns=1000)] if raw else []

class WireTests(unittest.TestCase):
    def test_known_crc_vectors(self):
        self.assertEqual(crc16(b'123456789',0xa001),0x4b37)
        self.assertEqual(crc16(b'123456789',0x8408),0x6f91)
    def test_navigation_frame_only(self):
        raw=frame()*8;result,rows=inspect_bytes(raw,receipts(raw))
        self.assertEqual(result['format_status'],'NAV39_MODBUS_OBSERVED');self.assertEqual(len(rows),8)
        self.assertTrue(all(r['relative_yaw_valid'] for r in rows));self.assertFalse(result['navigation_ready'])
        self.assertAlmostEqual(rows[0]['gimbal_relative_angle_rad'],.3,places=6)
        self.assertIsNone(rows[0]['device_measurement_stamp']);self.assertNotIn('imu_yaw_rad',rows[0])
    def test_invalid_angle_flag_never_becomes_valid(self):
        raw=frame(flags=7)*4;result,rows=inspect_bytes(raw,receipts(raw))
        self.assertEqual(result['format_status'],'NAV39_MODBUS_OBSERVED')
        self.assertTrue(all(not r['relative_yaw_valid'] and r['gimbal_relative_angle_rad'] is None for r in rows))
    def test_nan_valid_angle_rejected(self):
        raw=frame(angle=float('nan'))*4;result,rows=inspect_bytes(raw,receipts(raw))
        self.assertEqual(result['format_status'],'NAVIGATION_FEEDBACK_NOT_OBSERVED')
        self.assertTrue(all(not r['relative_yaw_valid'] for r in rows))
    def test_corrupt_and_partial_bytes_not_passed(self):
        broken=bytearray(frame());broken[-1]^=1
        raw=b'noise'+bytes(broken)*4+frame()[:20];result,_=inspect_bytes(raw,receipts(raw))
        self.assertEqual(result['format_status'],'NAVIGATION_FEEDBACK_NOT_OBSERVED')
    def test_diagnostic_crc_does_not_enable_expected_codec(self):
        raw=frame(poly=0x8408)*5;result,_=inspect_bytes(raw,receipts(raw))
        self.assertEqual(result['format_status'],'NAV39_DIAGNOSTIC_CRC_ONLY');self.assertIsNone(result['selected_for_control'])
    def test_ambiguous_crc_remains_ambiguous(self):
        raw=frame()*4+frame(poly=0x8408)*4;result,_=inspect_bytes(raw,receipts(raw))
        self.assertEqual(result['format_status'],'AMBIGUOUS_CANDIDATES')
    def test_missing_or_regressed_receipts_rejected(self):
        raw=frame()*4;r=receipts(raw);r[0]['offset']=1
        with self.assertRaises(ValueError):inspect_bytes(raw,r)
        with self.assertRaises(ValueError):inspect_bytes(raw,[])
        r=receipts(raw);r[0]['read_enter_monotonic_ns']=201
        with self.assertRaises(ValueError):inspect_bytes(raw,r)

    def test_noninteger_receipt_rejected(self):
        raw=frame()*4;r=receipts(raw);r[0]['read_return_monotonic_ns']=float('nan')
        with self.assertRaises(ValueError):inspect_bytes(raw,r)
    def test_missing_original_hashes_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'capture_meta.json').write_text(json.dumps(dict(writer_health=dict(closed=True),data_sha256={})))
            with self.assertRaises(ValueError):audit(root)

class CaptureTests(unittest.TestCase):
    def test_production_capture_on_virtual_uart_and_no_tx(self):
        master,slave=pty.openpty();device=os.ttyname(slave);os.close(slave);os.set_blocking(master,False)
        peer_bytes=[];peer_error=[]
        def peer():
            try:
                time.sleep(.5)
                for _ in range(100):
                    os.write(master,frame());time.sleep(.01)
                    if select.select([master],[],[],0)[0]:
                        try:peer_bytes.append(os.read(master,4096))
                        except (BlockingIOError,OSError):pass
            except Exception as e:peer_error.append(repr(e))
        thread=threading.Thread(target=peer);thread.start()
        try:
            with tempfile.TemporaryDirectory() as folder:
                root=Path(folder)/'capture';r=capture_session(root,device,460800,2.,allow_test_pty=True)
                self.assertTrue(r['capture_complete'],r);self.assertEqual(r['format_status'],'NAV39_MODBUS_OBSERVED')
                self.assertEqual(r['raw_bytes'],3900);self.assertFalse(any(peer_bytes));self.assertFalse(peer_error)
                reread,_=audit(root);self.assertEqual(reread['original_data_sha256'],r['original_data_sha256'])
                self.assertEqual(reread['candidate_reports'],r['candidate_reports'])
                with (root/'raw.bin').open('ab') as f:f.write(b'X')
                with self.assertRaises(ValueError):audit(root)
        finally:thread.join(3);os.close(master)
    def test_existing_serial_owner_refused_without_read(self):
        master,slave=pty.openpty();device=os.ttyname(slave)
        try:
            with tempfile.TemporaryDirectory() as folder:
                r=capture_session(Path(folder)/'capture',device,460800,1.,allow_test_pty=True)
                self.assertFalse(r['capture_complete']);self.assertEqual(r['raw_bytes'],0)
                self.assertIn('already open',r['capture_failure'])
        finally:os.close(slave);os.close(master)

if __name__=='__main__':unittest.main(verbosity=2)
