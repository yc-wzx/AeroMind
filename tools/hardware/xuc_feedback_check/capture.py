"""Linux receive-only serial byte capture. Serial descriptor is O_RDONLY; no TX codec exists."""
import argparse,copy,fcntl,json,os,queue,re,select,stat,subprocess,termios,threading,time
from pathlib import Path
from inspect_wire import audit,sha

class Writer:
    def __init__(self,root):
        self.raw=(root/'raw.bin').open('xb');self.reads=(root/'reads.jsonl').open('x')
        self.q=queue.Queue(maxsize=1024);self.stop=threading.Event();self.error=None;self.overflow=0
        self.received_bytes=self.received_chunks=self.written_bytes=self.written_chunks=0;self.closed=False
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def put(self,payload,enter,returned,wall):
        row=dict(sequence=self.received_chunks+1,offset=self.received_bytes,length=len(payload),
            read_enter_monotonic_ns=enter,read_return_monotonic_ns=returned,receive_wall_ns=wall)
        self.received_chunks+=1;self.received_bytes+=len(payload)
        try:self.q.put_nowait((payload,row))
        except queue.Full:self.overflow+=1;raise RuntimeError('Capture write queue overflow')
    def run(self):
        try:
            while not self.stop.is_set() or not self.q.empty():
                try:payload,row=self.q.get(timeout=.05)
                except queue.Empty:continue
                if row['offset']!=self.written_bytes:raise RuntimeError('Writer byte coverage mismatch')
                if self.raw.write(payload)!=len(payload):raise RuntimeError('Short raw-file write')
                self.reads.write(json.dumps(row)+'\n');self.written_bytes+=len(payload);self.written_chunks+=1
        except Exception as error:self.error=repr(error)
        finally:
            try:self.raw.close();self.reads.close();self.closed=True
            except Exception as error:self.error=repr(error)
    def finish(self):
        self.stop.set();self.thread.join(5)
        if self.thread.is_alive():self.error='Writer did not close within 5 seconds'
        return {key:getattr(self,key) for key in ['received_bytes','received_chunks','written_bytes','written_chunks','overflow','error','closed']}

def owners(device):
    result=subprocess.run(['fuser',str(device)],text=True,capture_output=True,timeout=3)
    if result.returncode not in (0,1):raise RuntimeError('Cannot inspect serial ownership: '+result.stderr.strip())
    return sorted(set(int(v) for v in re.findall(r'\b\d+\b',result.stdout)))

def capture_session(root,device,baud,seconds,allow_test_pty=False):
    root=Path(root);device=Path(device).resolve(strict=True)
    if not stat.S_ISCHR(device.stat().st_mode):raise ValueError('Device must be a serial character device')
    if not re.fullmatch(r'/dev/tty(?:USB|ACM)\d+',str(device)) and not (allow_test_pty and str(device).startswith('/dev/pts/')):
        raise ValueError('Supported production devices are ttyUSB/ttyACM only')
    if baud!=460800:raise ValueError('Only the operator-confirmed 460800 baud is supported in this probe')
    if not 1<=seconds<=30:raise ValueError('Duration must be 1..30 seconds')
    root.mkdir(parents=True,exist_ok=False);tools_before={p.name:sha(p) for p in Path(__file__).parent.glob('*.py')};writer=Writer(root);fd=None;saved=None;exclusive=False
    failure=None;interrupted=False;actual=None;start=time.monotonic_ns();restore_ok=None
    try:
        existing=owners(device)
        if existing:raise RuntimeError('Serial already open by PIDs '+','.join(map(str,existing))+'; no process stopped')
        fd=os.open(str(device),os.O_RDONLY|os.O_NOCTTY|os.O_NONBLOCK)
        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        fcntl.ioctl(fd,termios.TIOCEXCL);exclusive=True
        other=[pid for pid in owners(device) if pid!=os.getpid()]
        if other:raise RuntimeError('Another serial owner appeared: '+str(other))
        saved=termios.tcgetattr(fd);configured=copy.deepcopy(saved)
        configured[0]=0;configured[1]=0;configured[3]=0
        configured[2]=(configured[2]&~(termios.CSIZE|termios.PARENB|termios.CSTOPB|termios.CRTSCTS))|termios.CS8|termios.CLOCAL|termios.CREAD
        configured[4]=configured[5]=termios.B460800
        configured[6][termios.VMIN]=0;configured[6][termios.VTIME]=0
        termios.tcsetattr(fd,termios.TCSANOW,configured)  # No flushing of pre-existing bytes.
        observed=termios.tcgetattr(fd)
        if observed[4]!=termios.B460800 or observed[5]!=termios.B460800:raise RuntimeError('Actual baud differs')
        if (observed[2]&termios.CSIZE)!=termios.CS8 or observed[2]&(termios.PARENB|termios.CSTOPB|termios.CRTSCTS):raise RuntimeError('Actual framing differs from 8N1/no flow control')
        if observed[0] or observed[1] or observed[3]:raise RuntimeError('Raw receive TTY flags not applied')
        if fcntl.fcntl(fd,fcntl.F_GETFL)&os.O_ACCMODE!=os.O_RDONLY:raise RuntimeError('Serial descriptor is not read-only')
        actual=dict(baud=460800,data_bits=8,parity='none',stop_bits=1,flow_control='none',exclusive_ioctl=True,
                    descriptor_access='O_RDONLY',prior_bytes_flushed=False)
        print('仅接收原始串口数据；不发送指令。请保持底盘和云台静止。',flush=True)
        start=time.monotonic_ns();deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if writer.error:raise RuntimeError('Writer failed: '+writer.error)
            if not select.select([fd],[],[],min(.1,max(0.,deadline-time.monotonic())))[0]:continue
            enter=time.monotonic_ns()
            try:payload=os.read(fd,65536)
            except BlockingIOError:continue
            returned=time.monotonic_ns();wall=time.time_ns()
            if not payload:raise RuntimeError('Serial returned EOF/disconnected')
            writer.put(payload,enter,returned,wall)
    except KeyboardInterrupt:interrupted=True;failure='Operator interrupted'
    except Exception as error:failure=repr(error)
    finally:
        if fd is not None:
            try:
                if saved is not None:termios.tcsetattr(fd,termios.TCSANOW,saved);restore_ok=True
            except Exception as error:restore_ok=False;failure=failure or 'Could not restore prior TTY settings: '+repr(error)
            try:
                if exclusive:fcntl.ioctl(fd,termios.TIOCNXCL)
            finally:os.close(fd)
        health=writer.finish()
    names=['raw.bin','reads.jsonl'];meta=dict(scope='RECEIVE-ONLY SERIAL FORMAT PROBE',device=str(device),baud=baud,
        requested_seconds=seconds,elapsed_monotonic_s=(time.monotonic_ns()-start)*1e-9,
        failure=failure,interrupted=interrupted,actual_termios=actual,prior_termios_restore_succeeded=restore_ok,
        writer_health=health,serial_transmit_payload_bytes=0,
        tools_before=tools_before,tools_after={p.name:sha(p) for p in Path(__file__).parent.glob('*.py')},
        input_tools_unchanged=tools_before=={p.name:sha(p) for p in Path(__file__).parent.glob('*.py')},
        data_sha256={name:sha(root/name) for name in names} if health['closed'] else {})
    with (root/'capture_meta.json').open('x') as f:json.dump(meta,f,indent=2,allow_nan=False);f.write('\n')
    if not health['closed']:return dict(capture_complete=False,capture_failure='Writer did not close',writer_health=health)
    result,frames=audit(root)
    with (root/'crc_matched_frames.derived.jsonl').open('x') as f:
        for row in frames:f.write(json.dumps(row,allow_nan=False)+'\n')
    result['derived_frames_sha256']=sha(root/'crc_matched_frames.derived.jsonl')
    with (root/'summary.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--device',required=True);p.add_argument('--baud',type=int,required=True);p.add_argument('--seconds',type=float,default=15)
    p.add_argument('--output-dir');a=p.parse_args();root=Path(a.output_dir) if a.output_dir else Path.home()/'AeroMind_xuc_feedback_checks'/('raw_'+time.strftime('%Y%m%d_%H%M%S'))
    result=capture_session(root,a.device,a.baud,a.seconds)
    print(json.dumps(result,indent=2,allow_nan=False));print('SUMMARY:',root/'summary.json')
    return 0 if result.get('capture_complete') and result.get('format_status')=='NAV39_MODBUS_OBSERVED' and any(r['relative_yaw_valid_frames']>=3 and r['crc_algorithm']=='modbus_a001_ffff' for r in result.get('candidate_reports',[])) else 2
if __name__=='__main__':raise SystemExit(main())
