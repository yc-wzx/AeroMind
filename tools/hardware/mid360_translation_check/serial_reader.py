"""Bounded receive-only UART thread for operator-timed translation. No transmit codec."""
import copy,fcntl,json,math,os,re,select,stat,sys,termios,threading,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'xuc_feedback_check'))
from capture import Writer,owners
from inspect_wire import crc16,decode,audit,sha

class FeedbackReader:
    def __init__(self,root):
        self.root=Path(root);self.root.mkdir(exist_ok=False);self.writer=Writer(self.root)
        self.fd=None;self.saved=None;self.exclusive=False;self.error=None;self.actual=None;self.device=None;self.restore_ok=None
        self.stop=threading.Event();self.thread=None;self.buffer=bytearray();self.lock=threading.Lock();self.latest=None;self.count=0;self.invalid=False
        self.start_ns=time.monotonic_ns();self.before=self.inputs()
    def inputs(self):
        paths=[Path(__file__),HERE.parent/'xuc_feedback_check/capture.py',HERE.parent/'xuc_feedback_check/inspect_wire.py']
        return {str(p.resolve()):sha(p) for p in paths}
    def start(self,device,allow_test_pty=False):
        self.device=Path(device).resolve(strict=True)
        if not stat.S_ISCHR(self.device.stat().st_mode):raise ValueError('Not a serial character device')
        if not re.fullmatch(r'/dev/tty(?:USB|ACM)\d+',str(self.device)) and not (allow_test_pty and str(self.device).startswith('/dev/pts/')):
            raise ValueError('Only ttyUSB/ttyACM production devices supported')
        existing=owners(self.device)
        if existing:raise RuntimeError('Serial occupied; no process stopped: '+str(existing))
        self.fd=os.open(str(self.device),os.O_RDONLY|os.O_NOCTTY|os.O_NONBLOCK)
        fcntl.flock(self.fd,fcntl.LOCK_EX|fcntl.LOCK_NB);fcntl.ioctl(self.fd,termios.TIOCEXCL);self.exclusive=True
        other=[pid for pid in owners(self.device) if pid!=os.getpid()]
        if other:raise RuntimeError('Concurrent serial owner detected')
        self.saved=termios.tcgetattr(self.fd);cfg=copy.deepcopy(self.saved);cfg[0]=cfg[1]=cfg[3]=0
        cfg[2]=(cfg[2]&~(termios.CSIZE|termios.PARENB|termios.CSTOPB|termios.CRTSCTS))|termios.CS8|termios.CLOCAL|termios.CREAD
        cfg[4]=cfg[5]=termios.B460800;cfg[6][termios.VMIN]=cfg[6][termios.VTIME]=0
        termios.tcsetattr(self.fd,termios.TCSANOW,cfg)
        observed=termios.tcgetattr(self.fd)
        if observed[4]!=termios.B460800 or observed[5]!=termios.B460800 or (observed[2]&termios.CSIZE)!=termios.CS8 or observed[2]&(termios.PARENB|termios.CSTOPB|termios.CRTSCTS) or observed[0] or observed[1] or observed[3]:
            raise RuntimeError('Actual UART mode differs from 460800/8N1/raw/no flow control')
        if fcntl.fcntl(self.fd,fcntl.F_GETFL)&os.O_ACCMODE!=os.O_RDONLY:raise RuntimeError('UART descriptor is not read-only')
        self.actual=dict(baud=460800,data_bits=8,parity='none',stop_bits=1,flow_control='none',descriptor_access='O_RDONLY',exclusive_ioctl=True,prior_bytes_flushed=False)
        self.thread=threading.Thread(target=self.receive);self.thread.start()
    def receive(self):
        try:
            while not self.stop.is_set():
                if self.writer.error:raise RuntimeError(self.writer.error)
                if not select.select([self.fd],[],[],.05)[0]:continue
                enter=time.monotonic_ns()
                try:data=os.read(self.fd,65536)
                except BlockingIOError:continue
                returned=time.monotonic_ns();wall=time.time_ns()
                if not data:raise RuntimeError('UART EOF/disconnected')
                self.writer.put(data,enter,returned,wall);self.feed(data,returned)
        except Exception as error:self.error=str(error)
    def feed(self,data,received):
        self.buffer.extend(data)
        while True:
            i=self.buffer.find(b'SP')
            if i<0:self.buffer[:]=b'S' if self.buffer[-1:]==b'S' else b'';return
            if i:del self.buffer[:i]
            if len(self.buffer)<39:return
            frame=bytes(self.buffer[:39])
            if crc16(frame[:-2],0x8408)!=int.from_bytes(frame[-2:],'little'):del self.buffer[0];continue
            del self.buffer[:39];r=decode(frame)
            valid=r['relative_yaw_valid'] and r['structural_fields_valid']
            with self.lock:
                self.invalid=not valid
                if valid:self.latest=(received,r['gimbal_relative_angle_rad']);self.count+=1
    def fresh(self):
        with self.lock:
            return bool(not self.error and not self.writer.error and not self.invalid and self.count>=3 and self.latest is not None and 0<=time.monotonic_ns()-self.latest[0]<=200_000_000)
    def close(self,failure=None,interrupted=False):
        self.stop.set()
        if self.thread:
            self.thread.join(3)
            if self.thread.is_alive():failure=failure or 'UART thread did not stop'
        if self.fd is not None:
            try:
                if self.saved is not None:termios.tcsetattr(self.fd,termios.TCSANOW,self.saved);self.restore_ok=True
            except Exception as error:self.restore_ok=False;failure=failure or 'TTY restore failed: '+str(error)
            finally:
                try:
                    if self.exclusive:fcntl.ioctl(self.fd,termios.TIOCNXCL)
                finally:os.close(self.fd);self.fd=None
        health=self.writer.finish();after=self.inputs()
        meta=dict(scope='RECEIVE-ONLY MANUALLY TIMED TRANSLATION UART',device=str(self.device),baud=460800,expected_crc_profile='0x8408',guided_turn=False,
            elapsed_monotonic_s=(time.monotonic_ns()-self.start_ns)*1e-9,actual_termios=self.actual,prior_termios_restore_succeeded=self.restore_ok,
            writer_health=health,failure=failure or self.error,interrupted=interrupted,serial_transmit_payload_bytes=0,
            tools_before=self.before,tools_after=after,input_tools_unchanged=self.before==after,
            data_sha256={n:sha(self.root/n) for n in ['raw.bin','reads.jsonl']} if health['closed'] else {})
        with (self.root/'capture_meta.json').open('x') as f:json.dump(meta,f,indent=2,allow_nan=False);f.write('\n')
        result,_=audit(self.root)
        with (self.root/'summary.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
        return result
