"""Default-off bounded asynchronous monotonic diagnostics; no ROS publication."""
import os,time,json,queue,threading
from pathlib import Path
class TimingTrace:
    def __init__(self,path,capacity=20000):
        self.path=Path(path);self.queue=queue.Queue(maxsize=capacity)
        self.accepted=0;self.received=0;self.written=0;self.dropped=0;self.errors=[];self.closed=False;self.peak=0
        self.file=self.path.open('x');self.worker=threading.Thread(target=self._work,name='internal-timing-writer');self.worker.start()
    def submit(self,row):
        self.received+=1
        if self.closed:self.dropped+=1;return
        try:
            self.queue.put_nowait(dict(row,sequence=self.received));self.accepted+=1;self.peak=max(self.peak,self.queue.qsize())
        except queue.Full:self.dropped+=1
    def _work(self):
        try:
            while True:
                row=self.queue.get()
                if row is None:break
                self.file.write(json.dumps(row,allow_nan=True,separators=(',',':'))+'\n');self.written+=1
        except BaseException as error:self.errors.append(repr(error))
        finally:
            try:self.file.flush();self.file.close()
            except BaseException as error:self.errors.append(repr(error))
    def close(self):
        if self.closed:return
        self.closed=True
        while self.worker.is_alive():
            try:self.queue.put(None,timeout=.05);break
            except queue.Full:continue
        self.worker.join(timeout=10)
        health=dict(received=self.received,accepted=self.accepted,written=self.written,dropped=self.dropped,errors=self.errors,closed=not self.worker.is_alive(),queue_remaining=self.queue.qsize(),peak=self.peak,capacity=self.queue.maxsize,pid=os.getpid(),clock='time.monotonic_ns / Linux CLOCK_MONOTONIC')
        self.path.with_suffix('.health.json').write_text(json.dumps(health,indent=2))
TRACE=None
def configured_trace():
    global TRACE
    root=os.environ.get('AEROMIND_INTERNAL_TIMING_DIR')
    if root and TRACE is None:TRACE=TimingTrace(Path(root)/('python_'+str(os.getpid())+'.jsonl'))
    return TRACE
def stamp(message):
    value=getattr(getattr(message,'header',None),'stamp',None)
    if value is None:value=getattr(message,'clock',None)
    return None if value is None else int(value.sec)*1000000000+int(value.nanosec)
