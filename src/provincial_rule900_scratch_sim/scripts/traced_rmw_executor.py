"""Instrument actual production take/callback paths without extra DDS takes."""
import time
from rclpy.executors import await_or_execute
from rmw_diagnostic_executor import RmwDiagnosticExecutor as OriginalExecutor,_SubscriptionBatch
from internal_timing_trace import configured_trace,stamp
class TracedRmwExecutor(OriginalExecutor):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.internal_trace=configured_trace()
    def _record_info(self,sub,owner,info):
        super()._record_info(sub,owner,info)
        if self.internal_trace:self.internal_trace.submit(dict(kind='take',monotonic_ns=time.monotonic_ns(),topic=sub.topic_name,info=dict(info)))
    async def _one(self,sub,message,batch=False):
        if not self.internal_trace:
            return await await_or_execute(sub.callback,message) if batch else await super()._execute_subscription(sub,message)
        before=time.monotonic_ns();info=self.last_message_info.get(sub.topic_name)
        try:return await await_or_execute(sub.callback,message) if batch else await super()._execute_subscription(sub,message)
        finally:self.internal_trace.submit(dict(kind='subscription',start_ns=before,end_ns=time.monotonic_ns(),topic=sub.topic_name,callback=sub.callback.__qualname__,stamp_ns=stamp(message),info=dict(info) if info else None))
    async def _execute_subscription(self,sub,message):
        if not isinstance(message,_SubscriptionBatch):return await self._one(sub,message)
        owner=getattr(sub.callback,'__self__',None)
        # Identical original bounded loop, callback-group handler and row order.
        for row,info in message.rows:
            if self._is_shutdown:break
            self._record_info(sub,owner,info)
            await self._one(sub,row,batch=True)
    async def _execute_timer(self,timer,ready):
        if not self.internal_trace:return await super()._execute_timer(timer,ready)
        before=time.monotonic_ns();owner=getattr(timer.callback,'__self__',None)
        state={k:getattr(owner,k,None) for k in ('last_stage2_stamp','radar_stamp','x','y')}
        if owner is not None and hasattr(owner,'get_clock'):state['clock_ns']=owner.get_clock().now().nanoseconds
        try:return await super()._execute_timer(timer,ready)
        finally:self.internal_trace.submit(dict(kind='timer',start_ns=before,end_ns=time.monotonic_ns(),callback=timer.callback.__qualname__,state=state))
    def shutdown(self,*args,**kwargs):
        try:return super().shutdown(*args,**kwargs)
        finally:
            if self.internal_trace:self.internal_trace.close()
