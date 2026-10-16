#!/usr/bin/env python3
"""Read RMW info discarded by this installed Humble executor; no extra takes.

Version-specific protected hook, preserving its single-argument callback API.
MessageInfo timestamps are RMW system clocks, never header generation times or
monotonic callback times. Missing fields stay None.
"""
import time
from rclpy.executors import SingleThreadedExecutor, await_or_execute
FIELDS=('source_timestamp','received_timestamp','publication_sequence_number','reception_sequence_number')
def metadata(info):
    def value(name):return info.get(name) if isinstance(info,dict) else getattr(info,name,None)
    result={name:value(name) for name in FIELDS}
    gid=value('publisher_gid');result['publisher_gid']=None if gid is None else list(gid)
    return result
class RmwDiagnosticExecutor(SingleThreadedExecutor):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.last_message_info={}
    def _take_subscription(self,sub):
        owner=getattr(sub.callback,'__self__',None)
        limit=getattr(owner,'_subscription_batch_limits',{}).get(sub.topic_name,1)
        if type(limit) is not int or not 1<=limit<=8:
            raise ValueError('subscription batch must be integer1..8')
        rows=[]
        with sub.handle:
            for _ in range(limit):
                taken=sub.handle.take_message(sub.msg_type,sub.raw)
                if taken is None:break
                info=metadata(taken[1]);info['take_system_timestamp_ns']=time.time_ns();info['take_monotonic_s']=time.monotonic()
                rows.append((taken[0],info))
        if not rows:return None
        if limit==1:
            msg,info=rows[0]
            self._record_info(sub,owner,info)
            return msg
        return _SubscriptionBatch(rows)

    def _record_info(self,sub,owner,info):
        self.last_message_info[sub.topic_name]=info
        hook=getattr(owner,'record_rmw_info',None)
        if hook is not None:hook(sub.topic_name,info)

    async def _execute_subscription(self,sub,msg):
        if not isinstance(msg,_SubscriptionBatch):
            return await super()._execute_subscription(sub,msg)
        owner=getattr(sub.callback,'__self__',None)
        # Stay inside the parent's callback-group/work-tracker handler. No
        # background thread, no extra wait, no synthetic message or reordering.
        for row,info in msg.rows:
            if self._is_shutdown:break
            self._record_info(sub,owner,info)
            await await_or_execute(sub.callback,row)

class _SubscriptionBatch:
    def __init__(self,rows):self.rows=rows
