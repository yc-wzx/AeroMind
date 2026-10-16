#!/usr/bin/env python3
"""Private bounded input ordering; clocks come only from real /clock messages."""
import time
from rmw_diagnostic_executor import _SubscriptionBatch
from traced_rmw_executor import TracedRmwExecutor as RmwDiagnosticExecutor

class InputFirstExecutor(RmwDiagnosticExecutor):
    async def _prefetch(self,owner,group):
        entries=[]
        for sub in owner._input_first_subscriptions:
            if self._is_shutdown:break
            if sub.callback_group is not group:
                raise RuntimeError('Input-first callbacks require same callback group')
            before=time.monotonic();message=self._take_subscription(sub)
            count=0 if message is None else len(message.rows) if isinstance(message,_SubscriptionBatch) else 1
            if message is not None:
                await super()._execute_subscription(sub,message)
            entries.append(dict(topic=sub.topic_name,count=count,wall_s=time.monotonic()-before))
        return entries

    async def _execute_timer(self,tmr,ready):
        if self._is_shutdown:return
        owner=getattr(tmr.callback,'__self__',None)
        if ready and tmr.callback in getattr(owner,'_input_first_callbacks',()):
            started=time.monotonic();entries=await self._prefetch(owner,tmr.callback_group)
            if not self._is_shutdown:
                await super()._execute_timer(tmr,ready)
                hook=getattr(owner,'record_input_first',None)
                if hook is not None:hook(started,entries,time.monotonic()-started)
            return
        return await super()._execute_timer(tmr,ready)

    async def _execute_subscription(self,sub,msg):
        if self._is_shutdown:return
        owner=getattr(sub.callback,'__self__',None)
        if sub in getattr(owner,'_input_first_goal_subscriptions',()):
            started=time.monotonic();entries=await self._prefetch(owner,sub.callback_group)
            if not self._is_shutdown:
                # Original goal callback, including its freshness/route guards.
                await super()._execute_subscription(sub,msg)
                hook=getattr(owner,'record_input_first',None)
                if hook is not None:hook(started,entries,time.monotonic()-started)
            return
        return await super()._execute_subscription(sub,msg)
