#!/usr/bin/env python3
"""Bounded read-only retries; never set parameters or release unverified goals."""
import argparse,json,math,sys,time
import rclpy,yaml
from rclpy.node import Node
from rcl_interfaces.srv import ListParameters,GetParameters
from ros2param.api import get_value

def finite(value):
    if isinstance(value,float):return math.isfinite(value)
    if isinstance(value,(list,tuple)):return all(finite(x) for x in value)
    return value is not None

def dump(name):
    if not name.startswith('/') or name.endswith('/'):raise ValueError('Absolute node name required')
    rclpy.init();node=Node('lio_parameter_snapshot_reader_v2')
    deadline=time.monotonic()+20;clients={}
    def remaining():return max(0.,deadline-time.monotonic())
    def log(**event):print(json.dumps(event),file=sys.stderr,flush=True)
    try:
        for suffix,typ in [('list_parameters',ListParameters),('get_parameters',GetParameters)]:
            client=node.create_client(typ,name+'/'+suffix);clients[suffix]=client
            if not client.wait_for_service(timeout_sec=min(5.,remaining())):
                raise RuntimeError('service not ready '+suffix)
        # Let the response discovery path progress before the first request.
        stable_until=min(deadline,time.monotonic()+1.)
        while time.monotonic()<stable_until:rclpy.spin_once(node,timeout_sec=min(.05,remaining()))
        namespace,basename=name.rsplit('/',1);namespace=namespace or '/'
        count=sum(n==basename and ns==namespace for n,ns in node.get_node_names_and_namespaces())
        if count!=1:raise RuntimeError('parameter node identity ambiguous or missing: '+str(count))
        def request(suffix,message):
            client=clients[suffix]
            for attempt in range(1,4):
                if remaining()<=0:break
                started=time.monotonic_ns();future=client.call_async(message)
                rclpy.spin_until_future_complete(node,future,timeout_sec=min(8.,remaining()))
                response=future.result() if future.done() and not future.cancelled() else None
                log(service=name+'/'+suffix,attempt=attempt,started_monotonic_ns=started,
                    ended_monotonic_ns=time.monotonic_ns(),response_received=response is not None,
                    read_only=True)
                if response is not None:return response
                client.remove_pending_request(future);future.cancel()
            raise RuntimeError('bounded parameter request timed out '+suffix)
        names=request('list_parameters',ListParameters.Request()).result.names
        if not names or len(set(names))!=len(names):raise ValueError('Empty or duplicate parameter names')
        response=request('get_parameters',GetParameters.Request(names=names))
        if len(response.values)!=len(names):raise ValueError('Parameter response count mismatch')
        values={key:get_value(parameter_value=value) for key,value in zip(names,response.values)}
        if not all(finite(value) for value in values.values()):raise ValueError('Unset or nonfinite parameter value')
        return {name:{'ros__parameters':values}}
    finally:
        for client in clients.values():node.destroy_client(client)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('node');args=parser.parse_args()
    print(yaml.safe_dump(dump(args.node)))
