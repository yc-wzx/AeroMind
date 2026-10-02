#!/usr/bin/env python3
"""Bounded batch parameter read, bypass CLI's one service call per parameter."""
import sys,json,argparse
from pathlib import Path
import yaml,rclpy
from rclpy.node import Node
from rcl_interfaces.srv import ListParameters,GetParameters
from ros2param.api import get_value

def dump(node_name):
    rclpy.init();n=Node('lio_parameter_snapshot_reader')
    try:
        def request(typ,suffix,req):
            c=n.create_client(typ,node_name+'/'+suffix)
            if not c.wait_for_service(timeout_sec=5):raise RuntimeError('service not ready '+suffix)
            f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=8)
            if not f.done() or f.result() is None:raise RuntimeError('parameter request timed out '+suffix)
            n.destroy_client(c);return f.result()
        names=request(ListParameters,'list_parameters',ListParameters.Request()).result.names
        response=request(GetParameters,'get_parameters',GetParameters.Request(names=names))
        if len(response.values)!=len(names):raise ValueError('parameter response count mismatch')
        return {node_name:{'ros__parameters':{k:get_value(parameter_value=v) for k,v in zip(names,response.values)}}}
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('node');a=p.parse_args();print(yaml.safe_dump(dump(a.node)))
