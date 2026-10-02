#!/usr/bin/env python3
"""Isolated parameter protocol fixture, deliberately delayed or incomplete."""
import argparse,json,time
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import ListParameters,GetParameters
from rcl_interfaces.msg import ParameterValue,ParameterType

parser=argparse.ArgumentParser();parser.add_argument('--mode',choices=['delayed','always_delayed','nan','missing'],required=True);args=parser.parse_args()
rclpy.init();node=Node('parameter_protocol_fixture',start_parameter_services=False);calls=0
def listed(request,response):
    global calls
    calls+=1
    print(json.dumps({'event':'list_received','index':calls,'monotonic_ns':time.monotonic_ns()}),flush=True)
    if args.mode=='always_delayed' or (args.mode=='delayed' and calls==1):time.sleep(8.3)
    response.result.names=['mapping.acc_cov','mapping.gyr_cov','mapping.b_acc_cov','mapping.b_gyr_cov','common.planar_mode']
    print(json.dumps({'event':'list_returned','index':calls,'monotonic_ns':time.monotonic_ns()}),flush=True)
    return response
def get(request,response):
    values={'mapping.acc_cov':1e-6,'mapping.gyr_cov':1e-8,'mapping.b_acc_cov':0.,'mapping.b_gyr_cov':0.,'common.planar_mode':False}
    for name in request.names:
        if args.mode=='missing' and name=='mapping.gyr_cov':continue
        value=values[name]
        if args.mode=='nan' and name=='mapping.acc_cov':value=float('nan')
        response.values.append(ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=value) if isinstance(value,bool)
            else ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=value))
    return response
node.create_service(ListParameters,'/parameter_protocol_fixture/list_parameters',listed)
node.create_service(GetParameters,'/parameter_protocol_fixture/get_parameters',get)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:node.destroy_node();rclpy.try_shutdown()
