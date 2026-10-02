from pathlib import Path
import argparse
import json, math
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description="Generate a Gazebo world from one field geometry JSON")
parser.add_argument("--field", type=Path, default=root/"src/uav_planning/config/competition_field_2025.json")
parser.add_argument("--world", type=Path, default=root/"src/uav_bringup/worlds/competition_2025.sdf")
args = parser.parse_args()
field = json.loads(args.field.read_text(encoding="utf-8"))
sdf = ET.Element("sdf", version="1.8")
world = ET.SubElement(sdf, "world", name=field.get("world_name", "competition"))
def tag(parent, name, value, **attrs):
    e=ET.SubElement(parent,name,attrs); e.text=str(value); return e
physics = ET.SubElement(world,"physics",name="navigation",type="ignored")
tag(physics,"max_step_size",0.004); tag(physics,"real_time_factor",1.0)
tag(world,"gravity","0 0 -9.81")
for filename, name in [("physics","Physics"),("user-commands","UserCommands"),("scene-broadcaster","SceneBroadcaster"),("sensors","Sensors")]:
    p=ET.SubElement(world,"plugin",filename=f"ignition-gazebo-{filename}-system",name=f"ignition::gazebo::systems::{name}")
    if name=="Sensors": tag(p,"render_engine","ogre2")
scene=ET.SubElement(world,"scene")
tag(scene,"ambient","0.65 0.65 0.65 1"); tag(scene,"background","0.16 0.19 0.24 1"); tag(scene,"shadows","false")
light=ET.SubElement(world,"light",name="sun",type="directional")
tag(light,"pose","0 0 12 0 0 0"); tag(light,"diffuse","0.8 0.8 0.8 1")
tag(light,"direction","-0.4 0.2 -0.9"); tag(light,"cast_shadows","false")
def box(link,name,pose,size,color,collision=False):
    if collision:
        col=ET.SubElement(link,"collision",name=name+"_collision")
        tag(col,"pose",pose)
        tag(ET.SubElement(ET.SubElement(col,"geometry"),"box"),"size",size)
        surface=ET.SubElement(col,"surface")
        ode=ET.SubElement(ET.SubElement(surface,"friction"),"ode")
        tag(ode,"mu",0.0); tag(ode,"mu2",0.0)
    v=ET.SubElement(link,"visual",name=name)
    tag(v,"pose",pose); tag(ET.SubElement(ET.SubElement(v,"geometry"),"box"),"size",size)
    mat=ET.SubElement(v,"material"); tag(mat,"ambient",color); tag(mat,"diffuse",color)
def static_box(name,x,y,z,w,h,d,color,collision=False,yaw=0):
    m=ET.SubElement(world,"model",name=name); tag(m,"static","true")
    tag(m,"pose",f"{x} {y} {z} 0 0 {yaw}")
    l=ET.SubElement(m,"link",name="link")
    box(l,name,"0 0 0 0 0 0",f"{w} {h} {d}",color,collision)
static_box("arena_floor",4.7,5,-0.05,9.4,10,0.1,"0.30 0.20 0.38 1",True)
for i,r in enumerate(field["course_surfaces"]):
    static_box(f"course_{i}",r["x"],r["y"],0.002,r["width"],r["height"],0.004,"0.52 0.55 0.60 1")
for name,color in [("start","0.9 0.05 0.05 1"),("shooting_zone","0.04 0.25 0.95 1"),("target_zone","0.02 0.7 0.16 1")]:
    r=field[name]
    static_box(name,r["x"],r["y"],0.006,r["width"],r["height"],0.006,color)
for i,(x1,y1,x2,y2) in enumerate(field["collision_segments"]):
    static_box(f"boundary_{i}",(x1+x2)/2,(y1+y2)/2,0.27,
               math.hypot(x2-x1,y2-y1),0.055,0.54,"0.88 0.89 0.93 1",True,math.atan2(y2-y1,x2-x1))
for i,(a,b) in enumerate(zip(field["reference_route"],field["reference_route"][1:])):
    x1,y1=a; x2,y2=b
    static_box(f"route_{i}",(x1+x2)/2,(y1+y2)/2,0.014,
               math.hypot(x2-x1,y2-y1),0.065,0.008,"0.05 1 0.2 1",False,math.atan2(y2-y1,x2-x1))
# Ideal holonomic rigid body, collision footprint matches navigation model.
m=ET.SubElement(world,"model",name="omni_robot")
tag(m,"pose","4.7 0.5 0.15 0 0 1.5707963268")
link=ET.SubElement(m,"link",name="base_link")
tag(link,"gravity","false")
inertial=ET.SubElement(link,"inertial"); tag(inertial,"mass",15)
inertia=ET.SubElement(inertial,"inertia")
for n,v in [("ixx",0.27),("iyy",0.38),("izz",0.56)]: tag(inertia,n,v)
box(link,"body","0 0 0 0 0 0","0.52 0.42 0.18","1 0.52 0.03 1",True)
box(link,"front","0.22 0 0.105 0 0 0","0.07 0.28 0.025","0.95 0.98 1 1")
for x in [-0.17,0.17]:
    for y in [-0.22,0.22]:
        v=ET.SubElement(link,"visual",name=f"wheel_{x}_{y}")
        tag(v,"pose",f"{x} {y} -0.055 1.5707963 0 0")
        cyl=ET.SubElement(ET.SubElement(v,"geometry"),"cylinder")
        tag(cyl,"radius",0.085); tag(cyl,"length",0.045)
        mat=ET.SubElement(v,"material"); tag(mat,"ambient","0.05 0.06 0.07 1"); tag(mat,"diffuse","0.05 0.06 0.07 1")
box(link,"lidar_housing","0 0 0.19 0 0 0","0.07 0.07 0.08","0.08 0.1 0.12 1")
sensor=ET.SubElement(link,"sensor",name="lidar",type="gpu_lidar")
tag(sensor,"pose","0 0 0.25 0 0 0"); tag(sensor,"topic","/scan")
tag(sensor,"update_rate",10); tag(sensor,"always_on","true"); tag(sensor,"visualize","false")
lidar=ET.SubElement(sensor,"lidar"); scan=ET.SubElement(lidar,"scan")
horizontal=ET.SubElement(scan,"horizontal")
for n,v in [("samples",360),("resolution",1),("min_angle",-math.pi),("max_angle",math.pi)]: tag(horizontal,n,v)
ran=ET.SubElement(lidar,"range")
for n,v in [("min",0.08),("max",16),("resolution",0.01)]: tag(ran,n,v)
noise=ET.SubElement(lidar,"noise")
tag(noise,"type","gaussian")
tag(noise,"mean",0); tag(noise,"stddev",0.01)
p=ET.SubElement(m,"plugin",filename="ignition-gazebo-velocity-control-system",name="ignition::gazebo::systems::VelocityControl")
tag(p,"topic","/model/omni_robot/cmd_vel")
p=ET.SubElement(m,"plugin",filename="ignition-gazebo-odometry-publisher-system",name="ignition::gazebo::systems::OdometryPublisher")
for n,v in [("odom_frame","odom"),("robot_base_frame","base_link"),("odom_publish_frequency",50),("odom_topic","/gazebo/odometry"),("dimensions",2)]: tag(p,n,v)
args.world.parent.mkdir(parents=True, exist_ok=True)
ET.indent(sdf)
ET.ElementTree(sdf).write(args.world,encoding="unicode",xml_declaration=True)
