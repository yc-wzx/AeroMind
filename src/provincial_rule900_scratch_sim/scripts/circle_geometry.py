"""Conservative 0.60m full-circle envelope for the isolated training variant.

This is planar sampled-model geometry. Sensor freshness, reserve, prediction
intervals and safety thresholds remain owned by the qualified controllers.
"""
from pathlib import Path
from functools import lru_cache
import json
import math
import numpy as np

CONTRACT_PATH=Path(__file__).with_name('circle_contract.json')
CONTRACT=json.loads(CONTRACT_PATH.read_text())
RADIUS=float(CONTRACT['conservative_diameter_m'])/2
EQUALITY_GUARD_M=1e-10
if RADIUS!=.30 or CONTRACT['shape']!='circle' or CONTRACT['user_declared_maximum_diameter_m']>.60:
    raise ValueError('Unvalidated circular envelope contract')


def finite(values):
    return all(isinstance(v,(int,float,np.number)) and not isinstance(v,(bool,np.bool_)) and math.isfinite(v) for v in values)


@lru_cache(maxsize=16)
def _wall_arrays(key):
    wall=np.asarray(key,dtype=float)
    if wall.ndim!=3 or wall.shape[1:]!=(4,2) or not np.isfinite(wall).all() or not len(wall):
        raise ValueError('Finite nonempty four-vertex wall polygons required')
    edge=np.roll(wall,-1,axis=1)-wall
    norm=np.sum(edge*edge,axis=2)
    if np.any(norm<=0):raise ValueError('Degenerate wall')
    turns=np.cross(edge,np.roll(edge,-1,axis=1))
    if not np.all(np.all(turns>0,axis=1)|np.all(turns<0,axis=1)):
        raise ValueError('Strictly convex wall polygons required')
    return wall,edge,norm


def _key(walls):return tuple(tuple(tuple(p) for p in w) for w in walls)


def circle_gaps(centers,walls):
    xy=np.asarray(centers,dtype=float)
    if xy.ndim!=2 or xy.shape[1]!=2 or not len(xy) or not np.isfinite(xy).all():raise ValueError('Finite centers required')
    wall,edge,norm=_wall_arrays(_key(walls))
    delta=xy[:,None,None,:]-wall[None,:,:,:]
    projection=np.clip(np.sum(delta*edge[None,:,:,:],axis=3)/norm[None,:,:],0.,1.)
    distance=np.linalg.norm(delta-projection[:,:,:,None]*edge[None,:,:,:],axis=3).min(axis=2)
    side=edge[None,:,:,0]*delta[:,:,:,1]-edge[None,:,:,1]*delta[:,:,:,0]
    inside=np.all(side>=-1e-12,axis=2)|np.all(side<=1e-12,axis=2)
    signed=np.where(inside,-distance,distance)-RADIUS
    return signed.min(axis=1)


def body_wall_gap(x,y,yaw,walls):
    if not finite([x,y,yaw]):raise ValueError('Nonfinite pose')
    return float(circle_gaps([[x,y]],walls)[0])


def centers_for_command(pose,command,horizon=.25,step=.025):
    if len(pose)!=3 or len(command)!=3 or not finite([*pose,*command,horizon,step]) or min(horizon,step)<=0:
        raise ValueError('Invalid prediction input')
    x,y,yaw=pose;vx,vy,wz=command
    count=max(1,math.ceil(horizon/step));dt=horizon/count;centers=[(x,y)]
    for _ in range(count):
        mid=yaw+wz*dt/2
        x+=(math.cos(mid)*vx-math.sin(mid)*vy)*dt
        y+=(math.sin(mid)*vx+math.cos(mid)*vy)*dt
        yaw+=wz*dt;centers.append((x,y))
    return centers


def predict_command_gap(x,y,yaw,vx,vy,wz,walls,horizon=.25,step=.025):
    return float(np.min(circle_gaps(centers_for_command((x,y,yaw),(vx,vy,wz),horizon,step),walls)))


class CirclePlanarGapPredictor:
    def __init__(self,pose,walls,horizon=.25,step=.025):
        if len(pose)!=3 or not finite([*pose,horizon,step]) or min(horizon,step)<=0:raise ValueError('Invalid planar pose')
        self.pose=pose;self.walls=walls;self.horizon=horizon;self.step=step
        _wall_arrays(_key(walls))
    def __call__(self,vx,vy):
        return predict_command_gap(*self.pose,vx,vy,0.,self.walls,self.horizon,self.step)


def path_minimum_gap(points,yaw,yaw_error,walls,thresholds):
    if not points or not finite([yaw,yaw_error,*thresholds]):raise ValueError('Invalid local path')
    xy=[(p.x,p.y) for p in points]
    if not all(finite(p) for p in xy):raise ValueError('Nonfinite local path')
    minimum=math.inf
    for i in range(0,len(xy),64):minimum=min(minimum,float(np.min(circle_gaps(xy[i:i+64],walls))))
    return minimum


def sensor_command_gap(points,scan_pose,current_pose,command,horizon=.25,step=.025):
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=2 or not len(points) or not np.isfinite(points).all() or not finite(scan_pose):
        raise ValueError('Invalid sensor input')
    sx,sy,angle=scan_pose;c,s=math.cos(angle),math.sin(angle)
    world=points@np.array([[c,s],[-s,c]])+[sx,sy]
    centers=np.asarray(centers_for_command(current_pose,command,horizon,step))
    return float(np.linalg.norm(world[None,:,:]-centers[:,None,:],axis=2).min()-RADIUS)


def circle_in_zone(x,y,bounds):
    if len(bounds)!=4 or not finite([x,y,*bounds]):return False
    x0,y0,x1,y1=bounds
    return min(x-x0,x1-x,y-y0,y1-y)>RADIUS+EQUALITY_GUARD_M


def circle_crossed(checkpoint,previous,current):
    """Whole circle through a finite directional checkline, sampled only."""
    if previous is None:return False
    a,b=checkpoint['segment'];nx,ny=checkpoint['normal']
    if not finite([*a,*b,nx,ny,*previous,*current]):return False
    dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy);normal=math.hypot(nx,ny)
    if min(length,normal)<=0 or abs(nx*dx+ny*dy)>1e-9:return False
    before=nx*(previous[0]-a[0])+ny*(previous[1]-a[1])-RADIUS*normal
    after=nx*(current[0]-a[0])+ny*(current[1]-a[1])-RADIUS*normal
    tangent=((current[0]-a[0])*dx+(current[1]-a[1])*dy)/length
    return (before<=EQUALITY_GUARD_M*normal and after>EQUALITY_GUARD_M*normal
            and nx*(current[0]-previous[0])+ny*(current[1]-previous[1])>EQUALITY_GUARD_M*normal
            and RADIUS+EQUALITY_GUARD_M<tangent<length-RADIUS-EQUALITY_GUARD_M)
