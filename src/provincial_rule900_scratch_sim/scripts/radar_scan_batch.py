"""Batch every original static ray sample; no change to occupancy decisions."""
import math
import numpy as np

def ray_answers(grid, origin, angles, distances, margin=.60):
    if len(angles)!=len(distances):raise ValueError('ray lengths differ')
    if not all(math.isfinite(x) for x in (*origin,margin,grid.resolution,*angles,*distances)):
        raise ValueError('nonfinite ray query')
    step=grid.resolution*.5
    limits=np.asarray([d+margin for d in distances],dtype=float)
    if step<=0 or np.any(limits<0):raise ValueError('invalid ray sampling extent')
    counts=np.asarray([int(math.ceil(float(x)/step)) for x in limits],dtype=np.int64)
    answers=[]
    for start in range(0,len(angles),64):
        end=min(start+64,len(angles));n=counts[start:end]
        indices=np.arange(1,int(n.max())+1,dtype=float)
        active=indices[None,:]<=n[:,None]
        radius=np.minimum(indices[None,:]*step,limits[start:end,None])
        # math.cos/sin match the scalar production function exactly.
        c=np.asarray([math.cos(a) for a in angles[start:end]])[:,None]
        s=np.asarray([math.sin(a) for a in angles[start:end]])[:,None]
        x=origin[0]+radius*c;y=origin[1]+radius*s
        cols=np.floor((x-grid.origin_x)/grid.resolution).astype(np.int64)
        rows=grid.height-1-np.floor((y-grid.origin_y)/grid.resolution).astype(np.int64)
        inside=(rows>=0)&(rows<grid.height)&(cols>=0)&(cols<grid.width)
        # Padding never contributes. Clipping is only for safe array access;
        # any actual out-of-map cell still makes the original predicate False.
        cells=grid.pixels[np.clip(rows,0,grid.height-1),np.clip(cols,0,grid.width-1)]
        valid=inside&(cells>=250)
        answers.extend(bool(v) for v in np.all(~active|valid,axis=1))
    return answers

def filter_returns(grid,origin,points,angles,distances):
    if not len(points)==len(angles)==len(distances):raise ValueError('scan lengths differ')
    near=[grid.near_static_obstacle(p[0],p[1],tolerance=.60) for p in points]
    selected=[i for i,v in enumerate(near) if v]
    answers=ray_answers(grid,origin,[angles[i] for i in selected],[distances[i] for i in selected])
    planner=[p for p,n in zip(points,near) if not n]
    dynamic=[points[i] for i,v in zip(selected,answers) if v]
    return planner,dynamic
