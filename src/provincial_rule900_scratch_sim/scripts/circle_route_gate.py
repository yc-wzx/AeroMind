"""Offline circular route guard used by launch and the navigation entry point."""
import math
import numpy as np
from circle_geometry import RADIUS,body_wall_gap,circle_in_zone,CONTRACT,circle_gaps
from provincial_safety_geometry import reference_stages,wall_box,point_segment_distance


def segment_distance(a,b,c,d):
    def orient(p,q,r):return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    if orient(a,b,c)*orient(a,b,d)<0 and orient(c,d,a)*orient(c,d,b)<0:return 0.
    return min(point_segment_distance(a,c,d),point_segment_distance(b,c,d),point_segment_distance(c,a,b),point_segment_distance(d,a,b))


def grid_connectivity_precheck(grid,start,goal):
    """Cheap precheck only. Original parent still runs GridRoute.route at acceptance.

    Four-connected safe labels are the existing GridRoute connectivity labels;
    this neither fabricates a path nor replaces the parent's actual A* call.
    """
    source,target=grid.cell(*start),grid.cell(*goal)
    if source is None or target is None or not grid.safe[source] or not grid.safe[target]:
        raise ValueError('Start or goal is outside the free clearance area')
    if grid.component_labels[source]!=grid.component_labels[target]:
        raise ValueError('start and goal are in different static connected components')


def route_preflight(start,goal,grid,walls,reference,minimum_gap=.08,pose_margin=.04,require_explicit_grid_route=True):
    if not all(math.isfinite(v) for v in [*start,*goal,minimum_gap,pose_margin]) or minimum_gap!=.08 or pose_margin!=.04:
        raise ValueError('Invalid input or changed frozen thresholds')
    # Preserve the original production start/free/component/A* prerequisites.
    check=grid.route if require_explicit_grid_route else lambda a,b:grid_connectivity_precheck(grid,a,b)
    check(start,goal)
    stages=reference_stages(start,goal,reference);points=[tuple(start)]+stages
    required=minimum_gap+pose_margin;legs=[]
    # Same exact straight-segment/convex-edge geometry, evaluated as arrays.
    # Keep all route/free/component calls and endpoint-inside checks. This is
    # guard computation only; no freshness threshold or planner change.
    w=np.asarray(walls,dtype=float);c=w.reshape(-1,2)
    d=np.roll(w,-1,axis=1).reshape(-1,2);edge=d-c
    edge_norm=np.sum(edge*edge,axis=1)
    if np.any(edge_norm<=0):raise ValueError('Degenerate wall')
    endpoint_gaps=circle_gaps(points,walls)
    def cross(x,y):return x[...,0]*y[...,1]-x[...,1]*y[...,0]
    def endpoint_edge(point):
        delta=np.asarray(point)-c
        ratio=np.clip(np.sum(delta*edge,axis=1)/edge_norm,0.,1.)
        return np.linalg.norm(delta-ratio[:,None]*edge,axis=1)
    for i,(a,b) in enumerate(zip(points,points[1:])):
        check(a,b)
        av=np.asarray(a);bv=np.asarray(b);v=bv-av;length2=float(np.sum(v*v))
        if length2==0:
            minimum=min(endpoint_gaps[i],endpoint_gaps[i+1])
        else:
            def vertex_segment(vertices):
                delta=vertices-av;t=np.clip(np.sum(delta*v,axis=1)/length2,0.,1.)
                return np.linalg.norm(delta-t[:,None]*v,axis=1)
            distances=np.minimum.reduce([endpoint_edge(a),endpoint_edge(b),vertex_segment(c),vertex_segment(d)])
            intersects=(cross(v,c-av)*cross(v,d-av)<0)&(cross(edge,av-c)*cross(edge,bv-c)<0)
            distances=np.where(intersects,0.,distances)
            minimum=min(float(distances.min())-RADIUS,endpoint_gaps[i],endpoint_gaps[i+1])
        legs.append({'start':a,'goal':b,'nominal_circle_gap_m':float(minimum),
                     'required_static_reserved_gap_m':required,'pass':bool(minimum>=required)})
    passed=bool(legs) and all(r['pass'] for r in legs)
    return {'grid_path_check':'explicit GridRoute.route' if require_explicit_grid_route else 'connectivity precheck; original parent MUST still call GridRoute.route at acceptance','pass':passed,'shape':'circle','radius_m':RADIUS,'start':start,'goal':goal,
            'reference_stages':stages,'legs':legs,
            'minimum_circle_gap_m':min(r['nominal_circle_gap_m'] for r in legs),
            'minimum_body_gap_m':minimum_gap,'pose_margin_m':pose_margin,
            'required_static_gap_m':required,'motion_reserve_additional':True,
            'reason':None if passed else 'GOAL REJECTED: CIRCLE ENVELOPE RESERVED CLEARANCE',
            'nominal_body_gap_only_pass':all(r['nominal_circle_gap_m']>=minimum_gap for r in legs),
            'navigation_goal_may_be_sent':passed,'competition_arena_verified':False}
