import ast,json,math,sys,time,types,cProfile,pstats
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/uav_planning/scripts'))
from provincial_safety_geometry import body_wall_gap
from summarize_provincial_stage15_terminal_v3 import sdf_walls
p=ROOT/'tools/results/provincial_stage15_completion_20261001/round_trip_04/round_trip_04_forward'
rows=[json.loads(l) for l in (p/(p.name+'.native.jsonl')).read_text().splitlines()]
row=next(x for x in rows if x['topic']=='/ground/planning/planned_trajectory' and abs(x['message_stamp_s']-54.216)<.01)
def obj(v):return types.SimpleNamespace(**{k:obj(x) if isinstance(x,dict) else [obj(y) if isinstance(y,dict) else y for y in x] if isinstance(x,list) else x for k,x in v.items()})
marker=obj(row['data']);source=ROOT/'src/uav_planning/scripts/gazebo_navigation_interface.py';tree=ast.parse(source.read_text());node=next(n for c in tree.body if isinstance(c,ast.ClassDef) for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='planned_trajectory_callback')
ns={'timed_callback':lambda f:f,'math':math,'time':time,'body_wall_gap':body_wall_gap,'stamp_seconds':lambda s:s.sec+s.nanosec*1e-9,'wrap_angle':lambda a:math.atan2(math.sin(a),math.cos(a))};exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(source),'exec'),ns)
s=types.SimpleNamespace(provincial_rect_guard=True,active_waypoint=obj({'header':{'stamp':{'sec':54,'nanosec':212000000}},'pose':{'orientation':{'w':math.cos(math.pi/4),'z':math.sin(math.pi/4),'x':0.,'y':0.}}}),yaw=math.pi/2,provincial_walls=sdf_walls(),provincial_min_body_gap=.08)
times=[]
for _ in range(5):
 t=time.perf_counter();ns['planned_trajectory_callback'](s,marker);times.append(time.perf_counter()-t)
print(json.dumps({'production_callback_seconds':times,'points':len(marker.points),'wall_count':len(s.provincial_walls),'min_gap_m':s.planned_path_min_gap,'safe':s.planned_path_safe}))
