#!/usr/bin/env python3
"""Offline flat-arena bundle from measured dimensions. Never publishes or installs."""
import argparse, hashlib, json, math, subprocess, sys, time, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/uav_planning/scripts'))
from grid_route import GridRoute
from provincial_safety_geometry import body_wall_gap, rectangle, wall_box, reference_stages

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def finite(v):
    if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v):raise ValueError('Unknown/nonfinite dimension')
    return float(v)
def validate(s):
    if s.get('units')!='m' or s.get('terrain')!='flat':raise ValueError('Require metres and confirmed flat model; no automatic scaling/3D inference')
    if s.get('origin_convention')!='bottom-left-x-right-y-up':raise ValueError('Origin/axes must be explicitly confirmed')
    f=s['field']
    for k in ('width','height'):
        if finite(f['arena'][k])<=0:raise ValueError('Invalid arena dimensions')
    for zone in ('start','shooting_zone','target_zone'):
        z=f[zone]
        for k in ('x','y','width','height'):finite(z[k])
        if z['width']<=0 or z['height']<=0:raise ValueError('Invalid zone dimensions')
        if not 0<=z['x']<=f['arena']['width'] or not 0<=z['y']<=f['arena']['height']:raise ValueError('Zone outside arena')
    finite(f['start']['yaw'])
    if len(f['reference_route'])<2 or len(f['course_surfaces'])<1:raise ValueError('Missing route or traversable polygons')
    for point in f['reference_route']:
        if len(point)!=2:raise ValueError('Route point must be XY')
        for v in point:finite(v)
    if math.dist(f['reference_route'][0],[f['start']['x'],f['start']['y']])>.001 or math.dist(f['reference_route'][-1],[f['shooting_zone']['x'],f['shooting_zone']['y']])>.001:raise ValueError('Reference endpoints must match task targets')
    for a,b in zip(f['reference_route'],f['reference_route'][1:]):
        if math.dist(a,b)<=1e-6:raise ValueError('Zero reference segment')
    for rect in f['course_surfaces']:
        for k in ('x','y','width','height'):finite(rect[k])
        if rect['width']<=0 or rect['height']<=0:raise ValueError('Invalid free-space rectangle')
        if rect['x']-rect['width']/2<0 or rect['y']-rect['height']/2<0 or rect['x']+rect['width']/2>f['arena']['width']+1e-9 or rect['y']+rect['height']/2>f['arena']['height']+1e-9:raise ValueError('Free rectangle outside arena')
    if not f['collision_segments']:raise ValueError('Missing wall geometry; cannot assume obstacle-free')
    for line in f['collision_segments']:
        if len(line)!=4:raise ValueError('Wall segment requires x0,y0,x1,y1')
        for v in line:finite(v)
        if math.dist(line[:2],line[2:])<=1e-6:raise ValueError('Zero wall segment')
    # Fixed geometry and resolution are this Stage 1.5 profile, not silently tuned.
    if finite(s['wall_thickness_m'])!=.055 or finite(s['map_resolution_m'])!=.05:raise ValueError('Unsupported geometry profile: thickness=.055,resolution=.05 required')
    for k in ('width','height'):
        if abs(f['arena'][k]/.05-round(f['arena'][k]/.05))>1e-6:raise ValueError('Arena extent must align to existing .05m map grid; document any chosen padding')
    if s.get('unresolved_dimensions'):raise ValueError('Unresolved dimensions: '+str(s['unresolved_dimensions']))
    if not s.get('evidence_note'):raise ValueError('Missing dimensional source record')
    return f

def build(source,out,competition=False):
    begun=time.monotonic(); s=json.loads(Path(source).read_text());f=validate(s)
    if competition and not all(s.get('confirmations',{}).get(k) is True for k in ('scale','axes','start','shooting_zone','free_space','fixed_walls','fixed_obstacles')):raise ValueError('Competition-input confirmations incomplete; no inference or auto approval')
    if out.exists():raise FileExistsError(out)
    out.mkdir(parents=True)
    (out/'source.json').write_text(json.dumps(s,ensure_ascii=False,indent=2)+'\n')
    (out/'field.json').write_text(json.dumps(f,ensure_ascii=False,indent=2)+'\n')
    subprocess.run([sys.executable,'-B',str(ROOT/'tools/generate_competition_world.py'),'--field',str(out/'field.json'),'--world',str(out/'arena.sdf')],check=True,capture_output=True,text=True)
    # Exported artifact only: use measured dimensions and spawn, retaining body,
    # physics, sensor rate and speed-control plugin from the current baseline.
    tree=ET.parse(out/'arena.sdf');world=tree.getroot().find('world');models={m.get('name'):m for m in world.findall('model')}
    w,h=f['arena']['width'],f['arena']['height'];floor=models['arena_floor'];floor.find('pose').text=f'{w/2} {h/2} -0.05 0 0 0'
    for size in floor.findall('./link/*/geometry/box/size'):size.text=f'{w} {h} 0.1'
    z=f['start'];models['omni_robot'].find('pose').text=f"{z['x']} {z['y']} 0.15 0 0 {z['yaw']}"
    ET.indent(tree);tree.write(out/'arena.sdf',encoding='unicode',xml_declaration=True)
    rows,cols=round(h/.05),round(w/.05);X,Y=np.meshgrid((np.arange(cols)+.5)*.05,(rows-np.arange(rows)-.5)*.05)
    free=np.zeros((rows,cols),dtype=bool)
    for rect in f['course_surfaces']:
        free|=(rect['x']-rect['width']/2<=X)&(X<rect['x']+rect['width']/2)&(rect['y']-rect['height']/2<=Y)&(Y<rect['y']+rect['height']/2)
    # Rasterise physical/declared virtual collision boxes from exactly the SDF geometry.
    walls=[];geometry_ok=True
    for i,line in enumerate(f['collision_segments']):
        x0,y0,x1,y1=line;cx,cy=(x0+x1)/2,(y0+y1)/2;angle=math.atan2(y1-y0,x1-x0);length=math.dist(line[:2],line[2:])
        model=models['boundary_'+str(i)];pose=list(map(float,model.findtext('pose').split()));size=list(map(float,model.findtext('./link/collision/geometry/box/size').split()))
        geometry_ok &= math.dist(pose[:2],[cx,cy])<1e-6 and abs(pose[5]-angle)<1e-6 and abs(size[0]-length)<1e-6 and abs(size[1]-.055)<1e-6
        walls.append(wall_box(line,.055));u=(X-cx)*math.cos(angle)+(Y-cy)*math.sin(angle);v=-(X-cx)*math.sin(angle)+(Y-cy)*math.cos(angle)
        free&=~((abs(u)<=length/2)&(abs(v)<=.055/2))
    Image.fromarray(np.where(free,255,0).astype('uint8')).save(out/'arena.pgm')
    (out/'arena.yaml').write_text('image: arena.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    grid=GridRoute(out/'arena.pgm',clearance=.4);checks={'collision_matches_input':bool(geometry_ok),'pgm_raster_consistent':np.array_equal(np.asarray(Image.open(out/'arena.pgm'))==255,free),'footprint_unchanged':models['omni_robot'].findtext('./link/collision/geometry/box/size')=='0.52 0.42 0.18','spawn_matches_input':math.dist(list(map(float,ET.parse(out/'arena.sdf').findtext('./world/model[@name="omni_robot"]/pose').split()))[:2],[z['x'],z['y']])<1e-9 and abs(float(ET.parse(out/'arena.sdf').findtext('./world/model[@name="omni_robot"]/pose').split()[5])-z['yaw'])<1e-9}
    routes={}
    yaw=z['yaw']
    for label,a,b in (('forward',f['reference_route'][0],f['reference_route'][-1]),('return',f['reference_route'][-1],f['reference_route'][0])):
        try:
            grid.route(a,b);stages=reference_stages(a,b,f['reference_route']);points=[a]+list(stages);gaps=[]
            for left,right in zip(points,points[1:]):
                if min(grid.line_min_clearance(left,right),float(grid.clearance[grid.cell(*left)]),float(grid.clearance[grid.cell(*right)]))<.4:raise ValueError('Reference segment violates .40m clearance')
                steps=max(1,math.ceil(math.dist(left,right)/.025))
                gaps.extend(body_wall_gap(left[0]+(right[0]-left[0])*i/steps,left[1]+(right[1]-left[1])*i/steps,yaw,walls) for i in range(steps+1))
            if min(gaps)<.08:raise ValueError('Reference footprint violates .08m sampled gap')
            routes[label]={'pass':True,'waypoints':stages,'grid_length_m':sum(math.dist(u,v) for u,v in zip(grid.last_raw_grid_path,grid.last_raw_grid_path[1:])),'minimum_sampled_body_gap_m':min(gaps)}
        except ValueError as e:routes[label]={'pass':False,'reason':str(e)}
    # Export an explicit launch using current installed nodes/config; never auto-launch.
    text=(ROOT/'src/uav_bringup/launch/provincial_2025_provisional.launch.py').read_text()
    for old,new in (("os.path.join(bringup, 'worlds', 'provincial_2025_training.sdf')",'arena.sdf'),("os.path.join(planning, 'config', 'provincial_2025_provisional.json')",'field.json'),("os.path.join(bringup, 'maps', 'provincial_2025_provisional.pgm')",'arena.pgm')):
        if old not in text:raise ValueError('Launch template changed; review required')
        text=text.replace(old,repr(str(out/new)))
    compile(text,str(out/'arena.launch.py'),'exec');(out/'arena.launch.py').write_text(text)
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(7,7));ax.imshow(free,origin='upper',extent=(0,w,0,h),cmap='gray')
    for line in f['collision_segments']:ax.plot([line[0],line[2]],[line[1],line[3]],'r-',linewidth=2)
    route=np.array(f['reference_route']);ax.plot(route[:,0],route[:,1],'g.-');ax.set(xlabel='x (m)',ylabel='y (m)',title='Dimensions input / collision / route (review required)');ax.set_aspect('equal');fig.tight_layout();fig.savefig(out/'arena_preview.png',dpi=150);plt.close(fig)
    checks['offline_routes']=all(v['pass'] for v in routes.values());report={'all_pass':all(checks.values()),'checks':checks,'routes':routes,'classification':'COMPETITION INPUT CANDIDATE' if competition else 'TRAINING-ONLY / PROVISIONAL','competition_arena_verified':False,'no_navigation_launched':True,'machine_seconds':time.monotonic()-begun,'human_digitising_time_measured':False,'manual_preview_review_required':True,'supported':'Flat XY; union of axis-aligned free rectangles and .055m collision segments; no automatic PDF/CAD import','source_sha256':sha(source),'tool_sha256':sha(__file__),'files':{p.name:sha(p) for p in out.iterdir() if p.is_file()}}
    (out/'offline_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');return report

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output-dir',type=Path,required=True);parser.add_argument('--competition-input',action='store_true');args=parser.parse_args()
    result=build(args.source.resolve(),args.output_dir.resolve(),args.competition_input);print(json.dumps(result,ensure_ascii=False,indent=2));sys.exit(0 if result['all_pass'] else 2)
