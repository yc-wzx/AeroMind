"""Frozen route plus planner-only occupied-cell connectivity; no ROS actions."""
import json
import math
import sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'src/stage2_lio_sim/scripts'), str(ROOT/'tools')]
from lio_safety_geometry import planning_envelope
from run_provincial_forward_integration import offline_leg, FIELD, MAP, GridRoute, sdf_walls

field = json.loads(FIELD.read_text())
original = offline_leg((4.7, .5), (8.7, 4.25), GridRoute(str(MAP), clearance=.4),
                       sdf_walls(), field['reference_route'])
cloud = np.asarray(planning_envelope(field['collision_segments']), dtype=np.float32)
occupied = set()
# Same float32 cloud ABI, double additions, origin and ceil(.22/.05) as EGO.
for p in cloud:
    for dx in range(-5, 6):
        for dy in range(-5, 6):
            occupied.add((math.floor((float(p[0])+dx*.05+10)*20),
                          math.floor((float(p[1])+dy*.05+10)*20)))
samples = []
for a, b in zip(field['reference_route'], field['reference_route'][1:]):
    count = math.ceil(math.dist(a, b)/.025)
    samples += [(a[0]+(b[0]-a[0])*i/count, a[1]+(b[1]-a[1])*i/count)
                for i in range(count+1)]
blocked = [p for p in samples if (math.floor((p[0]+10)*20),
                                math.floor((p[1]+10)*20)) in occupied]
result = {'pass': not blocked, 'original_route_preflight': original,
          'planner_wall_padding_m': .075, 'EGO_inflation_m': .22,
          'EGO_resolution_m': .05, 'cloud_points': len(cloud),
          'reference_samples': len(samples), 'occupied_reference_samples': blocked,
          'scope': 'Software planning envelope only; physical world/PGM unchanged. Not a proof of a future optimized spline.'}
path = ROOT/'tools/results/stage2_step4_lio_safe_profile_20261002/offline_preflight.json'
with path.open('x') as f: json.dump(result, f, indent=2)
print(json.dumps(result, indent=2))
raise SystemExit(0 if result['pass'] else 2)
