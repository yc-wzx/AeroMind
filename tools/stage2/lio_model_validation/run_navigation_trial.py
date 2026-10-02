import runpy
from pathlib import Path
import sys
p=Path(__file__).resolve().parents[1]/'lio'/'run_navigation_trial.py'
sys.path.insert(0,str(p.parent))
runpy.run_path(str(p),run_name='__main__')
