"""Verify the private portable dependency directory; no external overlays."""
from pathlib import Path
_loaded=False
def load_existing():
    global _loaded
    if _loaded:return
    directory=Path(__file__).resolve().parent
    for name in ('gimbal_contract.py','gazebo_navigation_interface.py','lio_contract.py','grid_route.py'):
        if not (directory/name).is_file():raise RuntimeError('Missing packaged dependency: '+str(directory/name))
    _loaded=True
