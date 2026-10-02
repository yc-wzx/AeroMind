"""One unchanged world/bridge/relay/planner; localization waits for capture readiness."""
from pathlib import Path
import runpy
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    helper=Path(get_package_share_directory('uav_bringup'))/'launch/stage2_scan_startup_split.py'
    return runpy.run_path(str(helper))['split_description'](False)
