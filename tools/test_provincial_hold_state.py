"""Exercise the production update() body with isolated state and publishers."""

import ast
import math
import time
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'src/uav_planning/scripts/gazebo_navigation_interface.py'


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


def load_production_update():
    tree = ast.parse(SOURCE.read_text())
    node = next(item for cls in tree.body if isinstance(cls, ast.ClassDef)
                for item in cls.body if isinstance(item, ast.FunctionDef)
                and item.name == 'update')
    namespace = {
        'math': math, 'time': time, 'Twist': Twist,
        'timed_callback': lambda method: method,
        'predict_command_gap': lambda *args: 0.20,
    }
    module = ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[]))
    exec(compile(module, str(SOURCE), 'exec'), namespace)
    return namespace['update']


UPDATE = load_production_update()


def drive_case(*, route_id, active=False, pending=False, plan=True,
               odom_fresh=True, command_speed=0.10):
    now = time.monotonic()
    command = Twist()
    command.linear.x = command_speed
    output = []
    diagnostics = []
    target = types.SimpleNamespace(pose=types.SimpleNamespace(
        position=types.SimpleNamespace(x=4.7, y=1.15))) if active else None
    state = types.SimpleNamespace(
        command=command,
        provincial_reference_route_mode=True,
        route_diag_id=route_id,
        active_waypoint=target,
        waypoints=[object()] if pending else [],
        grid_route=types.SimpleNamespace(clearance_required=0.40,
                                         line_safe=lambda *args, **kwargs: True),
        stage_sent_at=now,
        final_approach_active=False,
        last_command_time=now,
        command_timeout=0.5,
        last_odom_received=now if odom_fresh else now - 2.0,
        scan_received_at=now,
        last_drive_clock=None,
        imperfect_sensors=False,
        drive_tau=0.0,
        drive_scale=1.0,
        actuated=Twist(),
        provincial_rect_guard=True,
        planned_path_valid_until=2.0 if plan else None,
        planned_path_safe=True if plan else None,
        provincial_walls=[],
        provincial_min_body_gap=0.08,
        x=4.7, y=0.5, yaw=math.pi / 2,
        latest_odom=None,
        get_clock=lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(nanoseconds=1_000_000_000)),
        publish_actuation_diagnostic=lambda *args: diagnostics.append(args),
        drive_pub=types.SimpleNamespace(publish=lambda value: output.append(
            (value.linear.x, value.linear.y, value.angular.z))),
    )
    UPDATE(state)
    return output[-1], diagnostics[-1]


class HoldStateTests(unittest.TestCase):
    def test_pending_without_active_stops_stale_spline(self):
        self.assertEqual(drive_case(route_id='R0001', pending=True)[0],
                         (0.0, 0.0, 0.0))

    def test_completed_route_stops_stale_spline(self):
        self.assertEqual(drive_case(route_id='R0001')[0],
                         (0.0, 0.0, 0.0))

    def test_initial_idle_stops_fresh_stray_command(self):
        self.assertEqual(drive_case(route_id=None)[0],
                         (0.0, 0.0, 0.0))

    def test_new_active_goal_waits_for_fresh_safe_plan(self):
        self.assertEqual(drive_case(route_id='R0002', active=True,
                                    plan=False)[0], (0.0, 0.0, 0.0))

    def test_new_active_goal_with_fresh_safe_plan_can_move(self):
        self.assertEqual(drive_case(route_id='R0002', active=True)[0],
                         (0.10, 0.0, 0.0))

    def test_stale_odometry_stops_immediately(self):
        self.assertEqual(drive_case(route_id='R0002', active=True,
                                    odom_fresh=False)[0],
                         (0.0, 0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
