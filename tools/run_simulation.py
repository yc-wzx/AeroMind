#!/usr/bin/env python3
"""Own the whole simulation process group and prevent duplicate launches."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time

CACHE = Path.home() / ".cache" / "aeromind-simulation"
CACHE.mkdir(parents=True, exist_ok=True)
PID_FILE = CACHE / "process.json"


def stop():
    if not PID_FILE.exists():
        print("No managed Gazebo simulation is running.")
        return
    record = json.loads(PID_FILE.read_text())
    pid = int(record["pid"])
    try:
        command = Path(f"/proc/{pid}/cmdline").read_bytes()
        if (not any(name in command for name in
                    (b"competition_gazebo.launch.py", b"rmuc_gazebo.launch.py"))
                or os.getpgid(pid) != pid):
            raise RuntimeError("PID no longer belongs to this simulation; refusing to stop it")
        os.killpg(pid, signal.SIGINT)
    except ProcessLookupError:
        return
    except FileNotFoundError:
        return
    deadline = time.monotonic() + 12
    while Path(f"/proc/{pid}").exists() and time.monotonic() < deadline:
        time.sleep(0.1)
    print("Gazebo simulation stop requested.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stop", action="store_true")
    parser.add_argument("--world", choices=("competition", "rmuc"), default="competition")
    args, extra = parser.parse_known_args()
    if args.stop:
        stop()
        return
    with (CACHE / "lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("Simulation is already running. Open Gazebo/RViz from the taskbar.")
            return
        root = Path(__file__).resolve().parents[1]
        log_path = CACHE / "latest.log"
        with log_path.open("w") as log:
            child = subprocess.Popen(
                ["ros2", "launch", "uav_bringup",
                 f"{args.world}_gazebo.launch.py" if args.world == "rmuc"
                 else "competition_gazebo.launch.py", *extra],
                cwd=root, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            PID_FILE.write_text(json.dumps({"pid":child.pid}))
            def forward(signum, frame):
                try:
                    os.killpg(child.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            signal.signal(signal.SIGINT, forward)
            signal.signal(signal.SIGTERM, forward)
            print(f"Gazebo + navigation + RViz started. Log: {log_path}", flush=True)
            try:
                return_code = child.wait()
            finally:
                forward(signal.SIGINT, None)
                PID_FILE.unlink(missing_ok=True)
        if return_code:
            print(log_path.read_text()[-6000:])
            raise SystemExit(return_code)


if __name__ == "__main__":
    main()
