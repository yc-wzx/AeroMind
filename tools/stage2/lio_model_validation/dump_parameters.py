#!/usr/bin/env python3
"""Read parameters using the preserved, bounded step-four reader."""
import runpy
from pathlib import Path

if __name__ == '__main__':
    runpy.run_path(str(Path(__file__).resolve().parents[1] / 'lio' / 'dump_parameters.py'),
                   run_name='__main__')
