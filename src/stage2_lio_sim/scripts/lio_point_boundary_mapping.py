#!/usr/bin/env python3
"""Opt-in isolated Spark executable; never replaces the original library."""
import os
import sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix


def main():
    directory=Path(get_package_prefix('stage2_lio_sim'))/'lib/lio_point_boundary'
    binary=directory/'spark_lio_mapping'
    component=directory/'libspark_lio_component.so'
    if not binary.is_file() or not component.is_file():
        raise RuntimeError('Install the explicit point-boundary profile first; no fallback to original binary.')
    env=dict(os.environ)
    env['LD_LIBRARY_PATH']=str(directory)+':'+env.get('LD_LIBRARY_PATH','')
    os.execve(str(binary),[str(binary),*sys.argv[1:]],env)


if __name__=='__main__':main()
