"""Reproducible opt-in LIO copy; original source and binary remain sealed."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools/stage2/lio_replay'))
from make_diagnostic_source import prepare


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    prepare(output, False)  # Original equations, plus read-only state diagnostics.
    changes = []
    for name in ['include/spark_fast_lio.h', 'src/spark_fast_lio.cpp']:
        path = output/name
        old = path.read_text()
        if name.endswith('.h'):
            anchor = '  double planar_height_ = 0.0;'
            replacement = anchor+'\n  double laser_point_covariance_ = LASER_POINT_COV;'
        else:
            anchor = '  planar_height_ = declare_parameter<double>("common.planar_height", 0.0);'
            replacement = anchor+'''
  // Preserve the original value by default. Simulation candidates must be
  // explicit; radial sensor variance is not a calibrated map error model.
  laser_point_covariance_ = declare_parameter<double>("mapping.laser_point_covariance", LASER_POINT_COV);
  if (!std::isfinite(laser_point_covariance_) || laser_point_covariance_ <= 0.0) {
    throw std::invalid_argument("mapping.laser_point_covariance must be finite and positive");
  }
  RCLCPP_INFO(this->get_logger(), "POINT_VARIANCE_ACTUAL %.17g", laser_point_covariance_);
'''
        assert old.count(anchor) == 1
        new = old.replace(anchor, replacement)
        if name.endswith('.cpp'):
            anchor = 'kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);'
            assert new.count(anchor) == 1
            new = new.replace(anchor, 'kf_.update_iterated_dyn_share_modified(laser_point_covariance_, solve_H_time);')
        path.write_text(new)
        changes.append(''.join(difflib.unified_diff(old.splitlines(True), new.splitlines(True), fromfile=name, tofile=name)))
    (output/'point_variance_changes.patch').write_text(''.join(changes))
    manifest = {'original_default': .001, 'candidate': .0001,
        'scope': 'Only optional observation variance and read-only diagnostics; no planar projection, GT, planner or control changes.',
        'files': {str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in output.rglob('*') if p.is_file()}}
    with (output/'candidate_manifest.json').open('x') as stream:
        json.dump(manifest, stream, indent=2)


if __name__ == '__main__':
    main()
