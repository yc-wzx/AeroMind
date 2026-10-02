"""Buildable isolated LIO copy: read-only diagnostics, optional iterator fix."""
import argparse,shutil,hashlib,json,difflib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]

def prepare(out,fix):
    if out.exists():raise FileExistsError(out)
    original=ROOT/'src/third_party/spark-fast-lio/spark_fast_lio'
    shutil.copytree(original,out,ignore=shutil.ignore_patterns('__pycache__','Log','PCD'))
    (out/'Log').mkdir(exist_ok=True)
    cpp=out/'src/spark_fast_lio.cpp';old=cpp.read_text();text=old
    text=text.replace('#include <cmath>','#include <cmath>\n#include <Eigen/Eigenvalues>')
    at='  solve_time_ += omp_get_wtime() - solve_start;'
    diagnostic='''  // Offline diagnostic only: no change to measurement or state.
  const Eigen::Matrix3d translation_gram = ekfom_data.h_x.leftCols(3).transpose() * ekfom_data.h_x.leftCols(3);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(translation_gram);
  const auto values=eig.eigenvalues();
  const auto weak=eig.eigenvectors().col(0);
  RCLCPP_INFO(this->get_logger(), "OFFLINE_H t=%.9f count=%d eigen=%.12g,%.12g,%.12g diag=%.12g,%.12g,%.12g weak=%.12g,%.12g,%.12g", Measures_.lidar_end_time, effect_feat_num_, values(0),values(1),values(2),translation_gram(0,0),translation_gram(1,1),translation_gram(2,2),weak(0),weak(1),weak(2));
'''
    assert text.count(at)==1;text=text.replace(at,diagnostic+at)
    at='  kf_for_preintegration_ = kf_;'
    diagnostic='''  const auto debug_state=kf_.get_x();
  RCLCPP_INFO(this->get_logger(), "OFFLINE_STATE t=%.9f pos=%.12g,%.12g,%.12g vel=%.12g,%.12g,%.12g ba=%.12g,%.12g,%.12g bg=%.12g,%.12g,%.12g grav=%.12g,%.12g,%.12g imu_end=%.9f imu_count=%zu", Measures.lidar_end_time,debug_state.pos(0),debug_state.pos(1),debug_state.pos(2),debug_state.vel(0),debug_state.vel(1),debug_state.vel(2),debug_state.ba(0),debug_state.ba(1),debug_state.ba(2),debug_state.bg(0),debug_state.bg(1),debug_state.bg(2),debug_state.grav[0],debug_state.grav[1],debug_state.grav[2],rclcpp::Time(Measures.imu.back()->header.stamp).seconds(),Measures.imu.size());
'''
    assert text.count(at)==1;text=text.replace(at,at+'\n'+diagnostic);cpp.write_text(text)
    patches=[''.join(difflib.unified_diff(old.splitlines(True),text.splitlines(True),fromfile='src/spark_fast_lio.cpp',tofile='src/spark_fast_lio.cpp'))]
    if fix:
        header=out/'include/imu_processing.hpp';old=header.read_text();at='      if (it_pcl == pcl_out.points.begin()) break;'
        assert old.count(at)==1
        new=old.replace(at,'      // All points consumed: do not transform the first point in older IMU intervals.\n      if (it_pcl == pcl_out.points.begin()) return;')
        header.write_text(new);patches.append(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='include/imu_processing.hpp',tofile='include/imu_processing.hpp')))
    (out/'offline_changes.patch').write_text(''.join(patches))
    files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in out.rglob('*') if p.is_file()}
    (out/'offline_manifest.json').write_text(json.dumps({'source':str(original),'isolated_copy':True,'navigation_modified':False,'first_point_fix':fix,'files':files},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--fix-first-point',action='store_true');a=p.parse_args();prepare(a.output,a.fix_first_point)
