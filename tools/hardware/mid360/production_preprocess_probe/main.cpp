#include "preprocess.h"
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

template<class T> void store(std::vector<uint8_t>&data,size_t offset,T value) {
  std::memcpy(data.data()+offset,&value,sizeof(T));
}

int main() {
  sensor_msgs::msg::PointCloud2 message;
  message.header.stamp.sec=100; message.header.frame_id="livox_frame";
  message.height=1; message.width=3; message.point_step=26; message.row_step=78;
  const char* names[]={"x","y","z","intensity","tag","line","timestamp"};
  const uint32_t offsets[]={0,4,8,12,16,17,18};
  const uint8_t types[]={7,7,7,7,2,2,8};
  for(int i=0;i<7;++i) {
    sensor_msgs::msg::PointField field;
    field.name=names[i]; field.offset=offsets[i]; field.datatype=types[i]; field.count=1;
    message.fields.push_back(field);
  }
  message.data.resize(message.row_step);
  const double offsets_ns[]={0.,50000000.,99000000.};
  for(int i=0;i<3;++i) {
    auto base=i*26;
    store(message.data,base,float(1+i)); store(message.data,base+4,float(2+i));
    store(message.data,base+8,float(-3-i)); store(message.data,base+12,float(20+i));
    store(message.data,base+16,uint8_t(0x10)); store(message.data,base+17,uint8_t(i));
    store(message.data,base+18,100e9+offsets_ns[i]);
  }
  Preprocess actual;
  actual.lidar_type=MID360; actual.N_SCANS=4; actual.blind=.5;
  actual.point_filter_num=1; actual.feature_enabled=false; actual.time_unit=NS;
  PointCloudXYZI::Ptr output(new PointCloudXYZI);
  actual.process(message,output);
  if(output->size()!=3) throw std::runtime_error("Production MID360 fields/line mapping failed");
  for(int i=0;i<3;++i) {
    const auto&p=output->at(i);
    if(p.x!=float(1+i) || p.y!=float(2+i) || p.z!=float(-3-i) ||
       std::abs(p.curvature-offsets_ns[i]*1e-6)>.001)
      throw std::runtime_error("Production point coordinates or relative millisecond timing changed");
  }
  std::cout<<"PASS: actual SPARK Preprocess::process MID360 enum5; 3 raw XYZ preserved; per-point offsets 0/50/99ms; no instantaneous simulation override\n";
}
