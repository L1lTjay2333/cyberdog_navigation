// Bounded SDK diagnostic, no ROS publishers or robot motion interfaces.
#include <src/CYdLidar.h>
#include <cmath>
#include <iostream>
#include <chrono>
template<class T> void option(CYdLidar& lidar,int key,T value) {
  if(!lidar.setlidaropt(key,&value,sizeof(value))) throw std::runtime_error("SDK option rejected");
}
int main() {
  CYdLidar lidar;
  try {
    const std::string port="/dev/ttyTHS0",ignore="";
    lidar.setlidaropt(LidarPropSerialPort,port.c_str(),port.size());
    lidar.setlidaropt(LidarPropIgnoreArray,ignore.c_str(),ignore.size());
    option(lidar,LidarPropSerialBaudrate,512000);
    option(lidar,LidarPropLidarType,0);option(lidar,LidarPropDeviceType,0);
    option(lidar,LidarPropSampleRate,10);option(lidar,LidarPropAbnormalCheckCount,4);
    option(lidar,LidarPropFixedResolution,false);option(lidar,LidarPropReversion,true);
    option(lidar,LidarPropInverted,false);option(lidar,LidarPropAutoReconnect,false);
    option(lidar,LidarPropSingleChannel,false);option(lidar,LidarPropIntenstiy,true);
    option(lidar,LidarPropSupportMotorDtrCtrl,false);
    option(lidar,LidarPropMaxAngle,90.f);option(lidar,LidarPropMinAngle,-90.f);
    option(lidar,LidarPropMaxRange,30.f);option(lidar,LidarPropMinRange,.01f);
    option(lidar,LidarPropScanFrequency,10.f);
    if(!lidar.initialize())throw std::runtime_error(lidar.DescribeError());
    if(!lidar.turnOn())throw std::runtime_error(lidar.DescribeError());
    int frames=0;auto begun=std::chrono::steady_clock::now();
    while(frames<15&&std::chrono::steady_clock::now()-begun<std::chrono::seconds(12)) {
      LaserScan scan;
      if(!lidar.doProcessSimple(scan))continue;
      std::cout<<"RAW_JSON {\"stamp_ns\":"<<scan.stamp<<",\"frame\":"<<frames++<<",\"points\":[";
      bool first=true;
      for(const auto& p:scan.points) {
        if(!first)std::cout<<',';first=false;
        std::cout<<'[';
        if(std::isfinite(p.angle))std::cout<<p.angle;else std::cout<<"null";
        std::cout<<',';
        if(std::isfinite(p.range))std::cout<<p.range;else std::cout<<"null";
        std::cout<<',';
        if(std::isfinite(p.intensity))std::cout<<p.intensity;else std::cout<<"null";
        std::cout<<']';
      }
      std::cout<<"]}"<<std::endl;
    }
    lidar.turnOff();lidar.disconnecting();return frames?0:2;
  } catch(const std::exception& e) {
    std::cerr<<"SDK_ERROR "<<e.what()<<std::endl;
    lidar.turnOff();lidar.disconnecting();return 3;
  }
}
