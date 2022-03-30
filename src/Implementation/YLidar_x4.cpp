#include "YLidar_x4.h"
#include "LidarMath.h"
//STL
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <memory>
//PCL
#include <pcl/visualization/pcl_visualizer.h>

using namespace ydlidar;
namespace sensor
{

YLidar_x4::YLidar_x4() :
    errCycle_(0),
    scanCloud_(new pcl::PointCloud<pcl::PointXYZ>),
    scanCloudIntensity_(new pcl::PointCloud<pcl::PointXYZI>),
    scanCloudColoured_(new pcl::PointCloud<pcl::PointXYZRGBA>)
{
    visuMode_ = visualization::VisualizationHandler::visuMode::NONE;

}

void YLidar_x4::getSettings()
{
    std::cout<<"YLidar_x4::getSettings"<<std::endl;

    printf("__   ______  _     ___ ____    _    ____  \n");
    printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
    printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
    printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
    printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
    printf("\n");
    fflush(stdout);

    ydlidar::os_init();

    std::map<std::string, std::string> port_s =
      ydlidar::lidarPortList();
    std::map<std::string, std::string>::iterator it;

    if (port_s.size() == 1)
    {
      port_ = port_s.begin()->second;
    }
    else
    {
      int id = 0;

      for (it = port_s.begin(); it != port_s.end(); it++) {
        printf("%d. %s\n", id, it->first.c_str());
        id++;
      }

      if (port_s.empty()) {
        printf("Not Lidar was detected. Please enter the lidar serial port_:");
        std::cin >> port_;
      } else {
        while (ydlidar::os_isOk()) {
          printf("Please select the lidar port_:");
          std::string number;
          std::cin >> number;

          if ((size_t)atoi(number.c_str()) >= port_s.size()) {
            continue;
          }

          it = port_s.begin();
          id = atoi(number.c_str());

          while (id) {
            id--;
            it++;        CYdLidar laser_;
          }

          port_ = it->second;
          break;
        }
      }
    }

    std::map<int, int> baudrateList;
    baudrateList[0] = 115200;
    baudrateList[1] = 128000;
    baudrateList[2] = 153600;
    baudrateList[3] = 230400;
    baudrateList[4] = 512000;

    printf("Baudrate:\n");

    for (std::map<int, int>::iterator it = baudrateList.begin();
         it != baudrateList.end(); it++) {
      printf("%d. %d\n", it->first, it->second);
    }

    while (ydlidar::os_isOk()) {
      printf("Please select the lidar baudrate:");
      std::string number;
      std::cin >> number;

      if ((size_t)atoi(number.c_str()) > baudrateList.size()) {
        continue;
      }

      baudrate_ = baudrateList[atoi(number.c_str())];
      break;
    }

    if (!ydlidar::os_isOk()) {
        throw std::runtime_error("invalid baudrate");
    }

    std::string input_channel;
    printf("Whether the Lidar is one-way communication[yes/no]:");
    std::cin >> input_channel;
    std::transform(input_channel.begin(), input_channel.end(),
                   input_channel.begin(),
    [](unsigned char c) {
      return std::tolower(c);  // correct
    });

    if (input_channel.find("y") != std::string::npos) {
      isSingleChannel_ = true;
    }

    if (!ydlidar::os_isOk()) {
      throw std::runtime_error("invalid communication");
    }

    std::string input_frequency;

    //float frequency = 8.0;

    while (ydlidar::os_isOk() && !isSingleChannel_) {
      printf("Please enter the lidar scan frequency[5-12]:");
      std::cin >> input_frequency;
      frequency_ = atof(input_frequency.c_str());

      if (frequency_ <= 12 && frequency_ >= 5.0) {
        break;
      }

      fprintf(stderr,
              "Invalid scan frequency,The scanning frequency range is 5 to 12 HZ, Please re-enter.\n");
    }

    if (!ydlidar::os_isOk()) {
      throw std::runtime_error("invalid scan frequency");
    }

}

void YLidar_x4::initSensor()
{
    std::cout<<"YLidar_x4::initSensor"<<std::endl;

    //////////////////////string property/////////////////
    /// lidar port_
    laser_.setlidaropt(LidarPropSerialPort, port_.c_str(), port_.size());
    /// ignore array
    std::string ignore_array;
    ignore_array.clear();
    laser_.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                      ignore_array.size());

    //////////////////////int property/////////////////
    /// lidar baudrate
    laser_.setlidaropt(LidarPropSerialBaudrate, &baudrate_, sizeof(int));
    /// tof lidar
    int optval = TYPE_TRIANGLE;
    laser_.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
    /// device type
    optval = YDLIDAR_TYPE_SERIAL;
    laser_.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
    /// sample rate
    optval = isSingleChannel_ ? 3 : 4;
    laser_.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
    /// abnormal count
    optval = 4;
    laser_.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

    //////////////////////bool property/////////////////
    /// fixed angle resolution
    bool b_optvalue = false;
    laser_.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
    /// rotate 180
    laser_.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
    /// Counterclockwise
    laser_.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
    b_optvalue = true;
    laser_.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
    /// one-way communication
    laser_.setlidaropt(LidarPropSingleChannel, &isSingleChannel_, sizeof(bool));
    /// intensity
    b_optvalue = false;
    laser_.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
    /// Motor DTR
    b_optvalue = true;
    laser_.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
    /// HeartBeat
    b_optvalue = false;
    laser_.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));

    //////////////////////float property/////////////////
    /// unit: °
    float f_optvalue = 180.0f;
    laser_.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
    f_optvalue = -180.0f;
    laser_.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
    /// unit: m
    f_optvalue = 64.f;
    laser_.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
    f_optvalue = 0.05f;
    laser_.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
    /// unit: Hz
    laser_.setlidaropt(LidarPropScanFrequency, &frequency_, sizeof(float));

    initSuccessfull_ = laser_.initialize();

    if (initSuccessfull_) {
      initSuccessfull_ = laser_.turnOn();
    } else {
      fprintf(stderr, "%s\n", laser_.DescribeError());
      fflush(stderr);
    }
}

void YLidar_x4::scan()
{

    visuHandler_.initVisualizationHandler(visuMode_,"YLidar_x4");

    while (initSuccessfull_ && ydlidar::os_isOk() && errCycle_ < 3)
    {
      if (laser_.doProcessSimple(scan_))
      {
        processScan();
      }
      else
      {
         fprintf(stderr, "Failed to get Lidar Data\n");
         fflush(stderr);
         errCycle_++;
         std::cout<<"------------" <<errCycle_<<std::endl;
         std::cout<<"CYCLE: " <<errCycle_<<std::endl;
         std::cout<<"------------" <<errCycle_<<std::endl;
      }

    }

    laser_.turnOff();
    laser_.disconnecting();

    std::cout<<"DISCONNECT"<<std::endl;
}

void YLidar_x4::processScan()
{
    switch (visuMode_)
    {
        case(visualization::IVisualizationHandler::visuMode::NONE):
        {
            for(auto& sp : scan_.points)
            {
              pcl::PointXYZ p = polar2cartesian(sp.range,sp.angle);
              scanCloud_->points.push_back(p);
              showScan();
            }
            scanCloud_->points.clear();
            break;//fall through
        }
        case(visualization::IVisualizationHandler::visuMode::CLOUDVIEWER):
        {
            for(auto& sp : scan_.points)
            {
              pcl::PointXYZ p = polar2cartesian(sp.range,sp.angle);
              scanCloud_->points.push_back(p);
              showScan();
            }
            scanCloud_->points.clear();
            break;
        }
        case(visualization::IVisualizationHandler::visuMode::PCLVISUALIZER):
        {
            for(auto& sp : scan_.points)
            {
              pcl::PointXYZ p = polar2cartesian(sp.range,sp.angle);
              scanCloud_->points.push_back(p);
              showScan();
            }
            scanCloud_->points.clear();
            break;
        }
        case(visualization::IVisualizationHandler::visuMode::INTENSITY):
        {
            for(auto& sp : scan_.points)
            {
              pcl::PointXYZI p = polar2cartesian(sp.range,sp.angle,sp.intensity);
              scanCloudIntensity_->points.push_back(p);
              showScan();
            }
            scanCloudIntensity_->points.clear();
            break;
        }
        case(visualization::IVisualizationHandler::visuMode::RANGE):
        {
            //Ylidar does not provide overloade of < operator
            auto rangeComp = [](const LaserPoint& a, const LaserPoint& b)
            {
                return a.range <= b.range;
            };

            const float farest = std::max_element(scan_.points.begin(),scan_.points.end(), rangeComp)->range;

            for(auto& sp : scan_.points)
            {
              pcl::PointXYZRGBA p = polar2cartesian(sp.range,sp.angle,farest, 255.0f);
              scanCloudColoured_->points.push_back(p);

              showScan();
            }
            scanCloudColoured_->points.clear();
            break;
        }

    }

    showScan();
}

void YLidar_x4::showScan()
{
    switch (visuMode_)
    {
        case(visualization::IVisualizationHandler::visuMode::NONE):
        {
            std::string points;

            for(const auto& p : scanCloud_->points)
            {
                points.append("X: ");
                points.append(std::to_string(p.x));
                points.append(" Y: ");
                points.append(std::to_string(p.y));
                points.append(" Z: ");
                points.append(std::to_string(p.z));
                points.append("\n");
            }
            std::cout<<points<<std::endl;
            std::cout<<"Points scanned: "<< points.size()<<std::endl;
            break;//fall through
        }
        case(visualization::IVisualizationHandler::visuMode::CLOUDVIEWER):
        {
            visuHandler_.renderCloud(scanCloud_);
            break;
        }
        case(visualization::IVisualizationHandler::visuMode::PCLVISUALIZER):
        {
            visuHandler_.renderPCL(scanCloud_);
            break;
        }
        case(visualization::IVisualizationHandler::visuMode::INTENSITY):
        {
            visuHandler_.renderPCLIntensity(scanCloudIntensity_);
            break;
        }
        case(visualization::IVisualizationHandler::visuMode::RANGE):
        {
            visuHandler_.renderPCLRGBA(scanCloudColoured_);
            break;
        }
    }
}


}




