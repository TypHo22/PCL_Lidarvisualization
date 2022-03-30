#include "src/Interfaces/ILidar.h"
#include "external/CYdLidar.h"
//PCL
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>

namespace sensor
{
    class YLidar_x4 : public ILidar
    {
    public:
        YLidar_x4();
        void getSettings() override;
        void initSensor() override;
        void scan() override;
        void processScan() override;
        void showScan() override;

    private:
        //settings
        std::string port_;
        int baudrate_;
        bool isSingleChannel_;
        float frequency_;

        //runtime
        CYdLidar laser_;
        LaserScan scan_;
        int errCycle_;
        bool initSuccessfull_;

        //pcl
        pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloud_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr scanCloudIntensity_;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scanCloudColoured_;
    };
}

