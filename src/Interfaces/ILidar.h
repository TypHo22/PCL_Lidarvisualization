//STL
#include <iostream>
//PCL
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "src/Implementation/VisualizationHandler.h"

namespace sensor
{
    class ILidar
    {
    public:

        virtual void getSettings() = 0;
        virtual void initSensor() = 0;
        virtual void scan() = 0;
        virtual void processScan() = 0;
        virtual void showScan() = 0;


        visualization::IVisualizationHandler::visuMode visuMode_;
        visualization::VisualizationHandler visuHandler_;
    };

}
