//STL
#include <iostream>
//PCL
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "src/Implementation/YLidar_x4.h"


int main()
{
    std::unique_ptr<sensor::ILidar> mySensorPtr(new sensor::YLidar_x4);
    mySensorPtr->getSettings();
    mySensorPtr->initSensor();
    mySensorPtr->visuMode_ = visualization::IVisualizationHandler::visuMode::PCLVISUALIZER;
    mySensorPtr->scan();
    return (0);
}
