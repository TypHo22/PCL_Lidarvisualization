#include "src/Implementation/VisualizationHandler.h"
//pcl
#include <pcl/range_image/range_image.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
namespace visualization
{

void VisualizationHandler::initVisualizationHandler(IVisualizationHandler::visuMode visuMode, std::string name)
{
    switch (visuMode) {
        case(IVisualizationHandler::visuMode::CLOUDVIEWER):
        {
            initCloudViewer(name);
            break;
        }
        case(IVisualizationHandler::visuMode::PCLVISUALIZER):
        {
            initPCLViewer(name);
            break;
        }
        case(IVisualizationHandler::visuMode::INTENSITY):
        {
            initCloudViewer(name);
            break;
        }
        case(IVisualizationHandler::visuMode::RANGE):
        {
            initPCLViewer(name);
            break;
        }
    }
}

void VisualizationHandler::initCloudViewer(std::string name)
{
  std::cout<<"VisualizationHandler::initCloudViewer"<<std::endl;
  cloudViewer_ = std::move(std::unique_ptr<pcl::visualization::CloudViewer>(new pcl::visualization::CloudViewer(name)));
}
void VisualizationHandler::renderCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& pclData)
{
    cloudViewer_->showCloud(pclData);
}

void VisualizationHandler::renderPCLIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr &pclData)
{
    cloudViewer_->showCloud(pclData);
}

void VisualizationHandler::initPCLViewer(std::string name)
{
    std::cout<<"initPCLViewer"<<std::endl;
    pclViewer_ = std::move(std::unique_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer(name)));
    pclViewer_->setBackgroundColor (0, 0, 0);
    pclViewer_->addCoordinateSystem (1.0);
    pclViewer_->initCameraParameters ();
    pclViewer_->setCameraPosition(0, 0, 16, 1, 0, 1);// 16 m from above
}

void VisualizationHandler::renderPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr &pclData)
{
    //http://pointclouds.org/documentation/tutorials/pcl_visualizer.html#pcl-visualizer
    pclViewer_->removeAllPointClouds();
    pclViewer_->addPointCloud(pclData,"cloud");
    pclViewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0,0, "cloud");
    pclViewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");
    pclViewer_->spinOnce(10);// to clarify spin once

}

void VisualizationHandler::renderPCLRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pclData)
{
    std::cout<<"renderPCLrgba"<<std::endl;
    pclViewer_->removeAllPointClouds();
    pclViewer_->addPointCloud(pclData,"cloud");
    pclViewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");
    pclViewer_->spinOnce(10);// to clarify spin once
}



}

