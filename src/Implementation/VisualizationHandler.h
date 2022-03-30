#include "src/Interfaces/IVisualizationHandler.h"
#include <pcl/visualization/range_image_visualizer.h>
namespace visualization
{
    class VisualizationHandler : public IVisualizationHandler
    {
        public:
        void initVisualizationHandler(visuMode visuMode, std::string name) override;

        void initCloudViewer(std::string name) override;
        void renderCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& pclData) override;
        void renderPCLIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr& pclData) override;

        void initPCLViewer(std::string name) override;
        void renderPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr& pclData) override;
        void renderPCLRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pclData) override;

        private:
        std::unique_ptr<pcl::visualization::CloudViewer> cloudViewer_;
        std::unique_ptr<pcl::visualization::RangeImageVisualizer> rangeViewer_;
        std::unique_ptr<pcl::visualization::PCLVisualizer> pclViewer_;
    };
}
