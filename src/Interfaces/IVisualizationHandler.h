#include <pcl/visualization/cloud_viewer.h>

namespace visualization
{
    class IVisualizationHandler
    {
        public:

        enum struct visuMode
        {
            NONE,
            CLOUDVIEWER,
            PCLVISUALIZER,
            INTENSITY,
            RANGE
        };

        virtual void initVisualizationHandler(visuMode visuMode, std::string name) = 0;

        virtual void initCloudViewer(std::string name) = 0;
        virtual void renderCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& pclData) = 0;
        virtual void renderPCLIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr& pclData) = 0;

        virtual void initPCLViewer(std::string name) = 0;
        virtual void renderPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr& pclData) = 0;
        virtual void renderPCLRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pclData)  = 0;
    };
}
