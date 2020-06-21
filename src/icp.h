#include <pcl/point_types.h>

//#define DEBUG

Eigen::Matrix4f
pcl_answer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_first,
           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_second);

Eigen::Matrix4f
ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_first,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_second);

double rmse(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_first,
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_second);