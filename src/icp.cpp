#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

void pcl_answer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_first,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_second)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_first);
    icp.setInputTarget(cloud_second);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
}