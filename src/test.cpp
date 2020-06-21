#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "test.h"
#include "icp.h"

void test()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_first (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../PCDdata/first.pcd", *cloud_first) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        exit (-1);
    }
    std::cout << "Loaded first "
              << cloud_first->width * cloud_first->height << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_second (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../PCDdata/second.pcd", *cloud_second) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        exit (-1);
    }
    std::cout << "Loaded second "
              << cloud_second->width * cloud_second->height << std::endl;

    //pcl_answer(cloud_first,cloud_second);


//    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//    viewer.showCloud (cloud_first);
//
//    while (!viewer.wasStopped ())
//    {
//    }
//
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    //viewer.setBackgroundColor (1.0, 0.5, 1.0);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr body (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile ("../PCDdata/first.pcd", *body);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr head (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::io::loadPCDFile ("../PCDdata/second.pcd", *head);


    //This will display the point cloud in red (R,G,B)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> rgb1 (body, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> rgb2 (head, 0, 255, 0); //This will display the point cloud in green (R,G,B)

    //viewer.addPointCloud (body,"body");// note that before it was showCloud

    viewer.addPointCloud<pcl::PointXYZRGBA> (body, rgb1, "cloud_RGB");
    //viewer.addPointCloud (head,"head");// note that before it was showCloud
    viewer.addPointCloud<pcl::PointXYZRGBA> (head, rgb2, "cloud_RGB2");
    viewer.spin();
}