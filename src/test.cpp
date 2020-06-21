#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/impl/io.hpp>


#include "test.h"
#include "icp.h"

#include <unistd.h>


void test()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_first (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../PCDdata/first.pcd", *cloud_first) == -1)
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        exit (-1);
    }
    std::cout << "Loaded first "
              << cloud_first->width * cloud_first->height << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_second (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../PCDdata/second.pcd", *cloud_second) == -1)
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        exit (-1);
    }
    std::cout << "Loaded second "
              << cloud_second->width * cloud_second->height << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    //auto transform_1 = pcl_answer(cloud_first,cloud_second);
    // pcl::transformPointCloud (*cloud_second, *transformed_cloud, transform_1);

    pcl::copyPointCloud(*cloud_second, *transformed_cloud);

    double error = 0;
    double threshold = 10;
    int num_iterations =  20;

    int i;
    // HOMEWORK
    for(i=0;i<num_iterations;++i){
        auto transform_2 = ICP(cloud_first,transformed_cloud);
        pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, transform_2);
        error = rmse(cloud_first,transformed_cloud);
        if(error < threshold){
            break;
        }
    }
    std::cout << "setp " << i << ", error : " << error << std::endl;

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    //viewer.setBackgroundColor (1.0, 0.5, 1.0);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr first_color (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr second_color (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud_color (new pcl::PointCloud<pcl::PointXYZRGBA>);


    pcl::copyPointCloud(*cloud_first, *first_color);
    pcl::copyPointCloud(*cloud_second, *second_color);
    pcl::copyPointCloud(*transformed_cloud, *transformed_cloud_color);


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> red (first_color, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> green (second_color, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> blue (transformed_cloud_color, 0, 0, 255);


    viewer.addPointCloud<pcl::PointXYZRGBA> (first_color, red, "cloud_first");
    viewer.addPointCloud<pcl::PointXYZRGBA> (second_color, green, "cloud_second");
    viewer.addPointCloud<pcl::PointXYZRGBA> (transformed_cloud_color, blue, "cloud_result");
    viewer.spin();
}