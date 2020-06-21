#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
//#include <pcl/registration/transformation_estimation.h.>

#include <Eigen/Dense>

#include "icp.h"

using namespace Eigen;

Eigen::Matrix4f
pcl_answer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_first,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_second)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_second);
    icp.setInputTarget(cloud_first);
    //icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-9);
    //icp.setEuclideanFitnessEpsilon(1);
    //icp.setMaxCorrespondenceDistance(0.5); // 50cm
    icp.setRANSACOutlierRejectionThreshold(0.03);

    //icp.setMaximumIterations(50);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    //Eigen::Matrix4f H;
    //pcl::estimateRigidTransform(cloud_first, cloud_second, false, H);

    //return H;
    return icp.getFinalTransformation();
}


double rmse(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_first,
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_second)
{
    double rmse=0;
    double distance=0;

    Vector3d p1,p2;
    int N = cloud_first->size() < cloud_second->size()
                                 ? cloud_first->size() : cloud_second->size();

    for(int i=0;i<N;++i){
        p1 = (*cloud_first)[i].getVector3fMap().cast<double>();
        p2 = (*cloud_second)[i].getVector3fMap().cast<double>();
        distance += (p1 - p2).squaredNorm();
    }
    rmse = std::sqrt(distance / N);

    return rmse;
}

// HOMEWORK
Eigen::Matrix4f
ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_first,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_second)
{
    std::vector< int > idx;
    pcl::getApproximateIndices<pcl::PointXYZ> (cloud_first,cloud_second, idx);

    Vector3d p1,p2;
    int N = cloud_first->size() < cloud_second->size()
                                 ? cloud_first->size() : cloud_second->size();
    for(int i=0;i<N;++i){
        p1 += (*cloud_first)[i].getVector3fMap().cast<double>();
        p2 += (*cloud_second)[idx[i]].getVector3fMap().cast<double>();
    }
    p1 = p1 / N;
    p2 = p2 / N;

    std::vector<Vector3d > q1(N),q2(N);

    for(int i=0;i<N;++i){
        q1[i] = (*cloud_first)[i].getVector3fMap().cast<double>() - p1;
        q2[i] = (*cloud_second)[idx[i]].getVector3fMap().cast<double>() - p2;
    }

    // compute q1*q2^T
    Matrix3d W = Matrix3d::Zero();
    for (int i=0;i<N;++i){
        //W += Vector3d(q1[i].x,q1[i].y,q1[i].z);
        W += q1[i] * q2[i].transpose();
    }

#ifdef DEBUG
    std::cout << "W = " << W << std::endl;
#endif


    // SVD on W
    JacobiSVD<Matrix3d> svd(W, ComputeFullU | ComputeFullV);

    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();

#ifdef DEBUG
    std::cout << "U= " << U << std::endl;
    std::cout << "V= " << V << std::endl;
#endif

    Matrix3d R_ = U * (V.transpose());

    if(R_.determinant()<0){
        R_ = -R_;
    }

    Vector3d t_ = p1 - R_ * p2;

#ifdef DEBUG
    std::cout << "R_ = " << R_ << std::endl;
    std::cout << "t_ = " << t_ << std::endl;
#endif


    Matrix4f T_ = Matrix4f::Zero();
    T_.block<3,3>(0,0) = R_.cast<float>();
    T_.block<3,1>(0,3) = t_.cast<float>();
    T_(3,3) = 1.0;

    std::cout << "T_ = " << T_ << std::endl;

    return T_;
}





