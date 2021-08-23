#include<iostream>
#include<string.h>

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/registration/icp.h>
#include<pcl/registration/ndt.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main(int argc, char** argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr dst_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    string src_path = "/home/luffy/work/data/TRO_corner1.pcd";
    string dst_path = "/home/luffy/work/data/TRO_corner2.pcd";

    pcl::io::loadPCDFile<pcl::PointXYZ>(src_path, *src_cloud);
    std::cout<<"src cloud size: "<<src_cloud->size()<<std::endl;
    
    pcl::io::loadPCDFile<pcl::PointXYZ>(dst_path, *dst_cloud);
    std::cout<<"dst cloud size: "<<dst_cloud->size()<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(src_cloud);
    icp.setInputTarget(dst_cloud);

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1);

    icp.align(*final_cloud);

    std::cout<<"has converged: "<<icp.hasConverged()<<std::endl;
    std::cout<<"score: "<<icp.getFitnessScore()<<std::endl;
    std::cout<<"final cloud size："<<final_cloud->size()<<std::endl;

    //pcl::io::savePCDFileASCII("/home/luffy/work/data/icp_result.pcd", *final_cloud);

    std::cout<<icp.getFinalTransformation()<<std::endl;

    final_cloud->clear();

     //初始化正态分布变换（NDT）
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    //设置依赖尺度NDT参数
    //为终止条件设置最小转换差异
    ndt.setTransformationEpsilon(0.01);
    //为More-Thuente线搜索设置最大步长
    ndt.setStepSize(0.1);
    //设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt.setResolution(1.0);
    //设置匹配迭代的最大次数
    ndt.setMaximumIterations(35);
    // 设置要配准的点云
    ndt.setInputCloud(src_cloud);
    //设置点云配准目标
    ndt.setInputTarget(dst_cloud);
    //将icp 配准结果作为ndt配准输入的初始位姿
    Eigen::Matrix4f init_guess = icp.getFinalTransformation();
    //计算需要的刚体变换以便将输入的点云匹配到目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud, init_guess);
    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
        << " score: " << ndt.getFitnessScore() << std::endl;
    std::cout<<ndt.getFinalTransformation()<<std::endl;
    //保存转换的输入点云
    pcl::io::savePCDFileASCII("/home/luffy/work/data/ndt_result.pcd", *output_cloud);
    
    // pcl::visualization::PCLVisualizer viewer("registration Viewer"); 
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(src_cloud, 0, 255, 0);  
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(dst_cloud, 255, 0, 0);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(final_cloud, 0, 0, 255);
    // viewer.setBackgroundColor(255, 255, 255);
    // //viewer.addPointCloud(src_cloud, src_h, "source cloud");
    // //viewer.addPointCloud(dst_cloud, tgt_h, "tgt cloud");
    // viewer.addPointCloud(final_cloud, final_h, "final cloud");

    // while (!viewer.wasStopped())
    // {   viewer.spinOnce(100);
    //     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    // }

    return 0;
}
