#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>

using namespace std;
using namespace pcl;
using namespace boost::filesystem;

namespace bpo = boost::program_options;

vector<string> LoadKittiVelodybeFiles(const string lidarFolderPath)
{
    vector<string> lidarFileList;
    directory_iterator end;
    for (directory_iterator dir(lidarFolderPath); dir!=end; dir++)
    {
        if (dir->path().extension() == ".bin")
        {
            lidarFileList.push_back(dir->path().string());
        }
    }
    sort(lidarFileList.begin(), lidarFileList.end());
    return lidarFileList;
}

pcl::PointCloud<pcl::PointXYZI> KittiVelodyneToPointCloud(const std::string lidarDataPath)
{
   std::ifstream lidarDataFile(lidarDataPath, ifstream::in | ifstream::binary);
   lidarDataFile.seekg(0, ios::end);
   const size_t numElements = lidarDataFile.tellg() / sizeof(float);
   lidarDataFile.seekg(0, ios::beg);

   vector<float> lidarDataBuffer(numElements);
   lidarDataFile.read(reinterpret_cast<char*>(&lidarDataBuffer[0]),numElements*sizeof(float));
   
   std::cout<<"totally: "<<lidarDataBuffer.size() / 4.0 << "points in this lidar frame \n";
   std::vector<Eigen::Vector3d> lidarPoints;

   pcl::PointCloud<pcl::PointXYZI> laserCloud;
   
   for (size_t i=0; i < lidarDataBuffer.size(); i+=4)
   {
       lidarPoints.emplace_back(lidarDataBuffer[i], lidarDataBuffer[i+1], lidarDataBuffer[i+2]);
       pcl::PointXYZI point;
       point.x = lidarDataBuffer[i];
       point.y = lidarDataBuffer[i+1];
       point.z = lidarDataBuffer[i+2];
       point.intensity = lidarDataBuffer[i+3];
       laserCloud.push_back(point);
   }

   return laserCloud;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "feed_kitti");
    ros::NodeHandle kitti_node("~");
    
    bpo::options_description opts("all options");
    bpo::variables_map vm;

    opts.add_options()
    ("lidar_folder_path", bpo::value<string>(), "the folder path of kitti velodyne");
    bpo::store(bpo::parse_command_line(argc, argv, opts), vm);

    string lidarFolderPath = vm["lidar_folder_path"].as<string>();
    std::cout<<"lidar folder path: "<<lidarFolderPath<<std::endl;

    vector<string> lidarFileList = LoadKittiVelodybeFiles(lidarFolderPath);
    float startTimeStamp = 0.0;
    float deltaTimeValue = 0.1; // 100ms

    ros::Rate r(5);
    
    //write a ros Publisher to publish the point cloud for loam input 
    ros::Publisher pubLaserCloud = kitti_node.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);
    int n = 0;

    while(n < lidarFileList.size() && ros::ok())
    {
        pcl::PointCloud<pcl::PointXYZI> laserCloud = KittiVelodyneToPointCloud(lidarFileList[n]);
        
        sensor_msgs::PointCloud2 laserCloudMsg;
        pcl::toROSMsg(laserCloud, laserCloudMsg);
        
        float curTimeStamp = startTimeStamp + n * deltaTimeValue;
        laserCloudMsg.header.stamp = ros::Time().fromSec(curTimeStamp);
        laserCloudMsg.header.frame_id = "/camera_init";
        pubLaserCloud.publish(laserCloudMsg);

        std::cout<<"msg timestamp: "<<curTimeStamp<<" has been published"<<std::endl;
        n++;
        r.sleep();
    }
    
    std::cout<<"All Done"<<std::endl;
    return 0;
    
}