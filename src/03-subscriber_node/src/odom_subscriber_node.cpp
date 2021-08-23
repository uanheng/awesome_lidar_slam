#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "../include/subscriber/laser_odom_subscriber.hpp"
#include "../include/subscriber/gnss_subscriber.hpp"
#include "../include/subscriber/imu_subscriber.hpp"

using namespace std;
using namespace subscriber_node;
namespace bpo = boost::program_options;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "subscriber_node");
    ros::NodeHandle subscriber_node("~");

    bpo::options_description opts("all options");
    bpo::variables_map vm;

    opts.add_options()
    ("out_file_name", bpo::value<string>(), "the out file of result");

    opts.add_options()
    ("record_size", bpo::value<int>(), "the size of record");
    bpo::store(bpo::parse_command_line(argc, argv, opts), vm);
    
    string outFileName = vm["out_file_name"].as<string>();
    int recordSize = vm["record_size"].as<int>();

    std::shared_ptr<LidarOdomSubscriber> odom_sub_ptr = std::make_shared<LidarOdomSubscriber>(subscriber_node, "/laser_odom_to_init", 1000000);
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(subscriber_node, "/kitti/oxts/gps/fix", 1000000);
    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(subscriber_node, "/kitti/oxts/imu", 1000000);

    while (ros::ok() && odom_sub_ptr->GetLidarOdomSize()<recordSize)
    {
        ros::spinOnce();
    }
    
    bool gnss_origin_position_inited = false;
    std::deque<OdomPoint> new_odom_data;
    odom_sub_ptr->ParseData(new_odom_data);

    deque<GNSSData> new_gnss_data;
    gnss_sub_ptr->ParseData(new_gnss_data);

    deque<IMUData> new_imu_data;
    imu_sub_ptr->ParseData(new_imu_data);

    for (size_t i=0; i<new_imu_data.size(); i++)
    {
        IMUData imu_data = new_imu_data.front();
        struct Orientation orientation = imu_data.orientation;
        Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
        Eigen::Vector3d eular = q.matrix().eulerAngles(2,1,0);
        std::cout<<"roll: "<<eular[0]<<std::endl;
        std::cout<<"pitch: "<<eular[1]<<std::endl;
        std::cout<<"yaw"<<eular[2]<<std::endl;
        new_imu_data.pop_front();
        std::cout<<"*******************"<<std::endl;
    }
    return 0;


    std::cout<<"gnss data size: "<<new_gnss_data.size()<<" "<<"imu data size: "<<new_imu_data.size()<<" "<<"laser odom size: "<<new_odom_data.size()<<std::endl;

    while (new_gnss_data.size()>0 && new_imu_data.size()>0 && new_odom_data.size()>0)
    {
        IMUData imu_data = new_imu_data.front();
        GNSSData gnss_data = new_gnss_data.front();
        OdomPoint odom_data = new_odom_data.front();

        double delta_time = odom_data.timestamp - imu_data.time;
        if (delta_time < -0.06) 
        {
            new_odom_data.pop_front();
        } 
        else if (delta_time > 0.06) 
        {
            new_imu_data.pop_front();
            new_gnss_data.pop_front();
        } 
        else 
        {
            std::cout<<"laser odom timestamp: "<<" "<<to_string(odom_data.timestamp)<<" "<<"imu timestamp: "<<to_string(imu_data.time)<<" "<<"gnss timestamp: "<<to_string(gnss_data.time)<<std::endl;
            break;
        }
    }

    GNSSData gnss_data = new_gnss_data.front();
    double latitude = gnss_data.latitude;
    double longitude = gnss_data.longitude;
    double altitude = gnss_data.altitude;

    IMUData imu_data = new_imu_data.front();

    OdomPoint odom_data = new_odom_data.front();
    double odom_x = odom_data.x;
    double odom_y = odom_data.y;
    double odom_z = odom_data.z;
    double roll = odom_data.roll;
    double pitch = odom_data.pitch;
    double yaw = odom_data.yaw;

    Eigen::Vector3d eular(roll,pitch,yaw);
    Eigen::Matrix3d lidar_rotation;
    lidar_rotation = Eigen::AngleAxisd(eular[2], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(eular[1], Eigen::Vector3d::UnitX()) * 
                       Eigen::AngleAxisd(eular[0], Eigen::Vector3d::UnitZ());

    std::cout<<"lidar rotation: "<<std::endl;
    std::cout<<lidar_rotation<<std::endl;
    
    Eigen::Matrix4f lidar_matrix = Eigen::Matrix4f::Identity();
    lidar_matrix(0,3) = odom_x;
    lidar_matrix(1,3) = odom_y;
    lidar_matrix(2,3) = odom_z;
    lidar_matrix.block<3,3>(0,0) = lidar_rotation.cast<float>();

    std::cout<<"lidar matrix: "<<std::endl;
    std::cout<<lidar_matrix<<std::endl; 

    double local_E, local_N, local_U;
    Eigen::Matrix4f odometry_matrix;
    if (!gnss_origin_position_inited)
    {
        gnss_sub_ptr->InitOriginPosition(latitude, longitude, altitude);
        gnss_origin_position_inited = true;
        
        gnss_sub_ptr->UpdateXYZ(latitude, longitude, altitude, local_E, local_N, local_U);
        odometry_matrix(0,3) = local_E;
        odometry_matrix(1,3) = local_N;
        odometry_matrix(2,3) = local_U;
        odometry_matrix.block<3,3>(0,0) = imu_sub_ptr->GetOrientationMatrix(imu_data.orientation);
    }

    Eigen::Matrix4f final_matrix = odometry_matrix;

    FILE *fp;
    fp = fopen(outFileName.c_str(), "wb");
    while(new_odom_data.size()>0)
    {
        OdomPoint odom_data = new_odom_data.front();
        Eigen::Matrix4f point;
        point(0,0) = odom_data.x;
        point(1,0) = odom_data.y;
        point(2,0) = odom_data.z;
        point(3,0) = 1;

        Eigen::Matrix4f global_point = lidar_matrix.inverse() * point;
        global_point = odometry_matrix * global_point;
        std::cout<<"point: "<<std::endl;
        std::cout<<point<<std::endl;
        std::cout<<"global point: "<<std::endl;
        std::cout<<global_point<<std::endl;
        std::cout<<"*************************************"<<std::endl;

        double gps_point_x = -global_point(0,0);
        double gps_point_y = global_point(2,0);
        double gps_point_z = global_point(1,0);
       
        new_odom_data.pop_front();
        fprintf(fp, "%f %f %f %f\n", odom_data.timestamp, gps_point_x, gps_point_y, gps_point_z);
    }
    fflush(fp);
    fclose(fp);
    std::cout<<"All Done"<<std::endl; 
    return 0;

}