/*
 * @Description: subscribe lidar odom data
 * @Author: fei
 * @Date: 2021-08-08
 */
#include "subscriber/laser_odom_subscriber.hpp"

namespace subscriber_node
{
    LidarOdomSubscriber::LidarOdomSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) 
    {
        lidar_odom_subscriber_ = nh_.subscribe(topic_name, buff_size, &LidarOdomSubscriber::laserOdomHandler, this);
    }

    void LidarOdomSubscriber::laserOdomHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
    {
        OdomPoint odom;

        odom.timestamp = laserOdometry->header.stamp.toSec();
        odom.x = laserOdometry->pose.pose.position.x;
        odom.y = laserOdometry->pose.pose.position.y;
        odom.z = laserOdometry->pose.pose.position.z;

        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        odom.roll = roll;
        odom.pitch = pitch;
        odom.yaw = yaw;
    
        new_odom_deque_.push_back(odom);
        std::cout<<"lidar odom data size: "<<new_odom_deque_.size()<<std::endl;
        return;
    }

    void LidarOdomSubscriber::ParseData(std::deque<OdomPoint>& deque_odom_buff)
    {
        if (new_odom_deque_.size() > 0) 
        {
            deque_odom_buff.insert(deque_odom_buff.end(), new_odom_deque_.begin(), new_odom_deque_.end());
            new_odom_deque_.clear();
        }
    }

    int LidarOdomSubscriber::GetLidarOdomSize()
    {
        return new_odom_deque_.size();
    }
}