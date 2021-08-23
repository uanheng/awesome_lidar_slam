/*
* @Description: lidar odom data subscriber
* @Author: fei
* @Data: 2021-08-08
*/

#ifndef SUBSCRIBER_LIDAR_ODOM_SUBSCRIBER_HPP_
#define SUBSCRIBER_LIDAR_ODOM_SUBSCRIBER_HPP_

#include<deque>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

struct OdomPoint
{
    double timestamp;

    double x;
    double y;
    double z;
    
    double roll;
    double pitch;
    double yaw;
};

namespace subscriber_node{
    class LidarOdomSubscriber 
    {
        public:
            LidarOdomSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
            LidarOdomSubscriber() = default;
            void ParseData(std::deque<OdomPoint>& deque_odom_buff);
            int GetLidarOdomSize();

        private:
            void laserOdomHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);

        private:
            ros::NodeHandle nh_;
            std::deque<OdomPoint> new_odom_deque_;
            ros::Subscriber lidar_odom_subscriber_;
    };
}
#endif