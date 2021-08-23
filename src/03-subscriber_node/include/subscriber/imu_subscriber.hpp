/*
 * @Description: subscribe imu data
 * @Author: fei
 * @Date: 2021-08-08
 */
#ifndef SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <deque>
#include <Eigen/Dense>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"

struct LinearAcceleration 
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct AngularVelocity 
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};
    
struct Orientation 
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 0.0;
};

struct IMUData
{
    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;
};

namespace subscriber_node {
class IMUSubscriber {
  public:
    IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    IMUSubscriber() = default;
    void ParseData(std::deque<IMUData>& deque_imu_data);
    Eigen::Matrix3f GetOrientationMatrix(struct Orientation orientation);

  private:
    void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_subscriber_;

    std::deque<IMUData> new_imu_data_; 
};
}
#endif