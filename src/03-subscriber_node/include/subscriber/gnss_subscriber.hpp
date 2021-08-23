/*
* @Description: gnss data subscriber
* @Author: fei
* @Data: 2021-08-08
*/

#ifndef SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define SUBSCRIBER_GNSS_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include "../../third_party/GeographicLib/include/Geocentric/LocalCartesian.hpp"

struct GNSSData
{
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;
    int service = 0;
};

namespace subscriber_node{

    class GNSSSubscriber
    {
    public:
        GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        GNSSSubscriber() = default;
        void ParseData(std::deque<GNSSData>& deque_gnss_data);
        int GetGnssDataDequeSize();

        void InitOriginPosition(double latitude, double longitude, double altitude);
        void UpdateXYZ(
            double latitude, 
            double longitude, 
            double altitude,
            double& local_E,
            double& local_N,
            double& local_U);
        
    private:
        void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber gnss_subscriber_;
        std::deque<GNSSData> new_gnss_data_;

        static GeographicLib::LocalCartesian geo_converter;
        static bool origin_position_inited;

    };
}

#endif