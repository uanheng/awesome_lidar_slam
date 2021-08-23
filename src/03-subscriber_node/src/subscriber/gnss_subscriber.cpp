/*
* @Description: gnss data subscriber
* @Author: fei
* @Data: 2021-08-08
*/

#include "subscriber/gnss_subscriber.hpp"

//静态成员变量必须在类外初始化
bool subscriber_node::GNSSSubscriber::origin_position_inited = false;
GeographicLib::LocalCartesian subscriber_node::GNSSSubscriber::geo_converter;

namespace subscriber_node{
    
    GNSSSubscriber::GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size) 
    :nh_(nh) 
    {
        gnss_subscriber_ = nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::msg_callback, this);
    }
    
    void GNSSSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr) 
    {
        GNSSData gnss_data;
        gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();
        gnss_data.latitude = nav_sat_fix_ptr->latitude;
        gnss_data.longitude = nav_sat_fix_ptr->longitude;
        gnss_data.altitude = nav_sat_fix_ptr->longitude;
        gnss_data.status = nav_sat_fix_ptr->status.status;
        gnss_data.service = nav_sat_fix_ptr->status.service;

        new_gnss_data_.push_back(gnss_data);
        std::cout<<"gnss data size: "<<new_gnss_data_.size()<<std::endl;
    }

    int GNSSSubscriber::GetGnssDataDequeSize()
    {
        return new_gnss_data_.size();
    }

    void GNSSSubscriber::ParseData(std::deque<GNSSData>& gnss_data_buff) 
    {
        if (new_gnss_data_.size() > 0) 
        {
            gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
            new_gnss_data_.clear();
        }
    }

    void GNSSSubscriber::InitOriginPosition(double latitude, double longitude, double altitude) 
    {
        geo_converter.Reset(latitude, longitude, altitude);
        origin_position_inited = true;
    }

    void GNSSSubscriber::UpdateXYZ(
            double latitude, 
            double longitude, 
            double altitude,
            double& local_E,
            double& local_N,
            double& local_U) 
    {
        if (!origin_position_inited) 
        {
            std::cout<< "GeoConverter has not set origin position"<<std::endl;
        }
        geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
    }
}