#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <deque>
#include <boost/program_options.hpp>

#include "../include/subscriber/gnss_subscriber.hpp"
#include "../include/subscriber/imu_subscriber.hpp"

using namespace std;
using namespace subscriber_node;
namespace bpo = boost::program_options;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gnss_subscriber_node");
    ros::NodeHandle gnss_node("~");
    
    bpo::options_description opts("all options");
    bpo::variables_map vm;

    opts.add_options()
    ("out_file_name", bpo::value<string>(), "the out file name of ground trurh");

    opts.add_options()
    ("record_size", bpo::value<int>(), "the size of record");
    
    bpo::store(bpo::parse_command_line(argc, argv, opts), vm);

    int recordSize = vm["record_size"].as<int>();
    string outFileName = vm["out_file_name"].as<string>();

    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(gnss_node, "/kitti/oxts/gps/fix", 1000000);
    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(gnss_node, "/kitti/oxts/imu", 1000000);


    while(ros::ok() && gnss_sub_ptr->GetGnssDataDequeSize()<recordSize)
    {
        ros::spinOnce();
    }
    
    deque<GNSSData> new_gnss_data;
    gnss_sub_ptr->ParseData(new_gnss_data);

    deque<IMUData> new_imu_data;
    imu_sub_ptr->ParseData(new_imu_data);

    while (new_gnss_data.size() > 0 && new_imu_data.size() >0)
    {
        IMUData imu_data = new_imu_data.front();
        GNSSData gnss_data = new_gnss_data.front();

        double delta_time = imu_data.time - gnss_data.time;
        if (delta_time < -0.05)
        {
            new_imu_data.pop_front();
        }
        else if (delta_time > 0.05)
        {
            new_gnss_data.pop_front();
        }
        else
        {
            std::cout<<"imu timestamp: "<<to_string(imu_data.time)<<" "<<"gnss timestamp: "<<to_string(gnss_data.time)<<std::endl;
            break;
        }
    }

    // FILE *fp = fopen(outFileName.c_str(), "wb");
    // for (int i=0; i<new_gnss_data.size(); i++)
    // {
    //     GNSSData gnss_data = new_gnss_data.at(i);
    //     double timestamp = gnss_data.time;
    //     double latitude = gnss_data.latitude;
    //     double longitude = gnss_data.longitude;
    //     double altitude = gnss_data.altitude;

    //     if (i == 0)
    //     {
    //         gnss_sub_ptr->InitOriginPosition(latitude,longitude,altitude);   
    //     }
    //     else
    //     {
    //         double x, y, z;
    //         gnss_sub_ptr->UpdateXYZ(latitude,longitude,altitude, x, y, z);
    //         std::cout<<"x: "<<x<<" y: "<<y<<" z:"<<z<<std::endl;

    //         fprintf(fp, "%f %f %f %f\n",timestamp, x, y, z);
    //         fflush(fp);
    //     }
    // }
    // fclose(fp);
    std::cout<<"All Done"<<std::endl;
    return 0;
}