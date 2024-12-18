#include "utility.h"
#include <sensor_msgs/NavSatFix.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

class OdomToGPSConverter {
public:
    OdomToGPSConverter() {
        //参数初始化
        init = false;

        // 订阅里程计话题
        odom_sub = nh.subscribe("liorf/mapping/odometry", 10, &OdomToGPSConverter::odomCallback, this);
        // 发布GPS话题
        gps_pub = nh.advertise<sensor_msgs::NavSatFix>("liorf/mapping/gps_optimized2", 10);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        sensor_msgs::NavSatFix gps_msg;
        if(!init)
        {
            ros::param::get("/origin_x", gpsInit[0]);
            ros::param::get("/origin_y", gpsInit[1]);
            ros::param::get("/origin_z", gpsInit[2]);
            ROS_INFO("origin_x, origin_y, origin_z: %f, %f, %f", gpsInit[0], gpsInit[1], gpsInit[2]);
            if(gpsInit[0])
            {
                init = true;
                // 初始化原点
                proj.Reset(gpsInit[0], gpsInit[1], gpsInit[2]);
            }
        }
        odomTrans << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

        // 转换到经纬度
        proj.Reverse(odomTrans[0], odomTrans[1], odomTrans[2], gps_msg.latitude, gps_msg.longitude, gps_msg.altitude);

        // 创建并发布NavSatFix消息
        gps_msg.header.stamp = ros::Time::now();
        gps_msg.header.frame_id = "gps";        
        gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        gps_pub.publish(gps_msg);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Publisher gps_pub;
    GeographicLib::LocalCartesian proj;
    bool init;
    Eigen::Vector3d gpsInit, odomTrans;
};

 

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_gps_node");
    OdomToGPSConverter converter;
    ros::spin();
    return 0;
}
