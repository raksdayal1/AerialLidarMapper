#ifndef PCL_TRANSFORMATION_H
#define PCL_TRANSFORMATION_H

#include <unistd.h>
#include <iostream>
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>

#define SEMI_MAJOR 6378137.0
#define SEMI_MINOR 6356752.314245

class PointTransform
{
public:
    PointTransform(int argc, char** argv);
    ~PointTransform();

private:
    void LLA2ECEF(tf::Point LLA, tf::Point& ECEF);

    void DJIGPS_InitLLA_cb(const sensor_msgs::NavSatFixConstPtr &dji_gps_msg);
    void DJIGPS_cb(const sensor_msgs::NavSatFixConstPtr &dji_gps_msg);
    void DJIAtt_cb(const geometry_msgs::QuaternionStampedConstPtr &dji_att_msg);
    void LidarAtt_cb(const sensor_msgs::ImuConstPtr &lidar_imu_msg);
    void LidarPC_cb(const sensor_msgs::PointCloud2ConstPtr &pc_msg);


    ros::NodeHandlePtr node;
    std::string nodeName;

    // Ros Subscribers
    ros::Subscriber djiGps_sub, djiAtt_sub, lidarImu_sub, lidarPC_sub;

    boost::shared_ptr<tf::TransformBroadcaster> djiinertial_tfbroadcaster; // This is only translation from inertial frame
    boost::shared_ptr<tf::TransformBroadcaster> djibodyfixed_tfbroadcaster; // This is the body fixed frmae
    boost::shared_ptr<tf::TransformBroadcaster> lidar_tfbroadcaster; //This is the frame conversion between lidar and body fixed frame


    // GPS transformation variables
    float lat_avg, long_avg, alt_avg;
    int counter; // counter to keep track of measurements
    tf::Point ECEF0; // origin of ECEF frame
    //geometry_msgs::PointStamped NED;

    bool GotReferenceLLA;

};


#endif // PCL_TRANSFORMATION_H
