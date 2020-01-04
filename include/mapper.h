#ifndef MAPPER_H
#define MAPPER_H

#include <unistd.h>
#include <iostream>
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "pcl_ros/impl/transforms.hpp"
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>

using namespace std;

class Mapper
{
public:
    Mapper(int argc, char** argv);
    ~Mapper();

private:
    void getTransformedPointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud);

    string nodeName;
    ros::NodeHandlePtr node;

    //This is the frame conversion between lidar and ground frame
    boost::shared_ptr<tf::TransformListener> frame_listener;

    ros::Publisher pc_pub;
    ros::Subscriber lidar_sub;

};

#endif // MAPPER_H
