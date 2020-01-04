#include "mapper.h"

Mapper::Mapper(int argc, char **argv)
{

    this->nodeName = "Mapper";
    if(!ros::isInitialized())
    {
        ros::init(argc, argv, this->nodeName);
    }

    this->node.reset(new ros::NodeHandle(this->nodeName));

    this->pc_pub = this->node->advertise<sensor_msgs::PointCloud2>("/mappedcloud", 1000);

    this->lidar_sub = this->node->subscribe("/os1_cloud_node/points", 1000, &Mapper::getTransformedPointCloud, this);

    this->frame_listener = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());

    ros::spin();

}

Mapper::~Mapper()
{

}

void Mapper::getTransformedPointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{

    tf::StampedTransform transform;
    sensor_msgs::PointCloud2 cloud_transformed, cloud_out;

    try{

        this->frame_listener->lookupTransform("map", "/os1_lidar", ros::Time(0), transform);

       Eigen::Matrix4f eigen_matrix;
       pcl_ros::transformAsMatrix(transform, eigen_matrix);
       pcl_ros::transformPointCloud(eigen_matrix, *cloud_in, cloud_out);
       //pcl_ros::transformPointCloud(eigen_matrix, *cloud_in, cloud_transformed);



       cloud_out.header.frame_id = "map";
       this->pc_pub.publish(cloud_out);



     }
   catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
     //ros::Duration(1.0).sleep();
     }


}
