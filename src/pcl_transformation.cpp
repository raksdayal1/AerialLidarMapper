#include "pcl_transformation.h"

using namespace std;

PointTransform::PointTransform(int argc, char **argv)
{

    this->nodeName = "PCLTransform"; // Node name
    if(!ros::isInitialized())
    {
        ros::init(argc, argv, this->nodeName); //init the ros node
    }

    this->node.reset(new ros::NodeHandle(this->nodeName));

    this->GotReferenceLLA = false; // Flag to check if the origi LLA is set

    ros::Subscriber djigpsinit_sub;
    djigpsinit_sub = this->node->subscribe("/dji_sdk/gps_position", 5, &PointTransform::DJIGPS_InitLLA_cb, this);

    int measure_len = 25;// hardcoded for now. Input later
    this->counter = 0; // counter to keep count of msgs got.. need a better way to do this

    //init value for avgvalues of lla
    this->lat_avg = 0;
    this->long_avg = 0;
    this->alt_avg = 0;

    cout <<"Averaging over "<< measure_len <<" measurements to set origin" << endl;

    while (!this->GotReferenceLLA && ros::ok())
    {
        ros::spinOnce();

        if(this->counter > measure_len)
        {
            tf::Point LLA0;

            LLA0.setX(this->lat_avg*(M_PI/180)/this->counter);
            LLA0.setY(this->long_avg*(M_PI/180)/this->counter);
            LLA0.setZ(this->alt_avg/this->counter);

            cout << "LLA origin set to Lat0 = " << LLA0.getX() << ", Lon0 = " << LLA0.getY() << ", Alt0 = " << LLA0.getZ() << endl;

            LLA2ECEF(LLA0, this->ECEF0);

            this->GotReferenceLLA = true;

        }

    }

    djigpsinit_sub.shutdown(); // shutdown the subscriber to GPS after getting init lla

    // Subscribe to the DJI GPS data
    this->djiGps_sub = this->node->subscribe("/dji_sdk/gps_position",100, &PointTransform::DJIGPS_cb, this);

    //Subscribe to the DJI attitude data
    this->djiAtt_sub = this->node->subscribe("/dji_sdk/attitude",100, &PointTransform::DJIAtt_cb, this);

    //Subscribe to Lidar Point information
    this->lidarPC_sub = this->node->subscribe("/os1_cloud_node/points", 100, &PointTransform::LidarPC_cb, this);

    //Init the tf broadcaster
    this->djiinertial_tfbroadcaster = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
    this->djibodyfixed_tfbroadcaster = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
    this->lidar_tfbroadcaster = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    ros::spin();

}

PointTransform::~PointTransform()
{

}

void PointTransform::DJIAtt_cb(const geometry_msgs::QuaternionStampedConstPtr &dji_att_msg)
{
    double roll, pitch, yaw;
    tf::Quaternion q(dji_att_msg->quaternion.x, dji_att_msg->quaternion.y,
                     dji_att_msg->quaternion.z, dji_att_msg->quaternion.w);

    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);

    cout << "Roll = " << roll*180/M_PI << endl;
    cout << "Pitch = " << pitch*180/M_PI << endl;
    cout << "Yaw = " << yaw*180/M_PI << endl;

    tf::Vector3 v(0,0,0);

    tf::Transform body_transform(q, v);

    this->djibodyfixed_tfbroadcaster->sendTransform(tf::StampedTransform(body_transform,
                                                                         dji_att_msg->header.stamp,
                                                                         "/body_inertial",
                                                                         "/body_fixed"));


}

void PointTransform::LidarPC_cb(const sensor_msgs::PointCloud2ConstPtr &pc_msg)
{

    tf::Quaternion q1(tf::Vector3(0,0,1), M_PI), q2(tf::Vector3(0,1,0),-M_PI/2);
    tf::Quaternion qFixed;

    qFixed = q1*q2;
    tf::Vector3 vL(0,0,0);

    tf::Transform lidarTransform(qFixed, vL);

    this->lidar_tfbroadcaster->sendTransform(tf::StampedTransform(lidarTransform,
                                                                  pc_msg->header.stamp,
                                                                  "/body_fixed",
                                                                  "/os1_lidar"));

}

void PointTransform::DJIGPS_InitLLA_cb(const sensor_msgs::NavSatFixConstPtr &dji_gps_msg)
{
    cout <<"Got " << this->counter <<" gps measurements." << endl;

    this->lat_avg += dji_gps_msg->latitude;
    this->long_avg += dji_gps_msg->longitude;
    this->alt_avg += dji_gps_msg->altitude;

    this->counter += 1;
}

void PointTransform::DJIGPS_cb(const sensor_msgs::NavSatFixConstPtr &dji_gps_msg)
{
    tf::Point LLA, ECEF;

    LLA.setX(dji_gps_msg->latitude*M_PI/180);
    LLA.setY(dji_gps_msg->longitude*M_PI/180);
    LLA.setZ(dji_gps_msg->altitude);

    double f = (SEMI_MAJOR - SEMI_MINOR) / SEMI_MAJOR;           // Ellipsoid Flatness
    double f_inv = 1.0 / f;       // Inverse flattening

    double a_sq = SEMI_MAJOR * SEMI_MAJOR;
    double b_sq = SEMI_MINOR * SEMI_MINOR;
    double e_sq = f * (2 - f);

    double lambda = LLA.getX();
    double phi = LLA.getY();
    double s = sin(lambda);
    double N = SEMI_MAJOR / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    LLA2ECEF(LLA, ECEF);

    double xd, yd, zd;
    xd = ECEF.getX() - this->ECEF0.getX();
    yd = ECEF.getY() - this->ECEF0.getY();
    zd = ECEF.getZ() - this->ECEF0.getZ();

    double xEast, yNorth, zUp;

    // This is the matrix multiplication
    xEast = -sin_phi * xd + cos_phi * yd;
    yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
    zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

    //cout << "xEast " << xEast <<endl;
    //cout << "yNorth " << yNorth <<endl;
    //cout << "zUp " << zUp <<endl;


    // The body_inertial frame for the DJI M600 is FLU which has Xaxis along East, Yaxis is along North
    // when yaw = 0, pitch = 0, yaw = 0
    //tf::Quaternion q1(tf::Vector3(0,0,1), M_PI/2), q2(tf::Vector3(1,0,0), M_PI);

    tf::Quaternion q1(0,0,0,1);
    tf::Quaternion qt;

    qt = q1;//*q2;

    tf::Vector3 vt(xEast, yNorth, zUp);
    tf::Transform inert_transform(qt,vt);



    this->djiinertial_tfbroadcaster->sendTransform(tf::StampedTransform(inert_transform,
                                                                        dji_gps_msg->header.stamp,
                                                                        "map",
                                                                        "/body_inertial"));

}

void PointTransform::LLA2ECEF(tf::Point LLA, tf::Point &ECEF){

    float X, Y, Z;
    float lat, lon, alt;

    lat = LLA.getX();
    lon = LLA.getY();
    alt = LLA.getZ();

    X = (SEMI_MAJOR/sqrt(pow(cos(lat),2) + pow((SEMI_MINOR/SEMI_MAJOR),2)*pow(sin(lat),2)) + alt)*cos(lat)*cos(lon);

    Y = (SEMI_MAJOR/sqrt(pow(cos(lat),2) + pow((SEMI_MINOR/SEMI_MAJOR),2)*pow(sin(lat),2)) + alt)*cos(lat)*sin(lon);

    Z = (SEMI_MINOR/sqrt(pow((SEMI_MAJOR/SEMI_MINOR),2)*pow(cos(lat),2) + pow(sin(lat),2)) + alt)*sin(lat);

    ECEF.setX(X);
    ECEF.setY(Y);
    ECEF.setZ(Z);


}
