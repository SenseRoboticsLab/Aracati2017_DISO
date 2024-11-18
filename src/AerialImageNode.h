#ifndef AERIALIMAGENODE_H
#define AERIALIMAGENODE_H

#include "AerialImage.h"
#include "SonShape.h"
#include "CircularVector.h"

// ROS lib (roscpp)
#include <ros/ros.h>

// ROS Img transport plugin
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>

// Pose msg
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


class AerialImageNode
{
    AerialImage ai; // Aerial Image (ai)
    SonShape sonShape; // Manipulate sonar field of view on aerial image

    // ROS stuff
    ros::NodeHandle n;
    ros::Subscriber subGtPose;
    ros::Subscriber subOdomPose;
    ros::Subscriber subSonarPose;
    ros::Subscriber subSonar2Pose;
    ros::Subscriber subPC;

    image_transport::ImageTransport it;
    image_transport::Publisher pubAerialImgs;
    image_transport::Publisher pubSonAerial;

    std_msgs::Header aerialImgHeader;

    // UTM reference of vehicle position (Offset)
    bool hasUTMRef = false;
    Point2d utmRef;

    // Paths on aerial image
    CircularVector odomPts;
    CircularVector gtPts;
    CircularVector sonarPts;
    CircularVector sonar2Pts;
    double lastHeading = 0.0;
    double firstHeading = 0.0;

    bool getUTMRef();

    bool initROS();

    void odomPoseCallback(const geometry_msgs::PoseStamped &msg);

    void sonarPoseCallback(const geometry_msgs::PoseStamped &msg);

    void sonar2PoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);

    void gtPoseCallback(const geometry_msgs::PoseStamped &msg);

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void publishAerialImg();

public:
    AerialImageNode();

    void start();

};

#endif // AERIALIMAGENODE_H
