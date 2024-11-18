#include "AerialImageNode.h"
#include <tf2/utils.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>

bool AerialImageNode::getUTMRef()
{
    ROS_INFO("Waiting first GPS msg to estimate UTM ref.");
    sensor_msgs::NavSatFix::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/dgps");

    if (msg != nullptr) {
        // Get UTM Ref
        Point2d latLong(-32.025024, -52.106634);
        utmRef = latLon2UTM(latLong);
        hasUTMRef = true;
        return true;
    }
    return false;
}

bool AerialImageNode::initROS()
{
    ros::NodeHandle nh("~");

    // Get parameters:
    string aerial_img_path;
    if (!nh.getParam("aerial_image_path", aerial_img_path)) {
        ROS_ERROR("Missing parameter aerial_image_path point to the .yaml file.");
        return false;
    }

    // Load aerial image
    if (!ai.loadMap(aerial_img_path)) {
        ROS_ERROR("It could not load the aerial image: %s", aerial_img_path.c_str());
        return false;
    }
    //  ai.resizeToMaxCols(800);

    // Load sonar FoV shape
    double sonOpenning, sonMinRange, sonMaxRange;
    nh.param<double>("son_fov_openning", sonOpenning, 130.0); // BlueView P900-130
    nh.param<double>("son_min_range", sonMinRange, 0.4); // BlueView P900 can be set from 0.4m to 100m
    nh.param<double>("son_max_range", sonMaxRange,
                     50.0); // BlueView P900 can be set from 0.4m to 100m (aracati2017 use 50m)

    sonShape.initShape(sonOpenning, sonMaxRange, sonMinRange);

    // Topic Subscriptions
    subGtPose = n.subscribe("/pose_gt", 5, &AerialImageNode::gtPoseCallback, this);

    subOdomPose = n.subscribe("/odom", 5, &AerialImageNode::odomPoseCallback, this);

    subSonarPose = n.subscribe("/direct_sonar/pose_draw", 5, &AerialImageNode::sonarPoseCallback, this);

    subSonar2Pose = n.subscribe("/bruce/slam/slam/pose", 5, &AerialImageNode::sonar2PoseCallback, this);

    subPC = n.subscribe("/direct_sonar/point_cloud", 5, &AerialImageNode::pointCloudCallback, this);

    // Topic Advertisements
    pubAerialImgs = it.advertise("/aerial_img", 1);
    pubSonAerial = it.advertise("/son_aerial", 1);

    // Get a UTM reference to the vehicle position
    // We assume the first DGPS msg (Ground Truth)
    // match the first vehicle position (It is true
    // on dataset aracati2017).

    // Otherwise the time difference between gps
    // and first vehicle position should be take
    // in account and the UTMRef should be interpolated
    // to get an approximated UTMRef.

    // It is not a good idea to use UTM in ROS, especially
    // on TF tree. We had some problems working
    // with high values, so we decided to remove the
    // UTM offset in the dataset.
    getUTMRef(); // This may wait forever (No timeout)

    aerialImgHeader.frame_id = "aerial_image";
    aerialImgHeader.seq = 0;

    return true;
}

void AerialImageNode::odomPoseCallback(const geometry_msgs::PoseStamped &msg)
{
    if (hasUTMRef) {
        Point2d UTMSonPosition = utmRef + Point2d(msg.pose.position.x, msg.pose.position.y);

        odomPts.push(UTMSonPosition);
    }
}

void AerialImageNode::gtPoseCallback(const geometry_msgs::PoseStamped &msg)
{
    if (hasUTMRef) {
        Point2d UTMSonPosition = utmRef + Point2d(msg.pose.position.x, msg.pose.position.y);


        gtPts.push(UTMSonPosition);
        lastHeading = tf2::getYaw(msg.pose.orientation);
    }
    else{
        firstHeading = tf2::getYaw(msg.pose.orientation);
        utmRef.x += msg.pose.position.x;
        utmRef.y += msg.pose.position.y;
    }
}

void AerialImageNode::publishAerialImg()
{
    Mat aerialImg = ai.getMapImg().clone();

    aerialImgHeader.stamp = ros::Time::now();

    // Too many point so we drop a few
    // when drawing...


    // Draw Odom Lines on aerial image
    // for (uint i = 10; i < odomPts.size(); i += 10)
    //     line(aerialImg, ai.UTM2Img(odomPts[i - 10]), ai.UTM2Img(odomPts[i]), Scalar(0, 0, 255), 2);

    // Draw GT Lines on aerial image
    for (uint i = 10; i < gtPts.size(); i += 10)
        line(aerialImg, ai.UTM2Img(gtPts[i - 10]), ai.UTM2Img(gtPts[i]), Scalar(255, 0, 255), 2);


    for (uint i = 10; i < sonarPts.size(); i += 10) {
        line(aerialImg, ai.UTM2Img(sonarPts[i - 10]), ai.UTM2Img(sonarPts[i]), Scalar(255, 0, 0), 2);
    }

    for (uint i = 10; i < sonar2Pts.size(); i += 10) {
        line(aerialImg, ai.UTM2Img(sonar2Pts[i - 10]), ai.UTM2Img(sonar2Pts[i]), Scalar(0, 2255, 0), 2);
    }
    // ROS_INFO_STREAM("draw sonar2 points: "<<sonar2Pts.size());

    ai.setMapIng(aerialImg.clone());

    if (!gtPts.empty()) {
        const Point2d &UTMSonPosition = gtPts.last();

        // sonShape.drawPoly(aerialImg,ai,
        //                   UTMSonPosition,
        //                   lastHeading,
        //                   Scalar(255,0,255),
        //                   2);

        // const Point2d &UTMSonPosition_direct = sonarPts.last();
        // sonShape.drawPoly(aerialImg,ai,
        //                   UTMSonPosition_direct,
        //                   lastHeading,
        //                   Scalar(0,0,255),
        //                   2);
        // Publish sonar aerial image
        Mat sonAerialImg = sonShape.cropSonShape(ai, UTMSonPosition, lastHeading);

        sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                                          sonAerialImg).toImageMsg();

        imgMsg->header = aerialImgHeader;
        pubSonAerial.publish(imgMsg);
    }

    // Publish aerial image
    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                                      aerialImg).toImageMsg();

    imgMsg->header = aerialImgHeader;
    pubAerialImgs.publish(imgMsg);


    aerialImgHeader.seq++;
}

AerialImageNode::AerialImageNode() : it(n), odomPts(60000), gtPts(60000), sonarPts(60000), sonar2Pts(60000)
{

}

void AerialImageNode::start()
{
    initROS();
    ros::Rate r(4);

    while (ros::ok()) {
        publishAerialImg();
        ros::spinOnce();
        r.sleep();
    }

}

void AerialImageNode::sonarPoseCallback(const geometry_msgs::PoseStamped &msg)
{
    if (hasUTMRef) {
        Point2d UTMSonPosition = utmRef + Point2d(msg.pose.position.y, msg.pose.position.x);

        sonarPts.push(UTMSonPosition);

        // const Point2d &UTMSonPosition = gtPts.last();
        cv::Mat img = ai.getMapPCImg().clone();
        sonShape.drawPoly(img,ai,
                          UTMSonPosition,
                          lastHeading,
                          Scalar(255,0,0),
                          2);
        ai.setMapIng(img);
    }
}

void AerialImageNode::sonar2PoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    if (hasUTMRef) {
        Point2d UTMSonPosition = utmRef + Point2d(msg.pose.pose.position.y, -msg.pose.pose.position.x);

        sonar2Pts.push(UTMSonPosition);
    }
}

void AerialImageNode::pointCloudCallback(const sensor_msgs::PointCloud2_<allocator<void>>::ConstPtr &msg)
{
    // return;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    cv::Mat img = ai.getMapOriImg().clone();
    vector<Point2d> points;
    for (auto p: cloud) {
        Point2d p_map(p.y,p.x);
        // 0.305
        double theta = (-90/180.0) * M_PI ;
        double s= sin(firstHeading+theta), c=cos(firstHeading+theta);



        // We flip y signal because image y grows down
        // On UTM to img it will flip again
        p_map.x =   (p.x*c - p.y*s);
        p_map.y = -(p.x*s + p.y*c);

        p_map.x +=  utmRef.x;
        p_map.y +=  utmRef.y;



        points.push_back(p_map);
    }
    // sonShape.getPoints(points,firstHeading,utmRef);

    for(auto p:points){
        cv::Mat tempImg = img.clone();

        // Draw a solid circle on the temporary image
        cv::circle(tempImg, ai.UTM2Img(p), 1, Scalar(0, 255, 0), 1);

        // Alpha blending
        float alpha = 0.6; // Set the level of transparency [0, 1]
        cv::addWeighted(tempImg, alpha, img, 1.0 - alpha, 0, img);
        // cv::circle(img, ai.UTM2Img(p), 1, Scalar(0, 255, 0), 1);
    }


    // const Point2d &UTMSonPosition = gtPts.last();
    //
    // sonShape.drawPoly(img,ai,
    //                   UTMSonPosition,
    //                   lastHeading,
    //                   Scalar(255,0,255),
    //                   2);

    ai.setMapPCIng(img);
    ROS_INFO_STREAM("draw "<<cloud.points.size()<<" points");
}

