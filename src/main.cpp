#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/time.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>

#include <iostream>
#include <string>
#include "tracker.h"

using namespace std;
using namespace pcl;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
bool isDebug=true;
CloudPtr cFrame;

ros::Publisher pub_headModel;
ros::Publisher pub_headPose;
ros::Publisher pub_headPoseTrajectory;

Tracker *theTracker;  //Tracker class
int cnt = 0;  //counter

nav_msgs::Path headPath;


void callbackSync(const sensor_msgs::PointCloud2ConstPtr& inputCloud, const sensor_msgs::ImageConstPtr& inputImage)
{
    cout<<cnt<<"th frame!"<<endl;
    double lastT=pcl::getTime();
    //convert image message to opencv Mat format
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(inputImage, "rgb8");
    //convert cloud message to pointcloud format.
    cFrame.reset(new Cloud);
    pcl::fromROSMsg(*inputCloud, *cFrame);
    if (isDebug) cout<<"computation time for loading: "<<pcl::getTime()-lastT<<"s"<<endl;
    lastT=pcl::getTime();

    theTracker->run(cFrame, cv_ptr->image);
    if (isDebug) cout<<"computation time for tracking: "<<pcl::getTime()-lastT<<"s"<<endl;

    //publish the human head model point cloud with the tracked pose.(messagetype: sensor_msgs::PointCloud2)
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*theTracker->getTransformedHeadModel(), output);
    output.header.frame_id=inputCloud->header.frame_id;
    pub_headModel.publish (output);

    //publish the human head pose (messagetype: geometry_msgs::PoseStamped)
    geometry_msgs::PoseStamped headPose;
    headPose = theTracker->getHeadPose();
    headPose.header.frame_id = inputCloud->header.frame_id;
    pub_headPose.publish (headPose);

    //publish the trajectory of head pose (messagetype: nav_msgs::Path)
    headPath.header.frame_id = inputCloud->header.frame_id;
    if (int(cnt)%7==1)
         headPath.poses.push_back(headPose);
         pub_headPoseTrajectory.publish(headPath);

    if (isDebug) cout<<"Total computation time: "<<pcl::getTime()-lastT<<"s"<<endl;
    cnt++;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "headTrack");
    ros::NodeHandle n;

    // publishers
    pub_headModel = n.advertise<sensor_msgs::PointCloud2>("/dhri/headModel", 1000);
    pub_headPose = n.advertise<geometry_msgs::PoseStamped>("/dhri/headPose", 1000);
    pub_headPoseTrajectory = n.advertise<nav_msgs::Path>("/dhri/headPoseTrajectory", 1000);

    theTracker = new Tracker;

    // message subscriber with synchronizer, pointCloud and Image message at each time step are
    //synchronized and feeded into callbackSync fucntion
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pointCloud(n, "/camera/depth_registered/points", 1000);
    message_filters::Subscriber<Image> sub_image(n, "/camera/rgb/image_rect_color", 1000);
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_pointCloud, sub_image);
    sync.registerCallback(boost::bind(&callbackSync, _1, _2));
    ros::spin();
    return 0;
}


