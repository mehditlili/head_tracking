#ifndef TRACKER_H
#define TRACKER_H

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/filter.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "common.h"
#include <ros/package.h>

using namespace pcl;
using namespace pcl::registration;
using namespace Eigen;
using namespace std;
using namespace cv;

class Tracker
{
public:
    Tracker();
    ~Tracker();
    void run(CloudPtr inputCloud, Mat inputImage);
    void extractHeadModel(Mat &image, CloudPtr inputCloud, int &m_cnt);
    CloudPtr getTransformedHeadModel();
    geometry_msgs::PoseStamped getHeadPose();
    void drawFace(Mat &image);



private:
    int width, height;   //image/pointcloud(organized) width and height
    bool isDebug;
    CloudPtr m_headModel;
    Affine3f m_headPose;
    int m_cnt;
    float fx, fy, cx, cy; //camera parameters
    CascadeClassifier cascade;
    vector<Rect> faces;
    string package_path;
    double scale;
    PointCloud<PointXYZRGB> final;
    PointCloud<PointXYZRGB> transformed_head;
    IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
    vector<int> indices;

};



#endif // TRACKER_H




