#include "tracker.h"
#include <Eigen/Geometry>
using namespace pcl;


Tracker::Tracker()
{
    width = 320;
    height = 240;
    fx= 285.17f; fy= 285.17f; cx=159.5; cy=119.5f; //asus camera parameters for 320x240 resolution
    isDebug = true;
    m_cnt = 0;
    m_headModel.reset(new Cloud);
    m_headPose=Affine3f::Identity();
    package_path = ros::package::getPath("head_tracking");
    scale = 1.0;
    cout << "Loading: "<<package_path+"/data/haarcascade_face.xml"<<endl;
    if( !cascade.load(package_path+"/data/haarcascade_face.xml") )
    {
        cerr << "ERROR: Could not load classifier cascade" << endl;
        exit(0);
    }
    namedWindow("image");
}

Tracker::~Tracker()
{

}

void Tracker::run(CloudPtr inputCloud, Mat inputImage)
{
    cout<<"Timestamp: "<<inputCloud->header.stamp<<endl;
    cout <<"Cloud width height <<"<<inputCloud->width<<"/"<<inputCloud->height<<endl;
    //TODO: for first frame, detect face from inputImage, and extract corresponding head Point cloud
    extractHeadModel(inputImage, inputCloud, m_cnt);
    //TODO: estimate the current head pose with Iterative Closest Point algorithm from PCL
    // Only proceed when a face model is available (m_cnt >0)
    if(m_cnt)
    {
        //ICP tracking here
        //removeNaNFromPointCloud(*inputCloud, *inputCloud, indices);
        icp.setInputCloud(m_headModel);
        icp.setInputTarget(inputCloud);
        icp.align(final);
        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
        m_headPose*= icp.getFinalTransformation();
        m_cnt++;
    }
    imshow("image", inputImage);
    waitKey(2);
}

void Tracker::extractHeadModel(Mat &image, CloudPtr inputCloud, int &m_cnt)
{
    //your code to detect face and extract head
    //equalizeHist(image, image);
    cascade.detectMultiScale(image, faces,
        1.1, 2, 0
        //|CASCADE_FIND_BIGGEST_OBJECT
        //|CASCADE_DO_ROUGH_SEARCH
        |CASCADE_SCALE_IMAGE,
        Size(30, 30) );
    if (faces.size())
    {
        drawFace(image);
        cout << "Face detected at "<<faces[0].x<<"/"<<faces[0].y<<endl;
        cout << m_cnt<<endl;
        if (m_cnt == 0)
        {
            m_headModel.reset(new PointCloud<PointXYZRGB>(inputCloud->width, inputCloud->height));
            double x_val,y_val, z_val;
            for(int y=0;y<faces[0].height;y++)
            {
                for(int x=0;x<faces[0].width;x++)
                {
                    x_val = inputCloud->at(x+faces[0].x, y+faces[0].y).x;
                    y_val = inputCloud->at(x+faces[0].x, y+faces[0].y).y;
                    z_val = inputCloud->at(x+faces[0].x, y+faces[0].y).z;
                    if(!(x_val!=x_val || y_val !=y_val || z_val!=z_val))
                    {
                     m_headModel->at(x, y) = inputCloud->at(x+faces[0].x, y+faces[0].y);
                    }
                     else
                    {
                     m_headModel->at(x, y).x = 0;
                     m_headModel->at(x, y).y = 0;
                     m_headModel->at(x, y).z = 0;
                    }
                }
            }

            //Get centroid of face and set it as pose
            int centroid_x = faces[0].width/2;
            int centroid_y = faces[0].height/2;
            m_headPose = Translation3f(m_headModel->at(centroid_x, centroid_y).x,
                                       m_headModel->at(centroid_x, centroid_y).y,
                                       m_headModel->at(centroid_x, centroid_y).z);

            //removeNaNFromPointCloud(*m_headModel, *m_headModel, indices);
            for(int y=0;y<m_headModel->height;y++)
            {
                for(int x=0;x<m_headModel->width;x++)
                {
                    cout<<m_headModel->at(x, y)<<endl;
                }
            }
            m_cnt++;
        }
    }
}

void Tracker::drawFace(Mat &image)
{
    Rect r = faces[0];
    Point center;
    Scalar color = Scalar(255,128,0);
    int radius;

    double aspect_ratio = (double)r.width/r.height;
    if( 0.75 < aspect_ratio && aspect_ratio < 1.3 )
    {
        center.x = cvRound((r.x + r.width*0.5)*scale);
        center.y = cvRound((r.y + r.height*0.5)*scale);
        radius = cvRound((r.width + r.height)*0.25*scale);
        circle( image, center, radius, color, 3, 8, 0 );
    }
    else
        rectangle( image, cvPoint(cvRound(r.x*scale), cvRound(r.y*scale)),
                   cvPoint(cvRound((r.x + r.width-1)*scale), cvRound((r.y + r.height-1)*scale)),
                   color, 3, 8, 0);
}

CloudPtr Tracker::getTransformedHeadModel()
{
    CloudPtr temp;
    temp.reset(new Cloud);
    transformPointCloud(*m_headModel, *temp, m_headPose);
    return temp;
}

geometry_msgs::PoseStamped  Tracker::getHeadPose()
{
    geometry_msgs::PoseStamped poseOutput;
    poseOutput.pose.position.x = m_headPose(0,3);
    poseOutput.pose.position.y = m_headPose(1,3);
    poseOutput.pose.position.z = m_headPose(2,3);
    Quaternion<float> q(m_headPose.rotation()); q.normalize();
    poseOutput.pose.orientation.x = q.x();
    poseOutput.pose.orientation.y = q.y();
    poseOutput.pose.orientation.z = q.z();
    poseOutput.pose.orientation.w = q.w();
    return poseOutput;
}
