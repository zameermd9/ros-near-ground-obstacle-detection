#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <vector>
#include <random>
#include <cmath>
#include <limits>

class GroundHeightFromDepth
{
public:
    GroundHeightFromDepth()
    {
        focalX_norm = 0.484f;
        focalY_norm = 0.856f;
        centerX_norm = 0.495f;
        centerY_norm = 0.497f;

        ransacIterations = 300;
        inlierThreshold = 0.01f;

        planeUpdateInterval = 10;
        frameCounter = 0;

        floorPlaneFound = false;

        minHeight = 0.07f;
        maxHeight = 0.25f;

        depth_sub_ =
            nh_.subscribe("/depth_float",
                          1,
                          &GroundHeightFromDepth::depthCallback,
                          this);

        height_pub_ =
            nh_.advertise<sensor_msgs::Image>(
                "/ground_height",
                1);

        ROS_INFO("GroundHeightFromDepth Node Started");
    }

private:

    ros::NodeHandle nh_;

    ros::Subscriber depth_sub_;
    ros::Publisher height_pub_;

    float focalX_norm;
    float focalY_norm;
    float centerX_norm;
    float centerY_norm;

    int ransacIterations;
    float inlierThreshold;

    int planeUpdateInterval;
    int frameCounter;

    bool floorPlaneFound;

    cv::Vec4f floorPlane;

    float minHeight;
    float maxHeight;

private:

    void depthCallback(
        const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(
                    msg,
                    sensor_msgs::image_encodings::TYPE_32FC1);

            const cv::Mat& depthImage =
                cv_ptr->image;

            int width  = depthImage.cols;
            int height = depthImage.rows;

            float focalX  = focalX_norm  * width;
            float focalY  = focalY_norm  * height;

            float centerX = centerX_norm * width;
            float centerY = centerY_norm * height;

            float invFocalX = 1.0f / focalX;
            float invFocalY = 1.0f / focalY;

            if(frameCounter % planeUpdateInterval == 0)
            {
                findFloorPlane(
                    depthImage,
                    width,
                    height,
                    centerX,
                    centerY,
                    invFocalX,
                    invFocalY);
            }

            if(!floorPlaneFound)
                return;

            cv::Mat heightImage =
                calculateHeightImage(
                    depthImage,
                    width,
                    height,
                    centerX,
                    centerY,
                    invFocalX,
                    invFocalY);

            cv_bridge::CvImage outMsg;

            outMsg.header = msg->header;
            outMsg.encoding =
                sensor_msgs::image_encodings::TYPE_32FC1;

            outMsg.image = heightImage;

            height_pub_.publish(
                *outMsg.toImageMsg());

            frameCounter++;

            ROS_INFO_STREAM(
                "Frame "
                << frameCounter
                << " processed");
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("%s", e.what());
        }
    }

    cv::Point3f pixelTo3D(
        int col,
        int row,
        float depth,
        float centerX,
        float centerY,
        float invFocalX,
        float invFocalY)
    {
        float X =
            (col - centerX)
            * depth
            * invFocalX;

        float Y =
            (row - centerY)
            * depth
            * invFocalY;

        float Z = depth;

        return cv::Point3f(X,Y,Z);
    }

    void findFloorPlane(
        const cv::Mat& depthImage,
        int width,
        int height,
        float centerX,
        float centerY,
        float invFocalX,
        float invFocalY)
    {
        std::vector<cv::Point3f> points3D;

        for(int row=(int)(height*0.6);
            row<height;
            row+=4)
        {
            for(int col=0;
                col<width;
                col+=4)
            {
                float depth =
                    depthImage.at<float>(
                        row,col);

                if(std::isnan(depth))
                    continue;

                if(depth < 0.3f ||
                   depth > 4.0f)
                    continue;

                points3D.push_back(
                    pixelTo3D(
                        col,row,depth,
                        centerX,
                        centerY,
                        invFocalX,
                        invFocalY));
            }
        }

        if(points3D.size() < 200)
            return;

        std::default_random_engine rng;

        std::uniform_int_distribution<size_t>
            randomIndex(
                0,
                points3D.size()-1);

        cv::Vec4f bestPlane;

        int bestInliers = 0;

        for(int i=0;
            i<ransacIterations;
            i++)
        {
            cv::Point3f p1 =
                points3D[randomIndex(rng)];

            cv::Point3f p2 =
                points3D[randomIndex(rng)];

            cv::Point3f p3 =
                points3D[randomIndex(rng)];

            cv::Vec4f plane =
                fitPlaneFrom3Points(
                    p1,p2,p3);

            if(plane[0]==0 &&
               plane[1]==0 &&
               plane[2]==0)
                continue;

            int inliers =
                countInliers(
                    points3D,
                    plane);

            if(inliers >
               bestInliers)
            {
                bestInliers =
                    inliers;

                bestPlane =
                    plane;
            }
        }

        floorPlane = bestPlane;
        floorPlaneFound = true;

        ROS_INFO(
            "Floor plane updated");
    }

    cv::Vec4f fitPlaneFrom3Points(
        cv::Point3f p1,
        cv::Point3f p2,
        cv::Point3f p3)
    {
        cv::Point3f v1 =
            p2-p1;

        cv::Point3f v2 =
            p3-p1;

        cv::Point3f normal =
            v1.cross(v2);

        float lenSq =
            normal.x*normal.x +
            normal.y*normal.y +
            normal.z*normal.z;

        if(lenSq < 1e-6f)
            return cv::Vec4f(
                0,0,0,0);

        float a=normal.x;
        float b=normal.y;
        float c=normal.z;

        float d =
            -(a*p1.x+
              b*p1.y+
              c*p1.z);

        return cv::Vec4f(
            a,b,c,d);
    }

    int countInliers(
        const std::vector<cv::Point3f>& points,
        cv::Vec4f plane)
    {
        float a=plane[0];
        float b=plane[1];
        float c=plane[2];
        float d=plane[3];

        float normSq =
            a*a+b*b+c*c;

        float threshSq =
            inlierThreshold*
            inlierThreshold*
            normSq;

        int count = 0;

        for(const auto& p :
            points)
        {
            float dist =
                a*p.x +
                b*p.y +
                c*p.z +
                d;

            if(dist*dist <
               threshSq)
            {
                count++;
            }
        }

        return count;
    }

    cv::Mat calculateHeightImage(
        const cv::Mat& depthImage,
        int width,
        int height,
        float centerX,
        float centerY,
        float invFocalX,
        float invFocalY)
    {
        cv::Mat heightImage(
            height,
            width,
            CV_32FC1,
            cv::Scalar(0));

        float a =
            floorPlane[0];

        float b =
            floorPlane[1];

        float c =
            floorPlane[2];

        float d =
            floorPlane[3];

        float normalizer =
            1.0f /
            std::sqrt(
                a*a+b*b+c*c);

        for(int row=0;
            row<height;
            row++)
        {
            for(int col=0;
                col<width;
                col++)
            {
                float depth =
                    depthImage.at<float>(
                        row,col);

                if(std::isnan(depth))
                    continue;

                if(depth < 0.3f ||
                   depth > 4.0f)
                    continue;

                cv::Point3f pt =
                    pixelTo3D(
                        col,row,
                        depth,
                        centerX,
                        centerY,
                        invFocalX,
                        invFocalY);

                float h =
                    std::abs(
                        a*pt.x +
                        b*pt.y +
                        c*pt.z +
                        d)
                    * normalizer;

                if(h < 0.05f)
                    h = 0.0f;

                if(h >= minHeight &&
                   h <= maxHeight)
                {
                    heightImage.at<float>(
                        row,col) = h;
                }
            }
        }

        return heightImage;
    }
};

int main(
    int argc,
    char** argv)
{
    ros::init(
        argc,
        argv,
        "ground_height_from_depth_node");

    GroundHeightFromDepth node;

    ros::spin();

    return 0;
}