#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>

class MaskToPointCloud
{
public:

    MaskToPointCloud()
    {
        focalX_norm  = 0.484f;
        focalY_norm  = 0.856f;

        centerX_norm = 0.495f;
        centerY_norm = 0.497f;

        maskReady  = false;
        depthReady = false;

        mask_sub_ =
            nh_.subscribe(
                "/filtered_cloth_mask",
                1,
                &MaskToPointCloud::maskCallback,
                this);

        depth_sub_ =
            nh_.subscribe(
                "/depth_float",
                1,
                &MaskToPointCloud::depthCallback,
                this);

        cloud_pub_ =
            nh_.advertise<
                sensor_msgs::PointCloud2>(
                "/obstacle_pointcloud",
                1);

        ROS_INFO(
            "MaskToPointCloud Started");
    }

private:

    ros::NodeHandle nh_;

    ros::Subscriber mask_sub_;
    ros::Subscriber depth_sub_;

    ros::Publisher cloud_pub_;

    cv::Mat maskImage;
    cv::Mat depthImage;

    bool maskReady;
    bool depthReady;

    float focalX_norm;
    float focalY_norm;

    float centerX_norm;
    float centerY_norm;

private:

    void maskCallback(
        const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(
                    msg,
                    sensor_msgs::image_encodings::MONO8);

            maskImage =
                cv_ptr->image.clone();

            maskReady = true;

            process();
        }
        catch(...)
        {
        }
    }

    void depthCallback(
        const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(
                    msg,
                    sensor_msgs::image_encodings::TYPE_32FC1);

            depthImage =
                cv_ptr->image.clone();

            depthReady = true;

            process();
        }
        catch(...)
        {
        }
    }

    void process()
    {
        if(!maskReady || !depthReady)
            return;

        if(maskImage.empty())
            return;

        if(depthImage.empty())
            return;

        int width  = depthImage.cols;
        int height = depthImage.rows;

        float focalX =
            focalX_norm * width;

        float focalY =
            focalY_norm * height;

        float centerX =
            centerX_norm * width;

        float centerY =
            centerY_norm * height;

        pcl::PointCloud<pcl::PointXYZ>
            cloud;

        cloud.header.frame_id =
            "camera_link";

        for(int row=0;
            row<height;
            row++)
        {
            for(int col=0;
                col<width;
                col++)
            {
                if(maskImage.at<uchar>(
                    row,col) == 0)
                {
                    continue;
                }

                float depth =
                    depthImage.at<float>(
                        row,col);

                if(std::isnan(depth))
                    continue;

                if(depth <= 0.1f)
                    continue;

                float X =
                    (col-centerX)
                    * depth
                    / focalX;

                float Y =
                    (row-centerY)
                    * depth
                    / focalY;

                float Z =
                    depth;

                cloud.points.push_back(
                    pcl::PointXYZ(
                        X,Y,Z));
            }
        }

        cloud.width =
            cloud.points.size();

        cloud.height = 1;

        sensor_msgs::PointCloud2 rosCloud;

        pcl::toROSMsg(
            cloud,
            rosCloud);

        rosCloud.header.stamp =
            ros::Time::now();

        rosCloud.header.frame_id =
            "camera_link";

        cloud_pub_.publish(
            rosCloud);

        ROS_INFO_STREAM(
            "Published obstacle cloud: "
            << cloud.points.size()
            << " points");
    }
};

int main(
    int argc,
    char** argv)
{
    ros::init(
        argc,
        argv,
        "mask_to_pointcloud_node");

    MaskToPointCloud node;

    ros::spin();

    return 0;
}