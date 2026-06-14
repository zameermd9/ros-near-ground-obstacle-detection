#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

class HeightBinaryMorphology
{
public:

    HeightBinaryMorphology()
    {
        heightThresh     = 0.0f;
        floorNoiseMargin = 0.04f;

        erodeIter  = 1;
        dilateIter = 4;

        minArea = 100;

        frameCount = 0;

        height_sub_ =
            nh_.subscribe(
                "/ground_height",
                1,
                &HeightBinaryMorphology::heightCallback,
                this);

        binary_pub_ =
            nh_.advertise<sensor_msgs::Image>(
                "/height_binary_mask",
                1);

        contour_pub_ =
            nh_.advertise<sensor_msgs::Image>(
                "/height_contours",
                1);

        ROS_INFO(
            "HeightBinaryMorphology Node Started");
    }

private:

    ros::NodeHandle nh_;

    ros::Subscriber height_sub_;

    ros::Publisher binary_pub_;
    ros::Publisher contour_pub_;

    float heightThresh;
    float floorNoiseMargin;

    int erodeIter;
    int dilateIter;

    int minArea;

    uint32_t frameCount;

private:

    void heightCallback(
        const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(
                    msg,
                    sensor_msgs::image_encodings::TYPE_32FC1);

            const cv::Mat& heightImage =
                cv_ptr->image;

            int width  = heightImage.cols;
            int height = heightImage.rows;

            frameCount++;

            int obstaclePixels = 0;
            int floorPixels    = 0;

            cv::Mat binaryMask(
                height,
                width,
                CV_8UC1,
                cv::Scalar(0));

            for(int row=0; row<height; row++)
            {
                for(int col=0; col<width; col++)
                {
                    float pixelHeight =
                        heightImage.at<float>(
                            row,col);

                    if(std::isnan(pixelHeight))
                        pixelHeight = 0.0f;

                    if(std::abs(pixelHeight)
                       < floorNoiseMargin)
                    {
                        pixelHeight = 0.0f;
                    }

                    if(pixelHeight >
                       heightThresh)
                    {
                        binaryMask.at<uchar>(
                            row,col) = 255;

                        obstaclePixels++;
                    }
                    else
                    {
                        floorPixels++;
                    }
                }
            }

            // Morphology disabled
            // Same as MIRA implementation

            std::vector<
                std::vector<cv::Point>> contours;

            cv::findContours(
                binaryMask.clone(),
                contours,
                cv::RETR_EXTERNAL,
                cv::CHAIN_APPROX_SIMPLE);

            int contoursDetected =
                static_cast<int>(
                    contours.size());

            int validContours    = 0;
            int rejectedContours = 0;

            cv::Mat filteredMask(
                height,
                width,
                CV_8UC1,
                cv::Scalar(0));

            cv::Mat contourImage(
                height,
                width,
                CV_8UC1,
                cv::Scalar(0));

            for(auto& contour :
                contours)
            {
                double area =
                    cv::contourArea(
                        contour);

                if(area > minArea)
                {
                    validContours++;

                    cv::drawContours(
                        filteredMask,
                        std::vector<
                        std::vector<cv::Point>>
                        {contour},
                        -1,
                        cv::Scalar(255),
                        cv::FILLED);

                    cv::drawContours(
                        contourImage,
                        std::vector<
                        std::vector<cv::Point>>
                        {contour},
                        -1,
                        cv::Scalar(255),
                        2);
                }
                else
                {
                    rejectedContours++;
                }
            }

            cv_bridge::CvImage binaryMsg;
            binaryMsg.header   = msg->header;
            binaryMsg.encoding =
                sensor_msgs::image_encodings::MONO8;
            binaryMsg.image    = filteredMask;

            binary_pub_.publish(
                *binaryMsg.toImageMsg());

            cv_bridge::CvImage contourMsg;
            contourMsg.header   = msg->header;
            contourMsg.encoding =
                sensor_msgs::image_encodings::MONO8;
            contourMsg.image    = contourImage;

            contour_pub_.publish(
                *contourMsg.toImageMsg());

            ROS_INFO_STREAM(
                "Frame "
                << frameCount
                << " | Contours: "
                << contoursDetected
                << " | Valid: "
                << validContours);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("%s", e.what());
        }
    }
};

int main(
    int argc,
    char** argv)
{
    ros::init(
        argc,
        argv,
        "height_binary_morphology_node");

    HeightBinaryMorphology node;

    ros::spin();

    return 0;
}