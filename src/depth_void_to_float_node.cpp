#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DepthVoidToFloat
{
public:
    DepthVoidToFloat()
    {
        depth_sub_ = nh_.subscribe(
            "/camera/depth/image_raw",
            1,
            &DepthVoidToFloat::depthCallback,
            this);

        depth_pub_ = nh_.advertise<sensor_msgs::Image>(
            "/depth_float",
            1);

        ROS_INFO("DepthVoidToFloat Node Started");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber depth_sub_;
    ros::Publisher depth_pub_;

    void depthCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr;

            cv_ptr = cv_bridge::toCvShare(
                msg,
                sensor_msgs::image_encodings::TYPE_16UC1);

            cv::Mat depth_float(
                cv_ptr->image.rows,
                cv_ptr->image.cols,
                CV_32FC1);

            for(int r = 0; r < cv_ptr->image.rows; r++)
            {
                for(int c = 0; c < cv_ptr->image.cols; c++)
                {
                    uint16_t depth_mm =
                        cv_ptr->image.at<uint16_t>(r,c);

                    if(depth_mm == 0)
                    {
                        depth_float.at<float>(r,c) =
                            std::numeric_limits<float>::quiet_NaN();
                    }
                    else
                    {
                        depth_float.at<float>(r,c) =
                            static_cast<float>(depth_mm) / 1000.0f;
                    }
                }
            }

            cv_bridge::CvImage out_msg;

            out_msg.header = msg->header;
            out_msg.encoding =
                sensor_msgs::image_encodings::TYPE_32FC1;
            out_msg.image = depth_float;

            depth_pub_.publish(
                *out_msg.toImageMsg());
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("%s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_void_to_float_node");

    DepthVoidToFloat node;

    ros::spin();

    return 0;
}
