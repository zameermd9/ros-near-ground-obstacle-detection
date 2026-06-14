#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

class MultiFilterContourProcessor
{
public:

    MultiFilterContourProcessor()
    {
        dilateCount        = 2;
        minFramesConfirm   = 5;
        stripWidth         = 100;
        stripHeight        = 40;
        floorSaturation    = 50;

        rgbReady = false;

        rgb_sub_ =
            nh_.subscribe(
                "/camera/rgb/image_raw",
                1,
                &MultiFilterContourProcessor::rgbCallback,
                this);

        mask_sub_ =
            nh_.subscribe(
                "/height_binary_mask",
                1,
                &MultiFilterContourProcessor::maskCallback,
                this);

        output_pub_ =
            nh_.advertise<sensor_msgs::Image>(
                "/filtered_cloth_mask",
                1);

        ROS_INFO(
            "MultiFilterContourProcessor Started");
    }

private:

    ros::NodeHandle nh_;

    ros::Subscriber rgb_sub_;
    ros::Subscriber mask_sub_;

    ros::Publisher output_pub_;

    cv::Mat hsvImage;
    bool rgbReady;

    int imageWidth;
    int imageHeight;

    int dilateCount;
    int minFramesConfirm;

    int stripWidth;
    int stripHeight;

    int floorSaturation;

    cv::Mat persistenceMap;

private:

    //----------------------------------------------------
    // RGB CALLBACK
    //----------------------------------------------------

    void rgbCallback(
        const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(
                    msg,
                    sensor_msgs::image_encodings::BGR8);

            cv::Mat bgr =
                cv_ptr->image.clone();

            imageWidth  = bgr.cols;
            imageHeight = bgr.rows;

            cv::cvtColor(
                bgr,
                hsvImage,
                cv::COLOR_BGR2HSV);

            rgbReady = true;
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("%s", e.what());
        }
    }

    //----------------------------------------------------
    // FLOOR COLOR LEARNING
    //----------------------------------------------------

    void learnFloorColor()
    {
        if(hsvImage.empty())
            return;

        int x =
            (hsvImage.cols - stripWidth)/2;

        int y =
            hsvImage.rows - stripHeight;

        x = std::max(0,x);
        y = std::max(0,y);

        int w =
            std::min(
                stripWidth,
                hsvImage.cols-x);

        int h =
            std::min(
                stripHeight,
                hsvImage.rows-y);

        cv::Rect roi(
            x,y,w,h);

        cv::Mat strip =
            hsvImage(roi);

        std::vector<int> sat;

        for(int r=0;r<strip.rows;r++)
        {
            for(int c=0;c<strip.cols;c++)
            {
                sat.push_back(
                    strip.at<cv::Vec3b>(r,c)[1]);
            }
        }

        if(sat.empty())
            return;

        std::sort(
            sat.begin(),
            sat.end());

        floorSaturation =
            sat[sat.size()/2];
    }

    //----------------------------------------------------
    // KMEANS FILTER
    //----------------------------------------------------

    cv::Mat separateObjectFromFloor(
        const cv::Mat& regionMask)
    {
        cv::Mat result =
            cv::Mat::zeros(
                hsvImage.rows,
                hsvImage.cols,
                CV_8UC1);

        std::vector<cv::Vec3f> colors;
        std::vector<cv::Point> locations;

        for(int r=0;r<hsvImage.rows;r++)
        {
            for(int c=0;c<hsvImage.cols;c++)
            {
                if(regionMask.at<uchar>(r,c)>0)
                {
                    cv::Vec3b p =
                        hsvImage.at<cv::Vec3b>(r,c);

                    colors.push_back(
                        cv::Vec3f(
                            p[0],p[1],p[2]));

                    locations.push_back(
                        cv::Point(c,r));
                }
            }
        }

        if(colors.size() < 2)
            return result;

        cv::Mat data(
            colors.size(),
            3,
            CV_32F);

        for(size_t i=0;i<colors.size();i++)
        {
            data.at<float>(i,0)=colors[i][0];
            data.at<float>(i,1)=colors[i][1];
            data.at<float>(i,2)=colors[i][2];
        }

        cv::Mat labels;
        cv::Mat centers;

        cv::kmeans(
            data,
            2,
            labels,
            cv::TermCriteria(
                cv::TermCriteria::EPS+
                cv::TermCriteria::COUNT,
                100,
                1.0),
            3,
            cv::KMEANS_RANDOM_CENTERS,
            centers);

        int objectCluster =
            (centers.at<float>(0,1)
             >
             centers.at<float>(1,1))
            ? 0 : 1;

        for(size_t i=0;i<locations.size();i++)
        {
            bool objectPixel =
                labels.at<int>(i,0)
                ==
                objectCluster;

            bool notFloor =
                colors[i][1]
                >
                floorSaturation;

            if(objectPixel && notFloor)
            {
                result.at<uchar>(
                    locations[i].y,
                    locations[i].x)
                    = 255;
            }
        }

        return result;
    }

    //----------------------------------------------------
    // TEMPORAL PERSISTENCE
    //----------------------------------------------------

    cv::Mat confirmPersistence(
        const cv::Mat& current)
    {
        if(persistenceMap.empty())
        {
            persistenceMap =
                cv::Mat::zeros(
                    current.rows,
                    current.cols,
                    CV_32SC1);
        }

        cv::Mat confirmed =
            cv::Mat::zeros(
                current.rows,
                current.cols,
                CV_8UC1);

        for(int r=0;r<current.rows;r++)
        {
            for(int c=0;c<current.cols;c++)
            {
                if(current.at<uchar>(r,c))
                {
                    persistenceMap
                        .at<int>(r,c)++;
                }
                else
                {
                    persistenceMap
                        .at<int>(r,c)=0;
                }

                if(
                    persistenceMap
                    .at<int>(r,c)
                    >=
                    minFramesConfirm)
                {
                    confirmed
                        .at<uchar>(r,c)
                        =255;
                }
            }
        }

        return confirmed;
    }

    //----------------------------------------------------
    // MASK CALLBACK
    //----------------------------------------------------

    void maskCallback(
        const sensor_msgs::ImageConstPtr& msg)
    {
        if(!rgbReady)
            return;

        try
        {
            cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(
                    msg,
                    sensor_msgs::image_encodings::MONO8);

            cv::Mat depthMask =
                cv_ptr->image.clone();

            if(depthMask.cols != imageWidth ||
               depthMask.rows != imageHeight)
            {
                cv::resize(
                    depthMask,
                    depthMask,
                    cv::Size(
                        imageWidth,
                        imageHeight),
                    0,
                    0,
                    cv::INTER_NEAREST);
            }

            learnFloorColor();

            cv::Mat kernel =
                cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size(7,7));

            cv::Mat dilated;

            cv::dilate(
                depthMask,
                dilated,
                kernel,
                cv::Point(-1,-1),
                dilateCount);

            cv::Mat colorFiltered =
                separateObjectFromFloor(
                    dilated);

            cv::Mat finalMask =
                confirmPersistence(
                    colorFiltered);

            cv_bridge::CvImage out;

            out.header =
                msg->header;

            out.encoding =
                sensor_msgs::image_encodings::MONO8;

            out.image =
                finalMask;

            output_pub_.publish(
                *out.toImageMsg());

            ROS_INFO(
                "Filtered mask published");
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("%s",e.what());
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
        "multi_filter_contour_processor_node");

    MultiFilterContourProcessor node;

    ros::spin();

    return 0;
}