//
// Created by eric1221bday on 2/25/17.
//

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>
#include <message_filters/subscriber.h>

#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <grid_tracker/GridTracker.h>

#include <opencv2/highgui.hpp>

class GridTrackerWrapper {
private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    message_filters::Subscriber<sensor_msgs::Range>range_sub_;
    image_transport::SubscriberFilter image_sub_;
    image_transport::Publisher image_pub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu,
            sensor_msgs::Image, sensor_msgs::Range> tracker_policy;
    message_filters::Synchronizer<tracker_policy> synchronizer_;
    image_transport::ImageTransport it_;

public:
    GridTrackerWrapper(ros::NodeHandle nh) :
        nh_(nh),
        it_(nh),
        imu_sub_(nh, "imu", 1),
        range_sub_(nh, "range", 1),
        image_sub_(it_, "image", 1),
        synchronizer_(tracker_policy(10), imu_sub_, image_sub_, range_sub_)
    {
        std::cout << "woo!" << std::endl;
        initializeCallbacks();
        cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE );

    }

    ~GridTrackerWrapper() {
        cv::destroyWindow("Display window");
    }

    void initializeCallbacks() {
        synchronizer_.registerCallback(boost::bind(&GridTrackerWrapper::topics_callback, this, _1, _2, _3));
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
    }

    void topics_callback(const sensor_msgs::Imu::ConstPtr& imuMsg,
                         const sensor_msgs::ImageConstPtr& imageMsg,
                         const sensor_msgs::Range::ConstPtr& rangeMsg) {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            throw std::runtime_error(
                    std::string("cv_bridge exception: ") + std::string(e.what()));
        }

        // Draw an example circle on the video stream
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

        // Update GUI Window
        cv::imshow("Display window", cv_ptr->image);
        cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "grid_tracker_node");
    ros::NodeHandle nh("~");
    GridTrackerWrapper wrapper(nh);
    std::cout << "initialization done!" << std::endl;
    ros::Rate rate(60);
    do {
        ros::spinOnce();
        rate.sleep();
    } while(ros::ok());

    return 0;
}