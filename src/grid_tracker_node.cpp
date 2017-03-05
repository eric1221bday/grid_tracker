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

class GridTrackerWrapper {
private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    message_filters::Subscriber<sensor_msgs::Range>range_sub_;
    image_transport::SubscriberFilter image_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu,
            sensor_msgs::ImageConstPtr&, sensor_msgs::Range> tracker_policy;
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
        initializeCallbacks();
    }

    void initializeCallbacks() {
        synchronizer_.registerCallback(boost::bind(&GridTrackerWrapper::topics_callback, this, _1, _2, _3));
    }

    void topics_callback(const sensor_msgs::Imu imuMsg, const sensor_msgs::ImageConstPtr& imageMsg, const sensor_msgs::Range rangeMsg) {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(imageMsg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            throw std::runtime_error(
                    std::string("cv_bridge exception: ") + std::string(e.what()));
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "grid_tracker_node");
    ros::NodeHandle nh("~");
    GridTrackerWrapper wrapper(nh);
    ros::Rate rate(60);
    do {
        ros::spinOnce();
        rate.sleep();
    } while(ros::ok());

    return 0;
}