//
// Created by eric1221bday on 2/25/17.
//

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
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
            sensor_msgs::Image, sensor_msgs::Range> tracker_policy;
    message_filters::Synchronizer<tracker_policy> synchronizer_;
    image_transport::ImageTransport it_;

public:
    GridTrackerWrapper(ros::NodeHandle nh) :
        nh_(nh),
        it_(nh),
        imu_sub_(nh, "imu", 1),
        range_sub_(nh, "range", 1)
    {
        initializeCallbacks();
    }

    void initializeCallbacks() {
        synchronizer_ = synchronizer_(tracker_policy(10), imu_sub_,
                      image_sub_, range_sub_);
        imu_sub_ = nh_.subscribe("imu", 100, &GridTrackerWrapper::imu_callback, this);
        image_sub_ = it_.subscribe("image", 100, &GridTrackerWrapper::image_callback, this);
        range_sub_ = nh_.subscribe("range", 100, &GridTrackerWrapper::range_callback, this);
    }

    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg) {
        MsgVariantsPtr p = std::make_shared < MsgVariants > (msg);
        MVGenericObs::Ptr q = MVGenericObs::Ptr(
                new MVGenericObs(p, "IMU", GridTracker::IMU,
                                 msg->header.stamp.toSec()));
        synchronizer_->addMessage(q);
    }

    void image_callback(const sensor_msgs::Image::ConstPtr &msg) {
        MsgVariantsPtr p = std::make_shared < MsgVariants > (msg);
        MVGenericObs::Ptr q = MVGenericObs::Ptr(
                new MVGenericObs(p, "IMAGE", GridTracker::IMAGE,
                                 msg->header.stamp.toSec()));
        synchronizer_->addMessage(q);
    }

    void range_callback(const sensor_msgs::Range::ConstPtr &msg) {
        MsgVariantsPtr p = std::make_shared < MsgVariants > (msg);
        MVGenericObs::Ptr q = MVGenericObs::Ptr(
                new MVGenericObs(p, "RANGE", GridTracker::RANGE,
                                 msg->header.stamp.toSec()));
        synchronizer_->addMessage(q);
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