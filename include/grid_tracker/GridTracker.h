//
// Created by eric1221bday on 3/4/17.
//

#ifndef GRID_TRACKER_GRIDTRACKER_H
#define GRID_TRACKER_GRIDTRACKER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdexcept>
#include <observation_synchronizer/ObservationSynchronizer.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/variant.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>

#include <memory>

class GridTracker {
public:
    typedef enum {
        IMU,
        IMAGE,
        RANGE
    } MsgTags;

    typedef boost::variant<sensor_msgs::Imu::ConstPtr,
    sensor_msgs::Image::ConstPtr, sensor_msgs::Range::ConstPtr> MsgVariants;
    typedef std::shared_ptr<MsgVariants> MsgVariantsPtr;
    typedef std::shared_ptr<const MsgVariants> MsgVariantsConstPtr;
    typedef ObservationSynchronizer<MsgVariants, MsgTags>::GenericObservation MVGenericObs;
};

#endif //GRID_TRACKER_GRIDTRACKER_H
