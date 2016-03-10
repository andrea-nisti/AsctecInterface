#ifndef INCLUDE_ROTORS_CONTROL_COMMON_H_
#define INCLUDE_ROTORS_CONTROL_COMMON_H_

#include <assert.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/common.h>


namespace rotors_control {

struct EigenOdometry {
    EigenOdometry()
        : position(0.0, 0.0, 0.0),
          orientation(Eigen::Quaterniond::Identity()),
          velocity(0.0, 0.0, 0.0),
          angular_velocity(0.0, 0.0, 0.0) {};

    EigenOdometry(const Eigen::Vector3d& _position,
                  const Eigen::Quaterniond& _orientation,
                  const Eigen::Vector3d& _velocity,
                  const Eigen::Vector3d& _angular_velocity) {
        position = _position;
        orientation = _orientation;
        velocity = _velocity;
        angular_velocity = _angular_velocity;
    };

    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d velocity; // Velocity is expressed in the Body frame!
    Eigen::Vector3d angular_velocity;
};

inline void eigenOdometryFromMsg(const nav_msgs::OdometryConstPtr& msg,
                                 EigenOdometry* odometry) {
    odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.pose.position);
    odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);
    odometry->velocity = mav_msgs::vector3FromMsg(msg->twist.twist.linear);
    odometry->angular_velocity = mav_msgs::vector3FromMsg(msg->twist.twist.angular);
}

inline void skewMatrixFromVector(Eigen::Vector3d& vector, Eigen::Matrix3d* skew_matrix) {
    *skew_matrix << 0, -vector.z(), vector.y(),
            vector.z(), 0, -vector.x(),
            -vector.y(), vector.x(), 0;
}

inline void vectorFromSkewMatrix(Eigen::Matrix3d& skew_matrix, Eigen::Vector3d* vector) {
    *vector << skew_matrix(2, 1), skew_matrix(0,2), skew_matrix(1, 0);
}
}

#endif /* INCLUDE_ROTORS_CONTROL_COMMON_H_ */
