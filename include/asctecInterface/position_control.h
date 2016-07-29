#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include "asctecInterface/common.h"
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/common.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tf/tf.h>

class position_control
{
public:
    position_control();
    void SetOdometry(const rotors_control::EigenOdometry& odometry);
    void SetPositionSP(const geometry_msgs::Point pos_sp);
    void SetVelocitySP(const geometry_msgs::Pose vel_sp);
    void ActivatePositionControl();
    void ActivateVelocityControl();
    void RunController();
    Eigen::Vector3d GetVelocityError();
    rotors_control::EigenOdometry _odometry;


    Eigen::Vector3d _thrust;
    Eigen::Vector3d _RPY_SP;
    double _yaw_command;
    const int _spinRate = 50;

private:

    bool _position_active = false;
    bool _velocity_active = true;

    void setup();
    void CalculateVelocitySP();
    void CalculateThrustVector();
    void UpdateIntegrals();
    void CalculateAttitudeSPFromThrust();
    void calculateYawCommand();

    double step = 1 / _spinRate;

    Eigen::Vector3d _pos_gains;
    Eigen::Vector3d _vel_gains_xy;
    Eigen::Vector3d _vel_gains_z;
    Eigen::Vector3d _yaw_gains;

    Eigen::Vector3d _position_sp;
    Eigen::Vector3d _position_err;

    Eigen::Vector3d _vel_sp;
    double _yaw_sp;
    Eigen::Vector3d _vel_err;
    Eigen::Vector3d _vel_err_t_1; //error at instant t-1
    Eigen::Vector3d _vel_err_t_2; //error at instant t-2
    double _yaw_err;
    double _yaw_err_t_1; //error at instant t-1
    double _yaw_err_t_2; //error at instant t-2
    double _yaw_err_i;
    Eigen::Vector3d _vel_err_i;


};

#endif // POSITION_CONTROL_H
