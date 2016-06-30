#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include "asctecInterface/common.h"
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/common.h>
#include <Eigen/Core>
#include <Eigen/Dense>

class position_control
{
public:
    position_control();
    void SetOdometry(const rotors_control::EigenOdometry& odometry);
    void SetPositionSP(const geometry_msgs::Point pos_sp);
    void SetVelocitySP(const geometry_msgs::Point vel_sp);
    void ActivatePositionControl();
    void ActivateVelocityControl();
    void RunController();
    Eigen::Vector3d GetVelocityError();

    Eigen::Vector3d _thrust;
    Eigen::Vector3d _RPY_SP;
    const int _spinRate = 50;

private:

    bool _position_active = false;
    bool _velocity_active = true;

    void setup();
    void CalculateVelocitySP();
    void CalculateThrustVector();
    void UpdateIntegrals();
    void CalculateAttitudeSPFromThrust();
    rotors_control::EigenOdometry _odometry;


    Eigen::Vector3d _pos_gains;
    Eigen::Vector3d _vel_gains_xy;
    Eigen::Vector3d _vel_gains_z;

    Eigen::Vector3d _prev_vel;

    Eigen::Vector3d _position_sp;
    Eigen::Vector3d _position_err;

    Eigen::Vector3d _vel_sp;
    Eigen::Vector3d _vel_err;
    Eigen::Vector3d _vel_err_t_1; //error at instant t-1
    Eigen::Vector3d _vel_err_t_2; //error at instant t-2


    Eigen::Vector3d _vel_err_i;


};

#endif // POSITION_CONTROL_H
