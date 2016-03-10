#include "asctecInterface/position_control.h"

position_control::position_control()
{
    ROS_INFO("[pos_ctl] Position controller created!");

    _position_sp << 0.0,0.0,2;
    _vel_gains_z << 10.0, 3.5 , 0.04; //P, D , I
    _vel_gains_xy<< 4, 0.3 , 0.01;  //P, D , I
    _pos_gains   << 0.2 , 0.2 , 0.2;  //X, Y , Z
    _thrust      << 0.0 , 0.0 , 0.0;
    _vel_err_i   << 0.0 , 0.0 , 0.0;

}

void position_control::SetOdometry(const rotors_control::EigenOdometry& odometry){

    _odometry = odometry;

}

void position_control::SetPositionSP(const geometry_msgs::Point pos_sp){

    _position_sp = mav_msgs::vector3FromPointMsg(pos_sp);

}

void position_control::CalculateVelocitySP(){

   _position_err = _position_sp - _odometry.position;
   _vel_sp[0] = _pos_gains[0] * _position_err[0];
   _vel_sp[1] = _pos_gains[1] * _position_err[1];
   _vel_sp[2] = _pos_gains[2] * _position_err[2];

}

void position_control::UpdateIntegrals(){

   _vel_err_i[0] +=  _vel_err[0] * _vel_gains_xy[2];
   _vel_err_i[1] +=  _vel_err[1] * _vel_gains_xy[2];
   _vel_err_i[2] +=  _vel_err[2] * _vel_gains_z[2];

}

void position_control::CalculateThrustVector(){

    _vel_err = _vel_sp - _odometry.velocity;

    Eigen::Vector3d vel_err_d;
    Eigen::Vector3d thrust_vect ;

    vel_err_d[0] = _pos_gains[0]*(/*Put here SP velocity*/ -_odometry.velocity[0]) - (_odometry.velocity[0] - _prev_vel[0]);
    vel_err_d[1] = _pos_gains[1]*(/*Put here SP velocity*/ -_odometry.velocity[1]) - (_odometry.velocity[1] - _prev_vel[1]);
    vel_err_d[2] = _pos_gains[2]*(/*Put here SP velocity*/ -_odometry.velocity[2]) - (_odometry.velocity[2] - _prev_vel[2]);

    UpdateIntegrals();

    thrust_vect[0] = _vel_gains_xy[0] * _vel_err[0] + _vel_gains_xy[1] * vel_err_d[0] + _vel_err_i[0];
    thrust_vect[1] = _vel_gains_xy[0] * _vel_err[1] + _vel_gains_xy[1] * vel_err_d[1] + _vel_err_i[1];
    thrust_vect[2] = _vel_gains_z[0]  * _vel_err[2] + _vel_gains_z[1]  * vel_err_d[2] + _vel_err_i[2] + 15;

    _prev_vel = _odometry.velocity;

    _thrust = thrust_vect;

}

void position_control::CalculateAttitudeSPFromThrust() {

    Eigen::Vector3d body_x(0, 0, 0);
    Eigen::Vector3d body_y(0, 0, 0);
    Eigen::Vector3d body_z(0, 0, 0);

    double actual_yaw = mav_msgs::yawFromQuaternion(_odometry.orientation);

    Eigen::Vector3d y_c(-sin(actual_yaw) , cos(actual_yaw) , 0.0 );

    if (_thrust.norm() > 0.01){
        body_z = _thrust / _thrust.norm();
    }else{
        body_z << 0,0,1;
    }

    //Be more careful

    body_x = y_c.cross(body_z);
    body_x.normalize();
    body_y = body_z.cross(body_x);
    body_y.normalize();

    //Create desired rotation matrix

    Eigen::Matrix3d rot_des;
    rot_des.setIdentity();

    rot_des.col(0) = body_x;
    rot_des.col(1) = body_y;
    rot_des.col(2) = body_z;

    Eigen::Quaterniond q(rot_des);

    //std::atan2(2*(_qw*_qx+_qy*_qz), 1-2*(_qx*_qx+_qy*_qy));
    //std::asin(2*(_qw*_qy - _qz*_qx));
    //std::atan2(2*(_qw*_qz+_qx*_qy), 1-2*(_qy*_qy + _qz*_qz));

    _RPY_SP[0] = std::atan2(2*(q.w()*q.x() + q.y()*q.z()), 1 - 2*(q.x()*q.x() + q.y() + q.y()));
    _RPY_SP[1] = std::asin(2*(q.w()*q.y() - q.z()*q.x()));
    _RPY_SP[2] = std::atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z() + q.z()));

    //Eigen::Vector3d desired_att_angles = rot_des.eulerAngles(2,0,1);

    //_RPY_SP = desired_att_angles;

    std::cout << "************" << std::endl;
    std::cout << rot_des << std::endl;
    std::cout << "************" << std::endl;

}
void position_control::RunController(){

    ROS_INFO_ONCE("[pos_ctl] First loop");

    CalculateVelocitySP();

    CalculateThrustVector();

    CalculateAttitudeSPFromThrust();

}


