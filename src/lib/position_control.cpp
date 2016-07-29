#include <asctecInterface/position_control.h>

position_control::position_control()
{
    ROS_INFO("[pos_ctl] Position controller created!");

    _position_sp << 0.0 , 0.0 , 2;
    _vel_gains_z << 10.0, 3.5 , 0.04;  //P, D , I
    _vel_gains_xy<< 4.0 , 2.0 , 0.04;  //P, D , I
    _yaw_gains   << 0.3 , 0.5 , 0.0;   //P, D, I
    _pos_gains   << 0.2 , 0.2 , 0.2;   //X, Y , Z
    _thrust      << 0.0 , 0.0 , 0.0;
    _vel_err_i   << 0.0 , 0.0 , 0.0;

}

void position_control::ActivatePositionControl() {

    _position_active = true;
    _velocity_active = false;

}

void position_control::ActivateVelocityControl(){

    _position_active = false;
    _velocity_active = true;

}



void position_control::SetOdometry(const rotors_control::EigenOdometry& odometry){

    _odometry = odometry;

}

void position_control::SetPositionSP(const geometry_msgs::Point pos_sp){

    _position_sp = mav_msgs::vector3FromPointMsg(pos_sp);

}

void position_control::SetVelocitySP(const geometry_msgs::Pose vel_sp){

    _vel_sp = mav_msgs::vector3FromPointMsg(vel_sp.position);
    _yaw_sp = tf::getYaw(vel_sp.orientation);
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

    _vel_err_t_2 = _vel_err_t_1;
    _vel_err_t_1 = _vel_err;
    _vel_err = _vel_sp - _odometry.velocity;

    Eigen::Vector3d vel_err_d;
    Eigen::Vector3d thrust_vect ;


    // three-point derivative (Lagrange Approach) f'(x) = ( f(x-2h) - 4f(x-h) + 3f(x) ) / 2h
    vel_err_d[0] = (_vel_err_t_2[0] - 4 * _vel_err_t_1[0] + 3 * _vel_err[0]) / 2 * step;
    vel_err_d[1] = (_vel_err_t_2[1] - 4 * _vel_err_t_1[1] + 3 * _vel_err[1]) / 2 * step;
    vel_err_d[2] = (_vel_err_t_2[2] - 4 * _vel_err_t_1[2] + 3 * _vel_err[2]) / 2 * step;


//    vel_err_d[0] = _pos_gains[0]*(_vel_sp[0] -_odometry.velocity[0]) - (_odometry.velocity[0] - _prev_vel[0]);
//    vel_err_d[1] = _pos_gains[1]*(_vel_sp[1] -_odometry.velocity[1]) - (_odometry.velocity[1] - _prev_vel[1]);
//    vel_err_d[2] = _pos_gains[2]*(_vel_sp[2] -_odometry.velocity[2]) - (_odometry.velocity[2] - _prev_vel[2]);



    UpdateIntegrals();

    thrust_vect[0] = _vel_gains_xy[0] * _vel_err[0] + _vel_gains_xy[1] * vel_err_d[0] + _vel_err_i[0];
    thrust_vect[1] = _vel_gains_xy[0] * _vel_err[1] + _vel_gains_xy[1] * vel_err_d[1] + _vel_err_i[1];
    thrust_vect[2] = _vel_gains_z[0]  * _vel_err[2] + _vel_gains_z[1]  * vel_err_d[2] + _vel_err_i[2] + 15;

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

    _RPY_SP[0] = std::atan2(2*(q.w()*q.x() + q.y()*q.z()), 1 - 2*(q.x()*q.x() + q.y()*q.y()));
    _RPY_SP[1] = std::asin(2*(q.w()*q.y() - q.z()*q.x()));
    _RPY_SP[2] = std::atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z()));



    std::cout << "************" << std::endl;
    std::cout << std::setprecision(2) << rot_des << std::endl;
    std::cout << std::setprecision(2) << "quaternion = [ " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << " ]" << std::endl;
    std::cout << "************" << std::endl;

}

void position_control::calculateYawCommand(){

    double actual_yaw = mav_msgs::yawFromQuaternion(_odometry.orientation);
//    std::cout << "actual yaw = " << actual_yaw << std::endl;
    std::cout << "desired yaw = " << _yaw_sp << std::endl;

    _yaw_err_t_2 = _yaw_err_t_1;
    _yaw_err_t_1 = _yaw_err;
    _yaw_err = _yaw_sp - actual_yaw;

    double yaw_err_d = (_yaw_err_t_2 - 4 * _yaw_err_t_1 + 3 * _yaw_err) / 2 * step;
    _yaw_err_i += _yaw_err * _yaw_gains[2];
    _yaw_command = _yaw_gains[0] * _yaw_err + _yaw_gains[1] * yaw_err_d + _yaw_err_i;

//    std::cout << "yaw command = " << _yaw_command << std::endl << std::endl;

}

void position_control::RunController(){

    ROS_INFO_ONCE("[pos_ctl] First loop");

    if (_position_active)
        CalculateVelocitySP();

    CalculateThrustVector();
    calculateYawCommand();
    CalculateAttitudeSPFromThrust();

}

Eigen::Vector3d position_control::GetVelocityError (){
    return _vel_err;
}
