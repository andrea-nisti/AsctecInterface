#include "ros/ros.h"
#include "std_msgs/String.h"
#include "asctecInterface/position_control.h"




void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
void velocityCommandCallback(const geometry_msgs::Pose pose_msg);
void positionCommandCallback(const geometry_msgs::Point pose_msg);

//create a global position controller
position_control position_controller;

// define thrust message
mav_msgs::RollPitchYawrateThrust thrust_msg;
ros::Publisher chatter_pub;
ros::Publisher vel_err_pub;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "position_controller_node");

    ros::NodeHandle n;

    ros::Subscriber odometry_sub = n.subscribe("/firefly/ground_truth/odometry",1,&odometryCallback);
    ros::Subscriber vel_sp_sub = n.subscribe("/firefly/position_control/vel_sp",1,&velocityCommandCallback);
    ros::Subscriber pos_sp_sub = n.subscribe("/firefly/position_control/pos_sp",1,&positionCommandCallback);

    chatter_pub = n.advertise<mav_msgs::RollPitchYawrateThrust>("/firefly/command/roll_pitch_yawrate_thrust", 1);
    vel_err_pub = n.advertise<geometry_msgs::Point>("/velocity_error",1);

    ros::Rate rate(position_controller._spinRate);

    while (ros::ok()) {

        position_controller.RunController();

        thrust_msg.thrust.z = position_controller._thrust[2];

        double actual_yaw = mav_msgs::yawFromQuaternion(position_controller._odometry.orientation);
        if (fabs(actual_yaw - 1.57) > 0.1) {
            thrust_msg.roll = position_controller._RPY_SP[0];
            thrust_msg.pitch = position_controller._RPY_SP[1];
        } else {
            thrust_msg.roll = 0;
            thrust_msg.pitch = 0;
        }

        thrust_msg.yaw_rate = position_controller._yaw_command;

        std::cout << "Thrust: " << position_controller._thrust[2] << std::endl;
        std::cout << "Roll = " << position_controller._RPY_SP[0] << std::endl;
        std::cout << "Pitch = " << position_controller._RPY_SP[1] << std::endl ;
        std::cout << "Yaw = " << thrust_msg.yaw_rate << std::endl;
        std::cout << "Actual yaw = " << actual_yaw << std::endl << std::endl;

//        std::cout << "____" << std::endl;
//        std::cout << "Roll: " << position_controller._RPY_SP[0]<<" Pitch: " << position_controller._RPY_SP[1] << std::endl;
//        std::cout << "_____________________" << std::endl;

        chatter_pub.publish(thrust_msg);

        Eigen::Vector3d vel_err = position_controller.GetVelocityError();
        geometry_msgs::Point vel_err_msg;
        vel_err_msg.x = vel_err[0];
        vel_err_msg.y = vel_err[1];
        vel_err_msg.z = vel_err[2];

        vel_err_pub.publish(vel_err_msg);

        ros::spinOnce();

        rate.sleep();
    }

    return 1;
}


void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("[pos_ctl_nd] First odometry message received");

    rotors_control::EigenOdometry odometry;
    rotors_control::eigenOdometryFromMsg(odometry_msg, &odometry);

    position_controller.SetOdometry(odometry);

    thrust_msg.header.stamp = odometry_msg->header.stamp;

}

void positionCommandCallback(const geometry_msgs::Point pose_msg) {
    //ROS_INFO("[pos_ctl_nd] Position command message received");

    position_controller.ActivatePositionControl();
    position_controller.SetPositionSP(pose_msg);

}

void velocityCommandCallback(const geometry_msgs::Pose vel_msg){

    //ROS_INFO("[pos_ctl_nd] Velocity command message received");

    position_controller.ActivateVelocityControl();
    position_controller.SetVelocitySP(vel_msg);

}
