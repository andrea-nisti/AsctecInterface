
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "asctecInterface/position_control.h"




void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
void positionCommandCallback(const geometry_msgs::Point pose_msg);

//create a global position controller
position_control position_controller;
// define thrust message
mav_msgs::RollPitchYawrateThrust thrust_msg;
ros::Publisher chatter_pub;
int main(int argc, char **argv)
{

  ros::init(argc, argv, "position_controller_node");

  ros::NodeHandle n;

  ros::Subscriber odometry_sub = n.subscribe("/firefly/ground_truth/odometry",1000,&odometryCallback);
  ros::Subscriber pos_sp_sub = n.subscribe("/firefly/position_control/pos_sp",1000,&positionCommandCallback);

  chatter_pub = n.advertise<mav_msgs::RollPitchYawrateThrust>("/firefly/command/roll_pitch_yawrate_thrust", 1000);
  ros::spin();
}


void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("[pos_ctl_nd] First odometry message received");

    rotors_control::EigenOdometry odometry;
    rotors_control::eigenOdometryFromMsg(odometry_msg, &odometry);

    position_controller.SetOdometry(odometry);

    position_controller.RunController();

    thrust_msg.header.stamp = odometry_msg->header.stamp;

    thrust_msg.thrust.z = position_controller._thrust[2];
    thrust_msg.roll = position_controller._RPY_SP[0];
    thrust_msg.pitch = position_controller._RPY_SP[1];
    std::cout << position_controller._thrust << std::endl;
    std::cout << "____" << std::endl;
    std::cout << position_controller._RPY_SP[0]<<" " << position_controller._RPY_SP[1] << std::endl;
    std::cout << "_____________________" << std::endl;
    chatter_pub.publish(thrust_msg);


}

void positionCommandCallback(const geometry_msgs::Point pose_msg){
    ROS_INFO("[pos_ctl_nd] Position command message received");

    position_controller.SetPositionSP(pose_msg);

}


