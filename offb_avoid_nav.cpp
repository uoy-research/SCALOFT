#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <gazebo_msgs/SetModelState.h>
#include "droneNav.h"


# define PI 3.14159265358979323846

using namespace std;

mavros_msgs::State current_state;
nav_msgs::Odometry pose_odom;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::Publisher local_vel_pub;
ros::Publisher move_base_pub;
ros::Time  last_request;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state=*msg;
}

void setup(const nav_msgs::Odometry::ConstPtr& msg)
{
  printf("Acuiring connections\n");
  pose_odom=*msg;

  geometry_msgs::TwistStamped vel;
  for (int i=0; i<100; ++i)
  {
    local_vel_pub.publish(vel);
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
            return;
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
                return;
            }
        }
  }

void land(std_msgs::Float32 msg)
{
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "AUTO.LAND";
  if( current_state.mode != "AUTO.LAND" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
}

int main(int argc, char **argv)
{

  float max_vel=atof(argv[1]);
  ros::init(argc, argv, "nav_node");
  ros::NodeHandle nh;
  last_request=ros::Time::now();
  ros::Rate rate(20.0);

  // Services
  arming_client=nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
  set_mode_client=nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

  local_vel_pub=nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
  geometry_msgs::TwistStamped vel;

  for (int i=0; i<100; ++i)
  {
    local_vel_pub.publish(vel);
  }

  ros::Publisher local_pos_pub=nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Subscriber local_pose_sub=nh.subscribe("mavros/global_position/local", 10, setup);
  ros::Subscriber state_sub=nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber land_sub=nh.subscribe<std_msgs::Float32>("/land_command",10, land);

  while (current_state.mode!="OFFBOARD" || !current_state.armed)
  {
    ros::spinOnce();
    rate.sleep();
  }
  local_pose_sub.shutdown();

  printf("Setup complete\n");

  printf("Max_vel setting:  %f\n", max_vel);
  droneNav test(max_vel, nh);

  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}