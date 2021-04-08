#ifndef PX4_CONTROLLER_HPP
#define PX4_CONTROLLER_HPP
/*
 * @Author: Wei Luo
 * @Date: 2021-03-14 23:40:44
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-04-06 15:53:41
 * @Note: Note
 */
# include <ros/ros.h>
// server msg
#include <itm_nonlinear_mpc/SetMode.h>
#include <itm_nonlinear_mpc/GetControllerState.h>
// topic msg
#include <itm_nonlinear_mpc/SetMission.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

// controller
#include <itm_nonlinear_mpc/controller/pid/PID_controller.hpp>
// #include <itm_nonlinear_mpc/experiment/command_to_px4.hpp>

mavros_msgs::State mavros_state;
itm_nonlinear_mpc::SetMode uav_mode;
itm_nonlinear_mpc::GetControllerState controller_state;
// bool controller_state;

bool current_pos_flag = false;
int controller_type;
geometry_msgs::PoseStamped current_pos;
geometry_msgs::PoseStamped initial_pos;
itm_nonlinear_mpc::SetMission user_command;
itm_nonlinear_mpc::SetMission last_user_command;

mavros_msgs::SetMode offboard_set_mode;
mavros_msgs::CommandBool arm_cmd;
ros::Time last_request;

bool is_controller_working = false;


geometry_msgs::PoseStamped command_pose;

ros::Publisher robot_command;

// subscribe mavros
void mavros_state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    if (mavros_state.armed != msg->armed)
    {
        std::cout <<"UAV: armed state changes from "<< static_cast<int16_t>(mavros_state.armed)<<"to"<<static_cast<int16_t>(msg->armed)<<std::endl;
    }

    if (mavros_state.connected != msg->connected)
    {
        std::cout <<"UAV: connection changes from "<< static_cast<int16_t>(mavros_state.connected)<<"to"<<static_cast<int16_t>(msg->connected)<<std::endl;
    }

    if (mavros_state.mode != msg->mode)
    {
        std::cout <<"UAV: mode changes from "<< mavros_state.mode<<"to"<<msg->mode<<std::endl;
    }

    if (mavros_state.system_status != msg->system_status)
    {

    }

    mavros_state = *msg;
}// mavros_state_cb

void current_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(!current_pos_flag){
        current_pos_flag = true;
        initial_pos = *msg;
        ROS_INFO("Inertial Position is %f, %f, %f", msg->pose.position.x,
        msg->pose.position.y, msg->pose.position.z);
    }
    current_pos = *msg;

    ROS_INFO_ONCE("mpc_current_pos_flag is true");
}//current_pos_callback

void current_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(!current_pos_flag){
        initial_pos.pose.position.x = msg->pose.pose.position.x;
        initial_pos.pose.position.y = msg->pose.pose.position.y;
        initial_pos.pose.position.z = msg->pose.pose.position.z;
        initial_pos.pose.orientation.x = msg->pose.pose.orientation.x;
        initial_pos.pose.orientation.y = msg->pose.pose.orientation.y;
        initial_pos.pose.orientation.z = msg->pose.pose.orientation.z;
        initial_pos.pose.orientation.w = msg->pose.pose.orientation.w;
        ROS_INFO("Inertial Position is %f, %f, %f", initial_pos.pose.position.x,
        initial_pos.pose.position.y, initial_pos.pose.position.z);
        current_pos_flag = true;
    }

    current_pos.pose.position.x = msg->pose.pose.position.x;
    current_pos.pose.position.y = msg->pose.pose.position.y;
    current_pos.pose.position.z = msg->pose.pose.position.z;
    current_pos.pose.orientation.x = msg->pose.pose.orientation.x;
    current_pos.pose.orientation.y = msg->pose.pose.orientation.y;
    current_pos.pose.orientation.z = msg->pose.pose.orientation.z;
    current_pos.pose.orientation.w = msg->pose.pose.orientation.w;
}

void user_command_callback(const itm_nonlinear_mpc::SetMission::ConstPtr& msg)
{
    user_command.mission_mode = msg->mission_mode;
    user_command.command_idx += 1;
}

void landing()
{
    /* let the quadrotor fly back to the inertial position */
    command_pose.pose.position.x = initial_pos.pose.position.x;
    command_pose.pose.position.y = initial_pos.pose.position.y;
    command_pose.pose.position.z = initial_pos.pose.position.z-0.1;
    command_pose.pose.orientation.w = 1.0;
    command_pose.pose.orientation.x = 0.0;
    command_pose.pose.orientation.y = 0.0;
    command_pose.pose.orientation.z = 0.0;
    robot_command.publish(command_pose);
    ROS_INFO_ONCE("landing");
}

#endif /* PX4_CONTROLLER_HPP */