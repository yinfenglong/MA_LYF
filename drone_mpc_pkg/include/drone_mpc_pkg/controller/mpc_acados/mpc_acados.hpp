/*
 * @Author: Wei Luo
 * @Date: 2021-04-04 00:13:31
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-04-07 17:45:15
 * @Note: Note
 */
#ifndef _MPC_ACADOS_HPP_
#define _MPC_ACADOS_HPP_

// ros
#include <geometry_msgs/PoseStamped.h>
#include <itm_nonlinear_mpc/GetControllerState.h>
#include <itm_nonlinear_mpc/itm_trajectory_msg.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <chrono>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
// Eigen
#include <Eigen/Dense>
// boost
#include <boost/thread.hpp>
// acados
#include <acados/utils/math.h>
#include <acados_c/ocp_nlp_interface.h>
#include <acados_sim_solver_quadrotor_q.h>
#include <acados_solver_quadrotor_q.h>

namespace acados_quadrotor
{
    class MPCAcadosController
    {
    public:
        MPCAcadosController(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);
        ~MPCAcadosController();
        bool is_initialized();
        void takeoff(ros::Time time_stamp, mavros_msgs::AttitudeTarget* msg);
        void mission_loop(ros::Time time_stamp, mavros_msgs::AttitudeTarget* msg);
        int command_id;

    private:
        /* ROS related */
        ros::NodeHandle nh_, private_nh_;
        ros::Subscriber robot_pose_sub_;
        ros::Subscriber trajectory_sub_;
        ros::ServiceServer controller_server_;
        /* state check */
        bool is_current_pose_sub_;
        bool is_robot_client_connected_;
        bool is_trajectory_provided_;
        /* states */
        Eigen::Vector3d robot_current_pose;
        geometry_msgs::PoseStamped current_pose_;
        geometry_msgs::PoseStamped initial_pose_;
        nav_msgs::Odometry initial_odom_;
        double takeoff_height_;
        /* mutex */
        boost::shared_mutex mutexTrajectoryCallback_;
        std::shared_ptr<itm_nonlinear_mpc::itm_trajectory_msg> trajectory_ref_point_;
        /* ACADOS */
        nlp_solver_capsule *acados_ocp_capsule;
        int acados_status;
        ocp_nlp_config *nlp_config;
        ocp_nlp_dims *nlp_dims;
        ocp_nlp_in *nlp_in;
        ocp_nlp_out *nlp_out;
        int time_horizon;
        int num_states;
        int num_controls;
        double robot_current_state[10]; // x, y, z, qw, qx, qy, qz, vx, vy, vz
        double robot_init_state[10];
        double robot_command[4]; // roll_r, pitch_r, yaw_r, thrust
        Eigen::MatrixXd trajectory_reference;

        /* callback functions */
        void robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void trajectory_callback(const itm_nonlinear_mpc::itm_trajectory_msg::ConstPtr &msg);
        bool controller_server_response_callback(itm_nonlinear_mpc::GetControllerState::Request &req, itm_nonlinear_mpc::GetControllerState::Response &res);
        void current_odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
        /* math functions */
        void rpy_to_quaternion(Eigen::Vector3d rpy, Eigen::Quaterniond &q);
        void load_trajectory(Eigen::MatrixXd &trajectory);
        /* ACADOS calculation */
        void solvingACADOS(Eigen::MatrixXd ref); //, double *estimated_control
    };
}

#endif /* _MPC_ACADOS_HPP_ */