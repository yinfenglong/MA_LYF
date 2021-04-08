/*
 * @Author: Wei Luo
 * @Date: 2021-03-21 21:07:20
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-04-08 23:35:19
 * @Note: Basic PID controller
 */

#include <drone_mpc_pkg/controller/pid/PID_controller.hpp>

PIDController::PIDController(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh) : nh_(nh),
                                                                                             private_nh_(private_nh),
                                                                                             takeoff_height_(0.8),
                                                                                             is_trajectory_provided_(false)
{
    /* subscribe topics */
    // robot_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/robot_pose", 10, &PIDController::robot_pose_callback, this);
    robot_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>("/robot_pose", 10, &PIDController::current_odom_callback, this);
    trajectory_sub_ = nh_.subscribe<drone_mpc_pkg::itm_trajectory_msg>("/robot_trajectory", 10, &PIDController::trajectory_callback, this);
    /* server */
    controller_server_ = nh_.advertiseService("/itm_quadrotor_control/get_controller_state", &PIDController::controller_server_response_callback, this);
    /* client */
    // robot_loc_pose_pub_ = nh_node.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    /* init parameters */
    command_id = 0;
    is_current_pose_sub_ = false;
    is_robot_client_connected_ = false;

    ROS_INFO("Initialize PID controller");
}

PIDController::~PIDController()
{
}

void PIDController::idle()
{
    // stay the quadrotor on the ground
}

void PIDController::landing(ros::Time time_stamp, geometry_msgs::PoseStamped *msg)
{
    assert(msg != NULL);
    msg->header.stamp = time_stamp;
    msg->pose.position.x = initial_pose_.pose.position.x;
    msg->pose.position.y = initial_pose_.pose.position.y;
    msg->pose.position.z = initial_pose_.pose.position.z + 0.05;
    msg->pose.orientation.x = 0.0;
    msg->pose.orientation.y = 0.0;
    msg->pose.orientation.z = 0.0;
    msg->pose.orientation.w = 1.0;

    if (current_pose_.pose.position.z - initial_pose_.pose.position.z < 0.08)
    {
        // disarm the quadrotor
    }
}

void PIDController::takeoff(ros::Time time_stamp, geometry_msgs::PoseStamped *msg)
{
    // assert(msg != NULL);
    msg->header.stamp = time_stamp;
    msg->pose.position.x = initial_pose_.pose.position.x;
    msg->pose.position.y = initial_pose_.pose.position.y;
    msg->pose.position.z = initial_pose_.pose.position.z + takeoff_height_;
    msg->pose.orientation.x = 0.0;
    msg->pose.orientation.y = 0.0;
    msg->pose.orientation.z = 0.0;
    msg->pose.orientation.w = 1.0;
}

bool PIDController::is_initialized()
{
    if (is_robot_client_connected_ && is_current_pose_sub_)
        return true;
    else
        return false;
}

/* callback functions */
void PIDController::robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose_ = *msg;
    if (!is_current_pose_sub_)
    {
        is_current_pose_sub_ = true;
        initial_pose_ = *msg;
    }
}

void PIDController::trajectory_callback(const drone_mpc_pkg::itm_trajectory_msg::ConstPtr &msg)
{
    if (!is_trajectory_provided_)
        is_trajectory_provided_ = true;
    {
        boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexTrajectoryCallback_);
        trajectory_ref_point_ = std::make_shared<drone_mpc_pkg::itm_trajectory_msg>(*msg);
    }
}

void PIDController::current_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_pose_.pose.position.x = msg->pose.pose.position.x;
    current_pose_.pose.position.y = msg->pose.pose.position.y;
    current_pose_.pose.position.z = msg->pose.pose.position.z;
    current_pose_.pose.orientation.x = msg->pose.pose.orientation.x;
    current_pose_.pose.orientation.y = msg->pose.pose.orientation.y;
    current_pose_.pose.orientation.z = msg->pose.pose.orientation.z;
    current_pose_.pose.orientation.w = msg->pose.pose.orientation.w;
    if (!is_current_pose_sub_)
    {
        is_current_pose_sub_ = true;
        initial_odom_ = *msg;
    }
}

bool PIDController::controller_server_response_callback(drone_mpc_pkg::GetControllerState::Request &req,
                                                        drone_mpc_pkg::GetControllerState::Response &res)
{
    command_id = req.command_id;
    // robot_name_ = req.robot_name;
    if (is_current_pose_sub_)
    {
        res.connected = true;
        if (!is_robot_client_connected_)
            is_robot_client_connected_ = true;
        return true;
    }
    else
    {
        res.connected = false;
        return false;
    }
}

void PIDController::mission_loop(ros::Time time_stamp, geometry_msgs::PoseStamped *msg)
{
    /* check if the trajectory command is avaliable */
    if (is_trajectory_provided_)
    {
        boost::shared_lock<boost::shared_mutex> mutexTrajectoryCallback(mutexTrajectoryCallback_);
        if (trajectory_ref_point_->size > 1)
        {
            ROS_INFO("PID only takes the first trajectory waypoint!");
        }
        msg->header.stamp = time_stamp;
        msg->pose.position.x = trajectory_ref_point_->traj[0].x;
        msg->pose.position.y = trajectory_ref_point_->traj[0].y;
        msg->pose.position.z = trajectory_ref_point_->traj[0].z;
        if (trajectory_ref_point_->traj[0].quaternion_given)
        {
            msg->pose.orientation.w = trajectory_ref_point_->traj[0].q[0];
            msg->pose.orientation.x = trajectory_ref_point_->traj[0].q[1];
            msg->pose.orientation.y = trajectory_ref_point_->traj[0].q[2];
            msg->pose.orientation.z = trajectory_ref_point_->traj[0].q[3];
        }
        else
        {
            /* convert rpy to quaternion */
            Eigen::Vector3d rpy = {trajectory_ref_point_->traj[0].roll, trajectory_ref_point_->traj[0].pitch, trajectory_ref_point_->traj[0].yaw};
            Eigen::Quaterniond quat;
            rpy_to_quaternion(rpy, quat);
            msg->pose.orientation.w = quat.w();
            msg->pose.orientation.x = quat.x();
            msg->pose.orientation.y = quat.y();
            msg->pose.orientation.z = quat.z();
        }
    }
    else
    {
        /* stay with the take off position */
        msg->header.stamp = time_stamp;
        msg->pose.position.x = initial_pose_.pose.position.x;
        msg->pose.position.y = initial_pose_.pose.position.y;
        msg->pose.position.z = initial_pose_.pose.position.z + takeoff_height_;
        msg->pose.orientation.x = 0.0;
        msg->pose.orientation.y = 0.0;
        msg->pose.orientation.z = 0.0;
        msg->pose.orientation.w = 1.0;
    }
}

void PIDController::rpy_to_quaternion(Eigen::Vector3d rpy, Eigen::Quaterniond &q)
{
    double cy = std::cos(rpy[2] * 0.5);
    double sy = std::sin(rpy[2] * 0.5);
    double cp = std::cos(rpy[1] * 0.5);
    double sp = std::sin(rpy[1] * 0.5);
    double cr = std::cos(rpy[0] * 0.5);
    double sr = std::sin(rpy[0] * 0.5);

    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "QuadrotorPIDNode");

    ros::NodeHandle nh_node, private_nh_node("~");
    PIDController controller_node(nh_node, private_nh_node);
    ros::Rate rate(50.0);
    /* Publisher: publish command_roll_pitch_yaw_thrust_*/
    ros::Publisher command_attitude_thrust_pub_ = nh_node.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 100);
    ros::Publisher command_position_pub_ = nh_node.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    mavros_msgs::AttitudeTargetPtr attitude_msg(new mavros_msgs::AttitudeTarget);
    geometry_msgs::PoseStampedPtr position_msg(new geometry_msgs::PoseStamped);

    while (ros::ok())
    {
        if (controller_node.is_initialized())
        {
            switch (controller_node.command_id)
            {
            case 0: // idle
                controller_node.idle();
                break;
            case 1: // take_off
            {
                controller_node.takeoff(ros::Time::now(), position_msg.get());
                command_position_pub_.publish(position_msg);
                ROS_INFO_ONCE("Take off");
                break;
            }
            case 2: // landing
            {
                // controller_node.landing(ros::Time::now(), position_msg.get());
                // command_position_pub_.publish(position_msg);
                break;
            }
            case 3: // go to position
            {
                controller_node.mission_loop(ros::Time::now(), position_msg.get());
                command_position_pub_.publish(position_msg);
                ROS_INFO_ONCE("Go to Position Mode.");
                break;
            }
            default:
            {
                ROS_INFO("Please check the command id.");
            }
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("PID controller is closed");
}