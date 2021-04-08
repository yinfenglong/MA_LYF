/*
 * @Author: Wei Luo
 * @Date: 2021-04-04 00:12:43
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-04-08 23:34:44
 * @Note: Note
 */
#include <drone_mpc_pkg/controller/mpc_acados/mpc_acados.hpp>
namespace acados_quadrotor
{
    MPCAcadosController::MPCAcadosController(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh) :
    nh_(nh),
    private_nh_(private_nh),
    is_trajectory_provided_(false),
    is_current_pose_sub_(false),
    is_robot_client_connected_(false),
    takeoff_height_(0.8)
    {
        /* subscribe topics */
        robot_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>("/robot_pose", 10, &MPCAcadosController::current_odom_callback, this);
        trajectory_sub_ = nh_.subscribe<drone_mpc_pkg::itm_trajectory_msg>("/robot_trajectory", 10, &MPCAcadosController::trajectory_callback, this);
        /* server */
        controller_server_ = nh_.advertiseService("/itm_quadrotor_control/get_controller_state", &MPCAcadosController::controller_server_response_callback, this);

        /* init ACADOS */
        acados_ocp_capsule = quadrotor_q_acados_create_capsule();
        acados_status = quadrotor_q_acados_create(acados_ocp_capsule);
        if (acados_status)
        {
            ROS_ERROR("Cannot create the ACADOS solver");
        }
        nlp_config = quadrotor_q_acados_get_nlp_config(acados_ocp_capsule);
        nlp_dims = quadrotor_q_acados_get_nlp_dims(acados_ocp_capsule);
        nlp_in = quadrotor_q_acados_get_nlp_in(acados_ocp_capsule);
        nlp_out = quadrotor_q_acados_get_nlp_out(acados_ocp_capsule);

        time_horizon = nlp_dims->N;
        num_states = *nlp_dims->nx;
        num_controls = *nlp_dims->nu;

        trajectory_reference = Eigen::MatrixXd::Zero(time_horizon+1, num_states + num_controls);

        ROS_INFO_STREAM("Time horizon is "<<time_horizon<<", with "
        <<num_states<<" states, and "<< num_controls<<" controls");
    }

    MPCAcadosController::~MPCAcadosController() {}

    void MPCAcadosController::takeoff(ros::Time time_stamp, mavros_msgs::AttitudeTarget *msg)
    {
    }

    void MPCAcadosController::mission_loop(ros::Time time_stamp, mavros_msgs::AttitudeTarget *msg)
    {

        if (is_trajectory_provided_)
        {
            load_trajectory(trajectory_reference);
            ROS_INFO_ONCE("Got trajectory");
            solvingACADOS(trajectory_reference);
            msg->header.stamp = time_stamp;
            msg->type_mask = 128;
            msg->thrust = robot_command[3] / 9.8066 - 0.5;
            msg->body_rate.x = robot_command[0];
            msg->body_rate.y = robot_command[1];
            msg->body_rate.z = robot_command[2];
        }
        else
        {
            ROS_INFO("Waiting for trajectory");
        }
    }

    void MPCAcadosController::load_trajectory(Eigen::MatrixXd &trajectory)
    {
        boost::shared_lock<boost::shared_mutex> mutexTrajectoryCallback(mutexTrajectoryCallback_);

        int num_waypoints = trajectory_ref_point_->size;

        if (num_waypoints>1)
        {
            int num_diff = 0;
            if (num_waypoints != time_horizon)
            {
                num_diff = (time_horizon - num_waypoints > 0) ? (time_horizon - num_waypoints) : (-time_horizon + num_waypoints);
            }
            int num_min_waypoints = std::min(num_waypoints, time_horizon);

            for (int i = 0; i < num_min_waypoints; i++)
            {
                trajectory(i, 0) = trajectory_ref_point_->traj[i].x;
                trajectory(i, 1) = trajectory_ref_point_->traj[i].y;
                trajectory(i, 2) = trajectory_ref_point_->traj[i].z;
                if (trajectory_ref_point_->traj[i].quaternion_given)
                {
                    trajectory(i, 3) = trajectory_ref_point_->traj[i].q[0];
                    trajectory(i, 4) = trajectory_ref_point_->traj[i].q[1];
                    trajectory(i, 5) = trajectory_ref_point_->traj[i].q[2];
                    trajectory(i, 6) = trajectory_ref_point_->traj[i].q[3];
                }
                else{
                    /* convert rpy to quaternion */
                    Eigen::Vector3d rpy = {trajectory_ref_point_->traj[i].roll, trajectory_ref_point_->traj[i].pitch, trajectory_ref_point_->traj[i].yaw};
                    Eigen::Quaterniond quat;
                    rpy_to_quaternion(rpy, quat);
                    trajectory(i, 3) = quat.w();
                    trajectory(i, 4) = quat.x();
                    trajectory(i, 5) = quat.y();
                    trajectory(i, 6) = quat.z();
                }
                trajectory(i, 7) = trajectory_ref_point_->traj[i].vx;
                trajectory(i, 8) = trajectory_ref_point_->traj[i].vy;
                trajectory(i, 9) = trajectory_ref_point_->traj[i].vz;
                trajectory(i, 10) = 0.0;
                trajectory(i, 11) = 0.0;
                trajectory(i, 12) = 0.0;
                trajectory(i, 13) = 9.8066;
            }

            if (num_diff > 0)
            {
                for (int i = num_min_waypoints; i < num_min_waypoints + num_diff; i++)
                {
                    /* repeat last required waypoint */
                    trajectory(i) = trajectory(num_min_waypoints - 1);
                }
            }
        }
        else{
            /* only a fixed point is provided */
            for (int i = 0; i < time_horizon+1; i++)
            {
                trajectory(i, 0) = trajectory_ref_point_->traj[0].x;
                trajectory(i, 1) = trajectory_ref_point_->traj[0].y;
                trajectory(i, 2) = trajectory_ref_point_->traj[0].z;
                if (trajectory_ref_point_->traj[0].quaternion_given)
                {
                    trajectory(i, 3) = trajectory_ref_point_->traj[0].q[0];
                    trajectory(i, 4) = trajectory_ref_point_->traj[0].q[1];
                    trajectory(i, 5) = trajectory_ref_point_->traj[0].q[2];
                    trajectory(i, 6) = trajectory_ref_point_->traj[0].q[3];
                }
                else{
                    /* convert rpy to quaternion */
                    Eigen::Vector3d rpy = {trajectory_ref_point_->traj[0].roll, trajectory_ref_point_->traj[0].pitch, trajectory_ref_point_->traj[0].yaw};
                    Eigen::Quaterniond quat;
                    rpy_to_quaternion(rpy, quat);
                    trajectory(i, 3) = quat.w();
                    trajectory(i, 4) = quat.x();
                    trajectory(i, 5) = quat.y();
                    trajectory(i, 6) = quat.z();
                }
                trajectory(i, 7) = trajectory_ref_point_->traj[0].vx;
                trajectory(i, 8) = trajectory_ref_point_->traj[0].vy;
                trajectory(i, 9) = trajectory_ref_point_->traj[0].vz;
                trajectory(i, 10) = 0.0;
                trajectory(i, 11) = 0.0;
                trajectory(i, 12) = 0.0;
                trajectory(i, 13) = 9.8066;
            }
        }
    }

    void MPCAcadosController::solvingACADOS(Eigen::MatrixXd ref)
    {
        std::vector<double> end_term_ref;
        for (int i = 0; i < 3; i++)
        {
            end_term_ref.push_back(ref(time_horizon, i));
        }
        for (int i = 7; i < num_states; i++)
        {
            end_term_ref.push_back(ref(time_horizon, i));
        }
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, time_horizon, "yref", &end_term_ref[0]);


        for (int i = 0; i < time_horizon; i++)
        {
            std::vector<double> y_ref;
            for (int j = 0; j < num_controls + num_states; j++)
            {
                y_ref.push_back(ref(i, j));
            }
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", &y_ref[0]);
        }
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", robot_current_state);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", robot_current_state);

        // solve MPC
        acados_status = quadrotor_q_acados_solve(acados_ocp_capsule);
        if (acados_status)
        {
            robot_command[0] = 0.0;
            robot_command[1] = 0.0;
            robot_command[2] = 0.0;
            robot_command[3] = 9.8066;
        }
        else
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &robot_command);

    }

    /* callback functions */
    void MPCAcadosController::robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_pose_ = *msg;
        if (!is_current_pose_sub_)
        {
            is_current_pose_sub_ = true;
            initial_pose_ = *msg;
        }
    }

    void MPCAcadosController::trajectory_callback(const drone_mpc_pkg::itm_trajectory_msg::ConstPtr &msg)
    {
        if (!is_trajectory_provided_)
            is_trajectory_provided_ = true;
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexTrajectoryCallback_);
            trajectory_ref_point_ = std::make_shared<drone_mpc_pkg::itm_trajectory_msg>(*msg);
        }
    }

    /**
     * @description:
     *      get the current odometry of the robot
     * @param {const} nav_msgs ptr
     * @return {None}
     */
    void MPCAcadosController::current_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        robot_current_state[0] = msg->pose.pose.position.x;
        robot_current_state[1] = msg->pose.pose.position.y;
        robot_current_state[2] = msg->pose.pose.position.z;
        robot_current_state[3] = msg->pose.pose.orientation.w;
        robot_current_state[4] = msg->pose.pose.orientation.x;
        robot_current_state[5] = msg->pose.pose.orientation.y;
        robot_current_state[6] = msg->pose.pose.orientation.z;
        robot_current_state[7] = msg->twist.twist.linear.x;
        robot_current_state[8] = msg->twist.twist.linear.y;
        robot_current_state[9] = msg->twist.twist.linear.z;
        if (!is_current_pose_sub_)
        {
            is_current_pose_sub_ = true;
            for (int i = 0; i < num_states; i++)
                robot_init_state[i] = robot_current_state[i];
        }
    }

    bool MPCAcadosController::controller_server_response_callback(drone_mpc_pkg::GetControllerState::Request &req,
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

    bool MPCAcadosController::is_initialized()
    {
        if (is_robot_client_connected_ && is_current_pose_sub_)
            return true;
        else
            return false;
    }

    void MPCAcadosController::rpy_to_quaternion(Eigen::Vector3d rpy, Eigen::Quaterniond& q)
    {
        double cy = std::cos(rpy[2]*0.5);
        double sy = std::sin(rpy[2]*0.5);
        double cp = std::cos(rpy[1]*0.5);
        double sp = std::sin(rpy[1]*0.5);
        double cr = std::cos(rpy[0]*0.5);
        double sr = std::sin(rpy[0]*0.5);

        q.w() = cr * cp * cy + sr * sp * sy;
        q.x() = sr * cp * cy - cr * sp * sy;
        q.y() = cr * sp * cy + sr * cp * sy;
        q.z() = cr * cp * sy - sr * sp * cy;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "QuadrotorMPCNode");
    ros::NodeHandle nh_node, private_nh_node("~");
    acados_quadrotor::MPCAcadosController controller_node(nh_node, private_nh_node);
    ros::Rate rate(100);

    /* publisher */
    ros::Publisher command_attitude_thrust_pub = nh_node.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 100);

    mavros_msgs::AttitudeTargetPtr attitude_msg(new mavros_msgs::AttitudeTarget);

    while (ros::ok())
    {
        if (controller_node.is_initialized())
        {
            switch (controller_node.command_id)
            {
                case 0: // idle
                    // controller_node.idle();
                    break;
                case 1: // takeoff
                {
                    controller_node.takeoff(ros::Time::now(), attitude_msg.get());
                    command_attitude_thrust_pub.publish(attitude_msg);
                    ROS_INFO_ONCE("Take off");
                    break;
                }
                case 2: // landing
                {
                    // this is implement by px4 controller already
                    break;
                }
                case 3: // go to position
                {
                    controller_node.mission_loop(ros::Time::now(), attitude_msg.get());
                    command_attitude_thrust_pub.publish(attitude_msg);
                    ROS_INFO_ONCE("Go to Position Mode");
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

    ROS_INFO("MPC controller is closed");
    return 0;
}