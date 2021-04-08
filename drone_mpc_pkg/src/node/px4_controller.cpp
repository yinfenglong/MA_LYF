

/*
 * @Author: Wei Luo
 * @Date: 2021-03-14 23:23:46
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-04-08 23:29:23
 * @Note: To control an experiment quadrotor with different
 *      controllers.
 */

// include header files
#include <drone_mpc_pkg/node/px4_controller.hpp>

/* -- main function -- */
int main(int argc, char **argv)
{
    // init ros node
    ros::init(argc, argv, "px4_controller");
    ros::NodeHandle nh_node, private_nh_node("~");

    // init variables
    /* user command mode
        0 -- idle
        1 -- take off
        2 -- (emergency) landing
        3 --
    */
    user_command.mission_mode = 0;
    /* default controller_state is false */
    controller_state.response.connected = false;

    uav_mode.request.mode = 0;

    /*Subscribers*/
    ros::Subscriber state_sub = nh_node.subscribe<mavros_msgs::State>("/mavros/state", 10, mavros_state_cb);
    ros::Subscriber user_command_sub = nh_node.subscribe<drone_mpc_pkg::SetMission>("itm_quadrotor_control/user_command", 10, user_command_callback);
    ros::Subscriber uav_state_sub = nh_node.subscribe<nav_msgs::Odometry>("/robot_pose", 10, current_odom_callback);
    /*ServiceClient*/
    ros::ServiceClient arming_client = nh_node.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh_node.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient set_mission_mode = nh_node.serviceClient<drone_mpc_pkg::SetMode>("/itm_quadrotor_control/setmode");
    ros::ServiceClient get_controller_state = nh_node.serviceClient<drone_mpc_pkg::GetControllerState>("/itm_quadrotor_control/get_controller_state");
    /*Publisher*/
    robot_command = nh_node.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    /*the setpoint publishing rate MUST be faster than 2Hz*/
    ros::Rate rate(50.0);

    /* get the controller type */
    if (private_nh_node.hasParam("controller"))
    {
        private_nh_node.getParam("controller", controller_type);
    }
    else
    {
        ROS_ERROR("NO controller specified, please provide a controller server");
    }


    /*wait for FCU connection */
    while(ros::ok() && !mavros_state.connected && !current_pos_flag)
    {
        // if get state from ROS and also the localization of the quadrotor
        ros::spinOnce();
        rate.sleep();
    }

    /*inertial takeoff height*/
    command_pose.pose.position.x = 0;
    command_pose.pose.position.y = 0;
    command_pose.pose.position.z = 1;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        robot_command.publish(command_pose);
        ros::spinOnce();
        rate.sleep();
    }

    /* check the controller state */
    while (ros::ok() && !controller_state.response.connected){
        // check the controller state through server
        ROS_INFO("Connect to Controller Server.");
        controller_state.request.robot_name = "ITM_Q330";
        controller_state.request.command_id = 0;
        get_controller_state.call(controller_state);
        ros::spinOnce();
        rate.sleep();
    }
    is_controller_working = true;
    ROS_INFO("Get connection with Controller Server");

    // /* check the Mission state */
    // while (ros::ok() && !uav_mode.response.success)
    // {
    //     ROS_INFO_ONCE("Waiting for a Mission Node");
    //     if (controller_type==0) // PID controller
    //         set_mission_mode.call(uav_mode);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    /*set offboard and armed*/
    offboard_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    last_request = ros::Time::now();
    ROS_INFO("Begin the loop");
    // main loop
    while (ros::ok())
    {
        if( mavros_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))&&
            user_command.mission_mode!=2){
            if( set_mode_client.call(offboard_set_mode) &&
                offboard_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !mavros_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(4.5)) &&
                user_command.mission_mode!=2){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO_ONCE("Vehicle armed");
                }
                else{
                    ROS_INFO("cannot arm");
                }
                last_request = ros::Time::now();
            }
        }

        // once the vehicle is ready to fly
        if (mavros_state.armed)
        {
            controller_state.request.command_id = user_command.mission_mode;
            get_controller_state.call(controller_state);
            if (!controller_state.response.connected)
            {
                // controller no response
                is_controller_working = false;
            }
            // robot_command.publish(command_pose);
            ROS_INFO_ONCE("Begin the Mission");
        }

        if (user_command.mission_mode==2 || !is_controller_working){
            // landing immediately
            ROS_INFO_ONCE("Start Landing.");
            landing();
            if (abs(current_pos.pose.position.z-initial_pos.pose.position.z)<=0.1)
            {
                // disarm the safely lock the UAV
                if (mavros_state.mode == "OFFBOARD")
                {
                    offboard_set_mode.request.custom_mode = "MANUAL";
                    // offboard_set_mode.request.custom_mode = "AUTO.LOITER";
                    // offboard_set_mode.request.custom_mode = "AUTO.LAND";
                    if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent)
                    {
                        ROS_INFO_ONCE("Landing detected and set to landing");
                    }
                }
                else if ( mavros_state.mode == "AUTO.RTL" || mavros_state.mode == "AUTO.LOITER")
                {
                    offboard_set_mode.request.custom_mode = "MANUAL";
                    if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent)
                    {
                        ROS_INFO_ONCE("Landing and set to MANUAL");
                    }
                }
                else if (mavros_state.mode == "MANUAL"|| mavros_state.mode == "AUTO.RTL"
                || mavros_state.mode == "AUTO.LOITER")
                {
                    if (mavros_state.armed)
                    {
                        arm_cmd.request.value = false;
                        arming_client.call(arm_cmd);
                    }
                    if (arm_cmd.response.success)
                    {
                        ROS_INFO_ONCE("Safely Disarm");
                    }
                    else
                    {
                        offboard_set_mode.request.custom_mode = "MANUAL";
                        if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent)
                        {
                            ROS_INFO_ONCE("Landing detected and set to POSITION");
                        }
                    }
                }
                else{
                    std::cout<<mavros_state.mode<<std::endl;
                    ROS_ERROR("Unknown state");
                }

            }
            else{
                printf("distance %f \n", abs(current_pos.pose.position.z-initial_pos.pose.position.z));
                printf("current height %f, initial height %f \n",current_pos.pose.position.z, initial_pos.pose.position.z);
            }
        }

        last_user_command = user_command;
        ros::spinOnce();
        rate.sleep();
    }
}
