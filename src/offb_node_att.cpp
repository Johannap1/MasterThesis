/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "pid.h"
#include "input_generator.h"

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pos = *msg;
}
geometry_msgs::TwistStamped current_vel;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_vel = *msg;
}
sensor_msgs::Imu current_acc;
void acc_cb(const sensor_msgs::Imu::ConstPtr& msg){
    current_acc = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/nxp_drone_0/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/nxp_drone_0/mavros/local_position/pose", 10, pos_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/nxp_drone_0/mavros/local_position/velocity_body", 10, vel_cb);
    ros::Subscriber local_acc_sub = nh.subscribe<sensor_msgs::Imu>
            ("/nxp_drone_0/mavros/imu/data", 10, acc_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/nxp_drone_0/mavros/setpoint_position/local", 10);
    ros::Publisher local_thr_pub = nh.advertise<std_msgs::Float64>
            ("/nxp_drone_0/mavros/setpoint_attitude/att_throttle", 10);
    ros::Publisher local_att_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/nxp_drone_0/mavros/setpoint_attitude/attitude", 10);
    ros::Publisher local_att1_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("/nxp_drone_0/mavros/setpoint_raw/attitude", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/nxp_drone_0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/nxp_drone_0/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    //Altitude setpoint
    double cmd_alt = 5;

    //Input type
    int inp_type = 1; //choose: STEP = 1, PRBS = 2;

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Waiting for FCU connection");
    }

    double v[3]={1.0, 0.0, 0.0}; 
    double v_norm=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    double theta=0; 

    bool takeoff = true;
    
    //Position setpoint for takeoff
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = cmd_alt;

    mavros_msgs::AttitudeTarget cmd_att;

    cmd_att.header.stamp = ros::Time::now();
    cmd_att.header.frame_id = "map";
    cmd_att.type_mask = 0;
    cmd_att.orientation.x = sin(theta/2.0)*v[0]/v_norm;
    cmd_att.orientation.y = sin(theta/2.0)*v[1]/v_norm;
    cmd_att.orientation.z = sin(theta/2.0)*v[2]/v_norm;
    cmd_att.orientation.w = cos(theta/2.0);
    cmd_att.thrust = 1;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        //local_att1_pub.publish(cmd_att);
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    ros::Time time_pid = ros::Time::now();

    ros::Time time_inp = ros::Time::now();

    //Initialize PID controller
    PID pid=PID(1.f, -1.f, 0.9, 0.1, 0.5, 1.f, 10.f, 0.001, 0.1);

    //Initialize Input generator
    double max_freq = 2.f;
    double min_freq = 0.2;
    double samp_freq = 20.f;
    double step_time = 5.f;

    INPUT input=INPUT(inp_type, max_freq, min_freq, samp_freq, step_time);

    bool update_time = true;

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                ROS_INFO("Arming failed");
                last_request = ros::Time::now();
            }
        }
        
        if(takeoff){
            local_pos_pub.publish(pose);
            ROS_INFO_ONCE("Takeoff");
            if(ros::Time::now()-last_request > ros::Duration(5.0)){
                takeoff = false;
                time_inp = ros::Time::now();
                ROS_INFO("Leaving Takeoff");
            }
        }

        else{
            //Compute commanded thrust using cascaded PID structure
            double dt = (ros::Time::now() - time_pid).toSec();
            double cmd_thrust =  pid.calculate(cmd_alt, current_pos.pose.position.z, current_vel.twist.linear.z, current_acc.linear_acceleration.z, dt);
            cmd_att.thrust = cmd_thrust;
            time_pid = ros::Time::now();

            //generate time update for input signals
            if(update_time){
                time_inp = ros::Time::now();
            }
            
            //generate PRBS sequence 
            double u = input.input(ros::Time::now(),time_inp);
            update_time = input.update();
            /*
            
            double d;
            if(inp_type == 2 && prbs){
                int count = prbs_gen(max_freq, min_freq, samp_freq);
                d = (count/samp_freq);
                std::cout<<d<<std::endl;
                prbs = false;
            }
            if(u >= 1 && (ros::Time::now()-t_prbs)>ros::Duration(d)){
                u = -1;
                prbs = true;
                ROS_INFO("NOW+");
                t_prbs = ros::Time::now();
            }
            else if(u < 1 && (ros::Time::now()-t_prbs)>ros::Duration(d)){
                u = +1;
                prbs = true;
                ROS_INFO("NOW-");
                t_prbs = ros::Time::now();
            }

            */

            std::cout<<u<<std::endl;


            //generate input signal 
            double t_inp = (ros::Time::now()-time_inp).toSec();



            if ((ros::Time::now() - time_inp)>ros::Duration(5.0)){
                double c = (std::rand()%20)-10;
                theta = c* 3.1415926535897932/180;
                std::cout<<theta<<std::endl;
                cmd_att.orientation.x = sin(theta/2.0)*v[0]/v_norm;
                cmd_att.orientation.y = sin(theta/2.0)*v[1]/v_norm;
                cmd_att.orientation.z = sin(theta/2.0)*v[2]/v_norm;
                cmd_att.orientation.w = cos(theta/2.0);
                time_inp = ros::Time::now();
            }


            /* DEBUGGING
            std::cout<<"Velocity Setpoint: "<<vel_sp_z<<std::endl;
            std::cout<<"Acceleration Setpoint: "<<acc_sp_z<<std::endl;
            std::cout<<"Commanded Thrust: "<<cmd_att.thrust<<std::endl;
            std::cout<<vel_error<<" "<<current_acc.linear_acceleration.z<<" "<<vel_int<<" "<<dt<<std::endl;
            */
            local_att1_pub.publish(cmd_att);
        }
      
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
