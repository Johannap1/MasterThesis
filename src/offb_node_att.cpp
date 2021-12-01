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

    //Subsribed Topics
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/nxp_drone_0/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/nxp_drone_0/mavros/local_position/pose", 10, pos_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/nxp_drone_0/mavros/local_position/velocity_body", 10, vel_cb);
    ros::Subscriber local_acc_sub = nh.subscribe<sensor_msgs::Imu>
            ("/nxp_drone_0/mavros/imu/data", 10, acc_cb);

    //Published Topics
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/nxp_drone_0/mavros/setpoint_position/local", 10);
    ros::Publisher local_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
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
    int inp_type = 1; //choose: STEP = 0, PRBS = 1
    int contr_type = 0; //choose: ATT= 0, POS = 1
    int exp_type = 3; //choose: PITCH = 0, ROLL = 1, YAW = 2, YAW_RATE = 3, VZ = 4

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Waiting for FCU connection");
    }

    // Initialize the attitude commands
    double v[3]={1.0, 0.0, 0.0}; 
    double v_norm=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    double theta=0; 

    mavros_msgs::AttitudeTarget cmd_att;

    cmd_att.header.stamp = ros::Time::now();
    cmd_att.header.frame_id = "map";
    cmd_att.type_mask = 0;
    
    //Position setpoint for takeoff
    bool takeoff = true;
    geometry_msgs::PoseStamped pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = cmd_alt;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
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

    //Set variables for experiment generator 
    double max_roll = 12*3.1415926535897932/180; //rad
    double max_pitch = 12*3.1415926535897932/180; //rad
    double max_yaw = 120*3.1415926535897932/180; //rad
    double max_yaw_rate = 100*3.1415926535897932/180; //rad/s
    double max_vz = 1; //m/s
    double att_to_vel = 0.5;

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
            if(ros::Time::now()-last_request > ros::Duration(15.0)){
                takeoff = false;
                time_inp = ros::Time::now();
                ROS_INFO("Leaving Takeoff");
            }
        }

        else{

            //generate time update for input signals
            if(update_time){
                time_inp = ros::Time::now();
            }
            
            //generate input sequence 
            double u = input.input(ros::Time::now(),time_inp);
            update_time = input.update();

            if(control_type == 0){
                //Compute commanded thrust using cascaded PID structure
                double dt = (ros::Time::now() - time_pid).toSec();
                double cmd_thrust =  pid.calculate(cmd_alt, current_pos.pose.position.z, current_vel.twist.linear.z, current_acc.linear_acceleration.z, dt);
                cmd_att.thrust = cmd_thrust;
                time_pid = ros::Time::now();

                //generate output based on experiment 
                switch(exp_type){
                    case 0: 
                        v[0]=1.0; 
                        theta = u*max_roll;
                        break;
                    case 1:
                        v[1]=1.0; 
                        theta = u*max_pitch;
                        break;
                    case 2:
                        v[2]=1.0;
                        theta = u*max_yaw;
                        break;
                    case 3:
                        cmd_att.body_rate.z = u*max_yaw_rate;
                        cmd_att.type_mask = 3;
                        break;
                    case 4: 
                        ROS_INFO("Not a valid experiment choice")
                        break;
                }

                v_norm=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
                cmd_att.orientation.x = sin(theta/2.0)*v[0]/v_norm;
                cmd_att.orientation.y = sin(theta/2.0)*v[1]/v_norm;
                cmd_att.orientation.z = sin(theta/2.0)*v[2]/v_norm;
                cmd_att.orientation.w = cos(theta/2.0);

                local_att_pub.publish(cmd_att);
            }

            else if(control_type == 1){
                switch(exp_type){
                    case 0: 
                        pose.position.x = NAN;
                        pose.position.y = NAN;
                        pose.velocity.y = u*max_roll*att_to_vel;
                        break;
                    case 1:
                        pose.position.x = NAN;
                        pose.position.y = NAN;
                        pose.velocity.x = u*max_pitch*att_to_vel;
                        break;
                    case 2:
                        pose.yaw = u*max_yaw;
                        break;
                    case 3:
                        pose.yaw_rate = u*max_yaw_rate;
                        break;
                    case 4: 
                        pose.velocity.z = u*max_vz;
                        break;
                }
                local_pos_pub.publish(pose);
            }
        }
      
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
