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
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/PositionTarget.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "pid.h"
#include "input_generator.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//Callback functions
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

//Initialize parameter names and variables. We define them as private members of the respective nodes.
const std::string PARAM_NAME1= "~input_type";
const std::string PARAM_NAME2= "~control_type";
const std::string PARAM_NAME3= "~experiment_type";
const std::string PARAM_NAME4= "~altitude_setpoint";
int inp_type;
int control_type;
int exp_type;
double alt_sp;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    //Subsribed Topics
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pos_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity_body", 10, vel_cb);
    ros::Subscriber local_acc_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data", 10, acc_cb);

    //Published Topics
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    ros::Publisher local_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    ros::ServiceClient rate_client = nh.serviceClient<mavros_msgs::MessageInterval>
            ("mavros/set_message_interval");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    //set streaming rate
    mavros_msgs::MessageInterval msg_interv;
    msg_interv.request.message_rate = 50;
    msg_interv.request.message_id = 30;
    rate_client.call(msg_interv);
    msg_interv.request.message_id = 31;
    rate_client.call(msg_interv);
    msg_interv.request.message_id = 32;
    rate_client.call(msg_interv);
    msg_interv.request.message_id = 83;
    rate_client.call(msg_interv);
    msg_interv.request.message_id = 85;
    rate_client.call(msg_interv);

    //Get the parameters
	bool ok = ros::param::get(PARAM_NAME1, inp_type);
	if(!ok) {
		ROS_FATAL_STREAM("could not get input type");
		exit(1);
	}
	ok = ros::param::get(PARAM_NAME2, control_type);
	if(!ok) {
		ROS_FATAL_STREAM("could not get control type");
		exit(1);
	}
    ok = ros::param::get(PARAM_NAME3, exp_type);
	if(!ok) {
		ROS_FATAL_STREAM("could not get experiment type");
		exit(1);
	}
    ok = ros::param::get(PARAM_NAME4, alt_sp);
	if(!ok) {
		ROS_FATAL_STREAM("could not get altitude setpoint");
		exit(1);
	}

    //Safety constraints. TODO:adjust according to the lab dimensions
    int max_x = 10;
    int max_y = 10;
    int max_z = 20;
    bool landing = false; 

    //Position setpoint for takeoff
    bool takeoff = true;
    mavros_msgs::PositionTarget pose;
    pose.coordinate_frame = 1;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = alt_sp;

    //Initialize landing Command
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;
   
    // Initialize the attitude commands
    double v[3]={0.0, 0.0, 0.0}; 
    double v_norm=1; // TODO: sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    double theta=0; 
    mavros_msgs::AttitudeTarget cmd_att;
    cmd_att.header.stamp = ros::Time::now();
    cmd_att.header.frame_id = 1;
    cmd_att.type_mask = 0;

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Waiting for FCU connection");
    }

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    //if offboard control is lost when not connected to an RC: Land
    //if offboard control is lost when connected to an RC: Position Control (current position)
    //1 s Timeout

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    //record a bunch of times (TODO: check which ones are actually used)
    ros::Time last_request = ros::Time::now();
    ros::Time time_pid = ros::Time::now();
    ros::Time time_inp = ros::Time::now();

    //Initialize PID controller
    PID pid=PID(1.f, -1.f, 0.9, 0.1, 0.5, 1.f, 10.f, 0.001, 0.1);

    //Initialize Input generator
    double max_freq = 0.5;//4; //2,10
    double min_freq = 0.1;//0.8; //0.8 0.4,12
    double samp_freq = 50.f;
    double step_time = 5.f;
    INPUT input=INPUT(inp_type, max_freq, min_freq, samp_freq, step_time);
    bool update_time = true;
    std::srand(time(0)); //seed ranom number sequence

    //Set variables for experiment generator 
    double max_roll = 12*3.1415926535897932/180; //rad
    double max_pitch = 12*3.1415926535897932/180; //rad
    double max_yaw = 120*3.1415926535897932/180; //rad
    double max_yaw_rate = 200*3.1415926535897932/180; //rad/s
    double max_vz = 1; //m/s
    double att_to_vel = 10;

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

        //Safety check for lab bounds:
        if(current_pos.pose.position.z > max_z || abs(current_pos.pose.position.y) > max_y || abs(current_pos.pose.position.x) > max_x){
            landing = true;
        }

        if(landing){
            last_request = ros::Time::now();
            if(land_client.call(land_cmd) && land_cmd.response.success){
                ROS_INFO_ONCE("Landing");
                ros::shutdown();
            }
        }
        
        if(takeoff){
            pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(pose);
            ROS_INFO_ONCE("Takeoff");
            if(ros::Time::now()-last_request > ros::Duration(15.0)){ //15.0
                takeoff = false;
                time_inp = ros::Time::now();
                ROS_INFO("Leaving Takeoff");
                last_request = ros::Time::now();
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
                double cmd_thrust =  pid.calculate(alt_sp, current_pos.pose.position.z, current_vel.twist.linear.z, current_acc.linear_acceleration.z, dt);
                cmd_att.thrust = cmd_thrust;
                time_pid = ros::Time::now();

                //TODO: uncomment to work with euler angles instead of quaternions directly
                //tf2::Quaternion q;

                //generate output based on experiment 
                switch(exp_type){
                    case 0: 
                        v[0]=1.0; 
                        theta = u*max_roll;
                        //q.setRPY(theta, 0, 0);
                        break;
                    case 1:
                        v[1]=1.0; 
                        theta = u*max_pitch;
                        break;
                    case 2:
                        v[2]=1.0;
                        theta = u*max_yaw;
                        break;
                    case 3: //TODO: yaw rate has feedforward term from yaw which messes things up
                        v[0]=1.0; 
                        cmd_att.body_rate.z = u*max_yaw_rate;
                        cmd_att.type_mask = 3;
                        break;
                    case 4: 
                        ROS_INFO_ONCE("Not a valid experiment choice");
                        break;
                    case 5:
                        v[0]=1.0; 
                        theta = 0;
                        cmd_att.thrust = 0.2*u+0.67;
                        break;
                }

                v_norm = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
                cmd_att.orientation.x = sin(theta/2.0)*v[0]/v_norm; //q[0]
                cmd_att.orientation.y = sin(theta/2.0)*v[1]/v_norm; //q[1]
                cmd_att.orientation.z = sin(theta/2.0)*v[2]/v_norm; //q[2]
                cmd_att.orientation.w = cos(theta/2.0); //q[3]
                cmd_att.header.stamp = ros::Time::now();
                local_att_pub.publish(cmd_att);

            }

            else if(control_type == 1){
                switch(exp_type){
                    case 0: 
                        pose.position.x = NAN;
                        pose.position.y = NAN;
                        pose.velocity.y = -u*max_roll*att_to_vel;
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
                        pose.position.z = NAN;
                        pose.type_mask = 4;
                        pose.velocity.z = u*max_vz;
                        break;
                    case 5:
                        ROS_INFO_ONCE("Not a valid experiment choice");
                        break;
                }
                pose.header.stamp = ros::Time::now();
                local_pos_pub.publish(pose);
            }

            //Terminate experiment some time after takeoff 
            if(ros::Time::now()-last_request > ros::Duration(120.0)){
                landing = true;
            }
        }
      
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
