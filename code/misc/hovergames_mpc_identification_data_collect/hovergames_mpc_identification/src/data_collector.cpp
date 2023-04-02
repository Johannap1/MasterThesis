#include "data_collector.h"
#include <ros/ros.h>
#include <iostream>
#include <fstream>

bool DataCollector::initialize()
{
    // ROS subscribers
    state_sub = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &DataCollector::state_cb, this);
    input_sub = nh_.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 1, &DataCollector::input_cb, this);
    data_toggle_sub = nh_.subscribe<std_msgs::Bool>("/drone_hovergames/record_data", 1, &DataCollector::toggle_cb, this);
    prediction_pub = nh_.advertise<nav_msgs::Odometry>("/drone_hovergames/state_prediction", 1);
    wind_pub = nh_.advertise<geometry_msgs::AccelStamped>("/drone_hovergames/wind", 1);
    save_trigger = nh_.subscribe<std_msgs::Bool>("/save_data",1, &DataCollector::save_cb, this);
    return true;
};

void DataCollector::state_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    now1 = msg->header.stamp.sec + msg->header.stamp.nsec*1e-9;
    current_pos = msg->pose.pose;
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(current_roll, current_pitch, current_yaw);
    current_vel = msg->twist.twist;
    current_state = {msg->pose.pose.position.x, 
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z,
                    msg->twist.twist.linear.x,
                    msg->twist.twist.linear.y,
                    msg->twist.twist.linear.z,
                    current_roll,
                    current_pitch,
                    current_yaw};
    wind_x = (state_prediction[3]-current_state[3])/dt;
    wind_y = (state_prediction[4]-current_state[4])/dt;
    wind_msg.accel.linear.x = wind_x;
    wind_msg.accel.linear.y = wind_y;
    wind_msg.accel.angular.x = current_state[0];
    wind_msg.accel.angular.y = current_state[1];
    wind_msg.accel.angular.z = current_state[2];
    wind_msg.header.stamp = ros::Time::now();

    if(toggle_){
        wind_pub.publish(wind_msg);
    }
}

void DataCollector::input_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    now2 = msg->header.stamp.sec + msg->header.stamp.nsec*1e-9;
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(input_roll, input_pitch, input_yaw);
    current_input = {input_roll, 
                    input_pitch,
                    input_yaw,
                    msg->thrust};
    timing1.push_back(now1);
    state_vel.push_back(current_state[4]);
    state_att.push_back(current_state[6]);
    input_cmd.push_back(current_input[0]);
    state_prediction = solve_rk4(current_state, current_input, dt);
    state_prediction_msg.pose.pose.position.x = state_prediction[0];
    state_prediction_msg.pose.pose.position.y = state_prediction[1];
    state_prediction_msg.pose.pose.position.z = state_prediction[2];
    state_prediction_msg.twist.twist.linear.x = state_prediction[3];
    state_prediction_msg.twist.twist.linear.y = state_prediction[4];
    state_prediction_msg.twist.twist.linear.z = state_prediction[5];
    state_pred.push_back(state_prediction[4]);
    timing2.push_back(now2);
    q.setRPY(state_prediction[6], state_prediction[7], state_prediction[8]);
    state_prediction_msg.pose.pose.orientation.x = q[0];
    state_prediction_msg.pose.pose.orientation.y = q[1];
    state_prediction_msg.pose.pose.orientation.z = q[2];
    state_prediction_msg.pose.pose.orientation.w = q[3]; 
    state_prediction_msg.header.stamp = ros::Time::now();
    prediction_pub.publish(state_prediction_msg);
}

std::vector<double> DataCollector::multiplyvector(std::vector<double> v, double k){
    std::vector<double> v_return;
    for (int i = 0; i <= v.size(); ++i){
        v_return.push_back(v[i]*k);
    }
    return v_return;
}

std::vector<double> DataCollector::addvector(std::vector<double> v1, std::vector<double> v2){
    std::vector<double> v_return;
    for (int i = 0; i <= v1.size(); ++i){
        v_return.push_back(v1[i]+v2[i]);
    }
    return v_return;
}

std::vector<double> DataCollector::continuous_model(double dt, std::vector<double> x, std::vector<double> u){

    double vx = x[3];
    double vy = x[4];
    double vz = x[5];
    double phi = x[6];
    double theta = x[7];
    double phi_c = u[0];
    double theta_c = u[1];
    double dpsi_c = u[2];
    double thrust_c = u[3];

    double x_dot = vx;
    double y_dot = vy;
    double z_dot = vz;

    double vx_dot = g * theta;
    double vy_dot = -g * phi;
    double vz_dot = thrust_c - g;

    double phi_dot = (k_phi * phi_c - phi) / tau_phi;
    double theta_dot = (k_theta * theta_c - theta) / tau_theta;
    double psi_dot = dpsi_c;

    return {x_dot, y_dot, z_dot, vx_dot, vy_dot, vz_dot, phi_dot, theta_dot, psi_dot};
}

std::vector<double> DataCollector::solve_rk4(std::vector<double> state_cur, std::vector<double> input, double dt){
    std::vector<double> k1 = continuous_model(dt, state_cur, input);
    std::vector<double> v_next = addvector(state_cur, multiplyvector(k1,0.5*dt));
    std::vector<double> k2 = continuous_model(dt, v_next ,input);
    v_next = addvector(state_cur, multiplyvector(k2, 0.5*dt));
    std::vector<double> k3 = continuous_model(dt, v_next, input);
    v_next = addvector(state_cur, multiplyvector(k3, dt));
    std::vector<double> k4 = continuous_model(dt, v_next, input);
    v_next = addvector(multiplyvector(k2,2),multiplyvector(k3,2));
    v_next = addvector(k1, v_next);
    v_next = addvector(k4, v_next);
    return addvector(state_cur, multiplyvector(v_next, 0.16666666666 * dt));
}

void DataCollector::save_cb(const std_msgs::Bool::ConstPtr& msg){
    std::ofstream myFile("/home/johanna/test.csv");
    for (int n=0; n<timing1.size(); n++)
    {
        myFile << timing1[n] << "," << timing2[n] << "," << state_vel[n] << "," << input_cmd[n] << "," << state_att[n] << "," << state_pred[n] << std::endl;
    }
}

void DataCollector::toggle_cb(const std_msgs::Bool::ConstPtr& msg){
    toggle_ = msg->data;
}