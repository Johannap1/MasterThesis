#include "mpc_identification_exp.h"
#include "helpers.h"

bool MPCIdentificationExp::initialize()
{

    // Set timeout for error printing
    print_timeout_ = ros::Duration(1.0);

    // Enable debug for message printing
    enable_debug_ = true;

    // ROS subscribers
    state_sub = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &MPCIdentificationExp::state_cb, this);
    local_acc_sub = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, &MPCIdentificationExp::acc_cb, this);
    battery_sub = nh_.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 1, &MPCIdentificationExp::batt_cb, this);
    

    // ROS publishers
    local_pos_pub = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    local_att_pub = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    rate_client = nh_.serviceClient<mavros_msgs::MessageInterval>("/mavros/set_message_interval");
    record_data_pub = nh_.advertise<std_msgs::Bool>("/drone_hovergames/record_data", 1);
    
    // ROS service clients
    mission_finished_client_ = nh_.serviceClient<std_srvs::Trigger>("/px4_mission_finished_ext_cont");

    // ROS service servers
    enable_control_sever_ = nh_.advertiseService("/px4_ext_cont_enable", &MPCIdentificationExp::enableControlCallback, this);
    disable_control_server_ = nh_.advertiseService("/px4_ext_cont_disable", &MPCIdentificationExp::disableControlCallback, this);

    // Store ROS parameters
    if (!getRosParameters()) {
        ROS_ERROR_STREAM(ros::this_node::getName().c_str() << " Failed to obtain ROS parameters!");
        return false;
    }

    // Set loop frequency to 50 Hz
    loop_frequency_ = 20;

    waiting_ = true;

    // Initialize PID controller
    hover_thrust = 0.66;
    pid.initialize(max_vel, -max_vel, max_thrust, min_thrust, hover_thrust, kp_pos, ki_pos, kp_vel, kd_vel, ki_vel);
    lqr.initialize(Q_11, Q_22, Q_33, ki_LQR, is_sim_, loop_frequency_);    

    // Initialize position commands
    pose.coordinate_frame = 1; //TODO: check this
    pose.position.x = exp_sp[0];
    pose.position.y = exp_sp[1];
    pose.position.z = exp_sp[2];

    // Initialize the attitude commands
    cmd_att.header.stamp = ros::Time::now();
    cmd_att.header.frame_id = "base_link";
    cmd_att.type_mask = 0;

    // Initialize time steps
    ros::Time last_request = ros::Time::now();
    ros::Time time_pid = ros::Time::now();
    ros::Time time_inp = ros::Time::now();

    // Initialize Input generator
    samp_freq = loop_frequency_;
    step_time = 1.f; //Number of seconds before experiment begins
    input.initialize(inp_type, max_freq, min_freq, samp_freq, step_time);
    update_time = true;
    std::srand(time(0)); //seed random number sequence

    if (!initConnection()) {
        ROS_ERROR_STREAM(ros::this_node::getName().c_str() << " Failed to initialize drone connection!");
        return false;
    }

    loop_timer_.stop();

    return true;
}

bool MPCIdentificationExp::initConnection()
{
    ros::Rate rate(loop_frequency_);

    // Run MPCIdentificationExp::loop() at 50 Hz
    loop_timer_ = nh_.createTimer(ros::Duration(1/loop_frequency_), &MPCIdentificationExp::loop, this);
    return true;
}

bool MPCIdentificationExp::getRosParameters()
{
    /* General interface parameters */
    retrieveParameter(nh_, "loop_frequency", loop_frequency_, loop_frequency_default_);

    /* 3D positions parameters */
    // Required
    if (!retrieveParameter(nh_, "input_type", inp_type)) return false;
    if (!retrieveParameter(nh_, "control_type", control_type)) return false;
    if (!retrieveParameter(nh_, "experiment_type", exp_type)) return false;
    if (!retrieveParameter(nh_, "pos/start", exp_sp)) return false;
    if (!retrieveParameter(nh_, "position_setpoint", pos_sp)) return false;
    if (!retrieveParameter(nh_, "velocity_setpoint", vel_sp)) return false;
    if (!retrieveParameter(nh_, "max_frequency", max_freq)) return false;
    if (!retrieveParameter(nh_, "min_frequency", min_freq)) return false;
    if (!retrieveParameter(nh_, "experiment_duration", exp_duration)) return false;
    // Non-required with default value
    retrieveParameter(nh_, "is_sim", is_sim_, is_sim_default_);
    retrieveParameter(nh_, "roll", max_roll, max_roll_default);
    retrieveParameter(nh_, "pitch", max_pitch, max_pitch_default);
    // Deg to rad
    max_roll = max_roll*3.1415926535897932/180;
    max_pitch = max_pitch*3.1415926535897932/180;
    return true;
}

void MPCIdentificationExp::loop(const ros::TimerEvent &event){

    if(!streamed){
        // Set streaming rates
        SetStreamingRates(loop_frequency_);  
        streamed = true;
    }

    // Generate time update for input signals
    if(update_time){
        time_inp = ros::Time::now();
    }
    
    // Generate input sequence 
    u = input.computeInput(ros::Time::now(),time_inp);

    update_time = input.update();

    if(control_type == "tuning"){
        CONTROLLER_INFO_ONCE("Attitude control tuning mode");
        double tune_sp = u*pos_sp + exp_sp[2];
        dt = (ros::Time::now() - time_pid).toSec();
        cmd_thrust =  pid.calculate(tune_sp, current_pos.position.z, current_vel.linear.z, current_acc.linear_acceleration.z, dt);
        cmd_att.thrust = cmd_thrust;
        time_pid = ros::Time::now();
        phi = 0;
        theta = 0;
        xi = 0;
        q.setRPY(phi, theta, xi);
        cmd_att.orientation.x = q[0];
        cmd_att.orientation.y = q[1];
        cmd_att.orientation.z = q[2];
        cmd_att.orientation.w = q[3];
        cmd_att.header.stamp = ros::Time::now();
        local_att_pub.publish(cmd_att);
    }

    if(control_type == "attitude"){
        CONTROLLER_INFO_ONCE("Attitude control mode");
        // Compute commanded thrust using cascaded PID structure
        dt = (ros::Time::now() - time_pid).toSec();
        cmd_thrust =  pid.calculate(exp_sp[2], current_pos.position.z, current_vel.linear.z, current_acc.linear_acceleration.z, dt);
        cmd_att.thrust = cmd_thrust;
        time_pid = ros::Time::now();

        // For step signal, change direction quadrotor before position setpoint is reached
        if(inp_type == 0){
            if(exp_type == 0 && abs(pos_sp-current_pos.position.y) < 0.1){
                pos_sp = -pos_sp;
            }
            if(exp_type == 1 && abs(pos_sp-current_pos.position.x) < 0.1){
                pos_sp = -pos_sp;
            }
            if(pos_sp<0){
                u=-u;
            }
        }

        // Generate output based on experiment 
        switch(exp_type){
            case 0: 
                phi = -u*max_roll;
                theta = lqr.calculate(exp_sp[0], current_pos.position.x, current_vel.linear.x, current_pitch, dt, 1);
                xi = 0;
                q.setRPY(phi, theta, xi);
                break;
            case 1:
                phi = lqr.calculate(exp_sp[1], current_pos.position.y, current_vel.linear.y, current_roll, dt, 0);
                theta = u*max_pitch;
                xi = 0;
                q.setRPY(phi, theta, xi);
                break;
            case 2:
                phi = 0; 
                theta = 0; 
                xi = u*max_yaw;
                q.setRPY(phi, theta, xi);
                break;
            case 3:
                phi = 0;
                theta = 0; 
                xi = 0;
                q.setRPY(phi, theta, xi);
                cmd_att.body_rate.z = u*max_yaw_rate;
                cmd_att.type_mask = 3;
                break;
            case 4: 
                CONTROLLER_WARN_ONCE("Not a valid experiment choice");
                break;
            case 5:
                phi = 0;
                theta = 0;
                xi = 0;
                q.setRPY(phi, theta, xi);
                cmd_att.thrust = 0.1*u + hover_thrust;
                break;
        }

        cmd_att.orientation.x = q[0];
        cmd_att.orientation.y = q[1];
        cmd_att.orientation.z = q[2];
        cmd_att.orientation.w = q[3];
        cmd_att.header.stamp = ros::Time::now();
        local_att_pub.publish(cmd_att);
    }

    else if(control_type == "velocity"){
        CONTROLLER_INFO_ONCE("Velocity control mode");
        switch(exp_type){
            case 0: 
                pose.position.x = NAN;
                pose.position.y = NAN;
                pose.velocity.y = -u*vel_sp;
                break;
            case 1:
                pose.position.x = NAN;
                pose.position.y = NAN;
                pose.velocity.x = u*vel_sp;
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
                pose.velocity.z = u*vel_sp;
                break;
            case 5:
                CONTROLLER_INFO_ONCE("Not a valid experiment choice");
                break;
        }
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
    }

    else if(control_type == "position"){
        CONTROLLER_INFO_ONCE("Position control mode");
        switch(exp_type){
            case 0: 
                pose.position.y = u*pos_sp + exp_sp[1];
                break;
            case 1:
                pose.position.x = u*pos_sp + exp_sp[0];
                break;
            case 2:
                pose.yaw = u*max_yaw;
                break;
            case 3:
                pose.yaw_rate = u*max_yaw_rate;
                break;
            case 4: 
                pose.position.z = u*pos_sp + exp_sp[2];
                break;
            case 5:
                CONTROLLER_INFO_ONCE("Not a valid experiment choice");
                break;
            case 6:
                CONTROLLER_INFO_ONCE("Random path chosen");
                if((abs(current_pos.position.x - exp_sp[0]) + abs(current_pos.position.y - exp_sp[1]) + abs(current_pos.position.z - exp_sp[2])) < 0.25) {
                    if(waiting_){
                        tic = ros::Time::now();
                        waiting_ = false;
                        toggle_record.data = true;
                        record_data_pub.publish(toggle_record);
                    }
                    else{
                        toc = ros::Time::now();
                        duration_ = toc.toSec() - tic.toSec();
                        CONTROLLER_INFO(duration_);
                        if(duration_ > 10){
                            waiting_ = true;
                            toggle_record.data = false;
                            record_data_pub.publish(toggle_record);
                            exp_sp[0] = ( std::rand() % ( 40 + 1 ) )*1e-1 - 2.0;
                            exp_sp[1] = ( std::rand() % ( 40 + 1 ) )*1e-1 - 2.0;
                        }
                    }
                    pose.position.x = exp_sp[0];
                    pose.position.y = exp_sp[1];
                    pose.position.z = 1;
                }
                break;
        }
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
    }

    // Terminate experiment some time after takeoff 
    missionFinishedCheck();
}

void MPCIdentificationExp::missionFinishedCheck()
{
    if (ros::Time::now()-last_request > ros::Duration(exp_duration)) {
        mission_finished_ = true;
        CONTROLLER_INFO("Mission finished! Transferred back control to PX4 control interface and got the following message back: '" << mission_finished_srv_.response.message << "'");
    }

    if (mission_finished_) {
        if (mission_finished_client_.call(mission_finished_srv_)) {
            loop_timer_.stop();
            timer_running_ = false;
        } else {
            if (ros::Time::now() - last_time_ > print_timeout_) {
                CONTROLLER_ERROR("Mission finished, but failed to transfer back control to PX4 control interface! Will try again every control loop execution until success");
                last_time_ = ros::Time::now();
            }
        }
    }
}

void MPCIdentificationExp::reconfigureCallback(hovergames_mpc_identification::MPCIdentificationGazeboConfig& config, uint32_t level){
    CONTROLLER_INFO("Load configurable params");
    kp_pos = config.kp_pos;
    ki_pos = config.ki_pos;
    kp_vel = config.kp_vel;
    kd_vel = config.kd_vel;
    ki_vel = config.ki_vel;
    Q_11 = config.Q_pos;
    Q_22 = config.Q_vel;
    Q_33 = config.Q_att;
    ki_LQR = config.I_LQR;
    pid.retune(kp_pos, ki_pos, kp_vel, kd_vel, ki_vel);
    lqr.retune(Q_11, Q_22, Q_33, ki_LQR);
};

void MPCIdentificationExp::state_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (!first_state_received_) {
        first_state_received_ = true;
    }
    current_pos = msg->pose.pose;
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(current_roll, current_pitch, current_yaw);
    current_vel = msg->twist.twist;
}

void MPCIdentificationExp::acc_cb(const sensor_msgs::Imu::ConstPtr& msg){
    current_acc = *msg;
}

void MPCIdentificationExp::batt_cb(const sensor_msgs::BatteryState::ConstPtr& msg){
    battery_status = *msg;
    if(!is_sim_){
        hover_thrust = battery_thrust_ratio/battery_status.voltage;
        pid.sethoverthrust(hover_thrust);
    }
}

bool MPCIdentificationExp::enableControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    if (!first_state_received_) {
        CONTROLLER_WARN_STREAM("enableControlCallback called: control loop can only be started if first state is received!");
        res.success = false;
        res.message = "Control loop can only be started if first state is received!";
        return false;
    }
    else{
        CONTROLLER_INFO("enableControlCallback called");
        loop_timer_.start();
        last_request = ros::Time::now();
        time_pid = ros::Time::now();
        res.success = true;
        res.message = "Enabled Identification experiments";
        return true;
    }
}

bool MPCIdentificationExp::disableControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    CONTROLLER_INFO("disableControlCallback called: stopping control loop");
    loop_timer_.stop();
    res.success = true;
    res.message = "Disabled Identification experiments";
    return true;
}

void MPCIdentificationExp::SetStreamingRates(int loop_frequency_){
    msg_interv.request.message_rate = loop_frequency_;
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
};
