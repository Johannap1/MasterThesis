#include "pid_control.h"

using namespace std;

PID_Control::PID_Control():
    _vel_int(0),
    _pos_int(0),
    _g(9.81)
{
}

void PID_Control::initialize(double max_vel, double min_vel, double max_thrust, double min_thrust, double hover_thrust, double Kp_pos, double Ki_pos, double Kp_vel, double Kd_vel, double Ki_vel){
    _max_vel = max_vel;
    _min_vel = min_vel;
    _max_thrust = max_thrust;
    _min_thrust = min_thrust;
    _hover_thrust = hover_thrust;
    _Kp_pos = Kp_pos;
    _Ki_pos = Ki_pos;
    _Kp_vel = Kp_vel;
    _Ki_vel = Ki_vel;
    _Kd_vel = Kd_vel;
}

double PID_Control::calculate(double setpoint_pos, double current_pos, double current_vel, double current_acc, double dt)
{
    // Position controller
    double pos_error = setpoint_pos - current_pos;
    _pos_int += _Ki_pos*pos_error*dt;
    double vel_sp_z = pos_error*_Kp_pos + _pos_int;

    // Limit velocity setpoint
    if(vel_sp_z > _max_vel){
        vel_sp_z = _max_vel;
    }
    if(vel_sp_z < _min_vel){
        vel_sp_z = _min_vel;
    }

    // Velocity_controller    
    double vel_error = vel_sp_z - current_vel;
    double acc_sp_z = vel_error*_Kp_vel + _vel_int + (current_acc-_g)*_Kd_vel;
    _vel_int += _Ki_vel*vel_error*dt;

    // Calculate thrust setpoint and limit commanded thrust
    double cmd_thrust = acc_sp_z * (_hover_thrust/_g) + _hover_thrust;
    if (cmd_thrust > _max_thrust){
        cmd_thrust = _max_thrust;
    }
    if (cmd_thrust < _min_thrust){
        cmd_thrust = _min_thrust;
    }
    return cmd_thrust;
}

void PID_Control::retune(double Kp_pos, double Ki_pos, double Kp_vel, double Kd_vel, double Ki_vel){
    _Kp_pos = Kp_pos;
    _Ki_pos = Ki_pos;
    _Kp_vel = Kp_vel;
    _Kd_vel = Kd_vel;
    _Ki_vel = Ki_vel;
}

void PID_Control::sethoverthrust(double hover_thrust){
    _hover_thrust = hover_thrust;
}

PID_Control::~PID_Control()
{
}

