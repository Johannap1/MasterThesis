#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;

PID::PID(double max_vel, double min_vel, double max_thrust, double min_thrust, double hover_thrust, double Kp_pos, double Kp_vel, double Kd_vel, double Ki_vel):
    _max_vel(max_vel),
    _min_vel(min_vel),
    _max_thrust(max_thrust),
    _min_thrust(min_thrust),
    _hover_thrust(hover_thrust),

    _Kp_pos(Kp_pos),
    _Kp_vel(Kp_vel),
    _Kd_vel(Kd_vel),
    _Ki_vel(Ki_vel),
    _vel_int(0),
    _g(9.81)
{
}

double PID::calculate(double setpoint_pos, double current_pos, double current_vel, double current_acc, double dt)
{
    // position controller
    double pos_error = setpoint_pos - current_pos;
    double vel_sp_z = pos_error*_Kp_pos;

    //limit velocity setpoint
    if(vel_sp_z > _max_vel){
        vel_sp_z = _max_vel;
    }
    if(vel_sp_z < _min_vel){
        vel_sp_z = _min_vel;
    }

    //velocity_controller    
    double vel_error = vel_sp_z - current_vel;
    double acc_sp_z = vel_error*_Kp_vel + _vel_int - (current_acc-_g)*_Kd_vel;
    _vel_int += pos_error*_Ki_vel*dt;

    //calculate thrust setpoint and limit commanded thrust
    double cmd_thrust = acc_sp_z * (_hover_thrust/_g) + _hover_thrust; //_acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
    if (cmd_thrust > _max_thrust){
        cmd_thrust = _max_thrust;
    }
    if (cmd_thrust < _min_thrust){
        cmd_thrust = _min_thrust;
    }
    return cmd_thrust;
}

PID::~PID()
{
}
