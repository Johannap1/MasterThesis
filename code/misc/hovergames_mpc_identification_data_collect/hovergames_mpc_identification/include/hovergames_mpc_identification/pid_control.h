#ifndef _PID_Control_H_
#define _PID_Control_H_

#include <ros/ros.h>

class PID_Control
{
    public:
        PID_Control();
        ~PID_Control();
        void initialize(double max_vel, double min_vel, double max_thrust, double min_thrust, double hover_thrust, double Kp_pos, double Ki_pos, double Kp_vel, double Kd_vel, double Ki_vel);
        void retune(double Kp_pos, double Ki_pos, double Kp_vel, double Kd_vel, double Ki_vel);
        void sethoverthrust(double hover_thrust);
        double calculate(double setpoint_pos, double current_pos, double current_vel, double current_acc, double dt);

    private:
    double _max_vel;
    double _min_vel;
    double _max_thrust;
    double _min_thrust;
    double _hover_thrust;

    double _Kp_pos;
    double _Ki_pos;
    double _Kp_vel;
    double _Kd_vel;
    double _Ki_vel;

    double _vel_int;
    double _pos_int;
    double _g;
};

#endif