#ifndef _PID_H_
#define _PID_H_

class PID
{
    public:
        PID(double max_vel, double min_vel, double max_thrust, double min_thrust, double hover_thrust, double Kp_pos, double Kp_vel, double Kd_vel, double Ki_vel);
        ~PID();
        double calculate(double setpoint_pos, double current_pos, double current_vel, double current_acc, double dt);

    private:
    double _max_vel;
    double _min_vel;
    double _max_thrust;
    double _min_thrust;
    double _hover_thrust;

    double _Kp_pos;
    double _Kp_vel;
    double _Kd_vel;
    double _Ki_vel;

    double _vel_int;
    double _g;
};

#endif