#ifndef _LQR_Control_H_
#define _LQR_Control_H_

#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>
#include <ros/ros.h>

class LQR_Control
{
    public:
        LQR_Control();
        ~LQR_Control();
        void initialize(double Q_11, double Q_22, double Q_33, double ki_LQR, bool is_sim, double loop_frequency);
        void retune(double Q_11, double Q_22, double Q_33, double ki_LQR);
        double calculate(double setpoint_pos, double current_pos, double current_vel, double current_att, double dt, int type);
        Eigen::Matrix<double, 1, 3> K_lqr(Eigen::Matrix<double, 3, 3> Ad, Eigen::Matrix<double, 3, 1> Bd);
        Eigen::MatrixXd c2d(Eigen::MatrixXd Ac);
        Eigen::Matrix<double, 3, 1> c2d(Eigen::Matrix<double, 3, 3> Ac, Eigen::Matrix<double, 3, 1> Bc);

    private:
    double _ki_LQR;

    Eigen::Matrix<double, 1, 3> K_roll;
    Eigen::Matrix<double, 1, 3> K_pitch;
    Eigen::Matrix<double, 3, 3> Q;
    Eigen::Matrix<double, 1, 1> R;

    Eigen::Matrix<double, 3, 3> Ac_roll;
    Eigen::Matrix<double, 3, 1> Bc_roll;

    Eigen::Matrix<double, 3, 3> Ac_pitch;
    Eigen::Matrix<double, 3, 1> Bc_pitch;

    Eigen::Matrix<double, 3, 3> Ad_roll;
    Eigen::Matrix<double, 3, 1> Bd_roll;

    Eigen::Matrix<double, 3, 3> Ad_pitch;
    Eigen::Matrix<double, 3, 1> Bd_pitch;

    double _k_phi;
    double _k_theta;
    double _tau_phi;
    double _tau_theta;
    double _samp_freq;

    double _att_int_roll;
    double _att_int_pitch;
    double _g;
};

#endif