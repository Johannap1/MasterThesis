#include "lqr_control.h"


LQR_Control::LQR_Control():
    _att_int_roll(0),
    _att_int_pitch(0),
    _g(9.81)
{
}

void LQR_Control::initialize(double Q_11, double Q_22, double Q_33, double ki_LQR, bool is_sim, double loop_frequency)
{
    Q(0,0) = Q_11;
    Q(1,1) = Q_22;
    Q(2,2) = Q_33;
    R(0,0) = 1;
    _ki_LQR = ki_LQR;
    _samp_freq = 1/loop_frequency;

    if(is_sim){
        _k_phi = 1.04;
        _k_theta = 1.04;
        _tau_phi = 0.17;
        _tau_theta = 0.17;
    }
    else{
        _k_phi = 0.9879;
        _k_theta = 1.1101;
        _tau_phi = 0.0531;
        _tau_theta = 0.0882;
    }

    Ac_pitch(0,1) = 1;
    Ac_pitch(1,2) = _g;
    Ac_pitch(2,2) = -1/_tau_theta;
    Bc_pitch(2,0) = _k_theta/_tau_theta;
    Ac_roll(0,1) = 1;
    Ac_roll(1,2) = -_g;
    Ac_roll(2,2) = -1/_tau_phi;
    Bc_roll(2,0) = _k_phi/_tau_phi;

    Ad_pitch = (_samp_freq * Ac_pitch).exp();
    Bd_pitch = c2d(Ac_pitch, Bc_pitch);
    Ad_roll = (_samp_freq * Ac_roll).exp();
    Bd_roll = c2d(Ac_roll, Bc_roll);

    K_roll = K_lqr(Ad_roll, Bd_roll);
    K_pitch = K_lqr(Ad_pitch, Bd_pitch);
}

void LQR_Control::retune(double Q_11, double Q_22, double Q_33, double ki_LQR)
{
    Q(0,0) = Q_11;
    Q(1,1) = Q_22;
    Q(2,2) = Q_33;
    _ki_LQR = ki_LQR;

    K_roll = K_lqr(Ad_roll, Bd_roll);
    K_pitch = K_lqr(Ad_pitch, Bd_pitch);
}

double LQR_Control::calculate(double setpoint_pos, double current_pos, double current_vel, double current_att, double dt, int exp_type){
    double att_d;
    if(exp_type == 0){
        att_d = -K_roll(0,0)*(current_pos - setpoint_pos)-K_roll(0,1)*current_vel-K_roll(0,2)*current_att + _att_int_roll;
        _att_int_roll += -_ki_LQR*(current_pos - setpoint_pos)*dt;
    }
    if(exp_type == 1){
        att_d = -K_pitch(0,0)*(current_pos - setpoint_pos)-K_pitch(0,1)*current_vel-K_pitch(0,2)*current_att + _att_int_pitch;
        _att_int_pitch += -_ki_LQR*(current_pos - setpoint_pos)*dt;
    }
    return att_d;
}

Eigen::MatrixXd LQR_Control::c2d(Eigen::MatrixXd Ac){
    Eigen::MatrixXd Ad = (_samp_freq * Ac).exp();
    return Ad;
}

Eigen::Matrix<double, 3, 1> LQR_Control::c2d(Eigen::Matrix<double, 3, 3> Ac, Eigen::Matrix<double, 3, 1> Bc){
    Eigen::MatrixXd integral_exp_A;
    integral_exp_A = Eigen::MatrixXd::Zero(3,3);
    const int count_integral_A = 100;
    for (int i = 0; i < count_integral_A; i++) {
        integral_exp_A += (Ac * _samp_freq * i / count_integral_A).exp()
            * _samp_freq / count_integral_A;
    }
    Eigen::MatrixXd Bd = integral_exp_A * Bc;
    return Bd;
}

Eigen::Matrix<double, 1, 3> LQR_Control::K_lqr(Eigen::Matrix<double, 3, 3> Ad, Eigen::Matrix<double, 3, 1> Bd){
    Eigen::Matrix<double, 3, 3> Q_final = Q;
    for (int i = 0; i < 1000; i++) {
        Eigen::MatrixXd temp = (Bd.transpose() * Q_final * Bd + R);
        Q_final = Ad.transpose() * Q_final * Ad
            - (Ad.transpose() * Q_final * Bd) * temp.inverse()
                * (Bd.transpose() * Q_final * Ad) + Q;
    }
    Eigen::MatrixXd temporary_matrix = Bd.transpose() * Q_final * Bd + R;
    Eigen::Matrix<double, 1, 3> K = temporary_matrix.inverse() * (Bd.transpose() * Q_final * Ad);
    return K;
}
    
LQR_Control::~LQR_Control()
{
}

    