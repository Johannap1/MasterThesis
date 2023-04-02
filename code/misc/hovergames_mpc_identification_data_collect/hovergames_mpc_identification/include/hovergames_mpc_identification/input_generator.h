#include <ros/ros.h>

#ifndef _Input_GENERATOR_H_
#define _Input_GENERATOR_H_

class Input
{
    public:
        Input();
        ~Input();
        void initialize(int inp_type, double max_freq, double min_freq, double samp_freq, double step_time);
        double computeInput(ros::Time now, ros::Time t_prbs);
        bool update();

    private:
        double _max_freq;
        double _min_freq;
        double _samp_freq;
        int _inp_type;
        double _step_time;

        double _d;
        double _prbs;
        double _u;
        bool _update_time;
        bool _start;
        double _t_sec;

        int prbs_gen();
};

#endif