#include <ros/ros.h>

#ifndef _INPUT_GENERATOR_H_
#define _INPUT_GENERATOR_H_

class INPUT
{
    public:
        INPUT(int inp_type, double max_freq, double min_freq, double samp_freq, double step_time);
        ~INPUT();
        double input(ros::Time now, ros::Time t_prbs);
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

        int prbs_gen();
};

#endif