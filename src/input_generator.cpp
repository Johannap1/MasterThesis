#include "input_generator.h"
#include <ros/ros.h>
#include <cstdlib>
#include <thread>
#include <chrono> 

INPUT::INPUT(int inp_type, double max_freq, double min_freq, double samp_freq, double step_time):
    _inp_type(inp_type),
    _max_freq(max_freq),
    _min_freq(min_freq),
    _samp_freq(samp_freq),
    _step_time(step_time),

    _d(0),
    _prbs(true),
    _u(0),
    _update_time(false)
{
}

int INPUT::prbs_gen(){
    int max_ticks = (int) _samp_freq/_min_freq;
    int min_ticks = (int) _samp_freq/_max_freq;
    int rand_diff = max_ticks - min_ticks + 1;
    int rand_t = (std::rand()%rand_diff)+min_ticks;
    return rand_t;
}

double INPUT::input(ros::Time now, ros::Time t_prbs){
    switch(_inp_type){
        case 0:
        ROS_INFO_ONCE("Generating step signal");
            if((now-t_prbs)>ros::Duration(_step_time)){
                _u = 1;
            }
            break;
        case 1:
        ROS_INFO_ONCE("Generating PRBS signal");
            if(_prbs){
                int count = prbs_gen();
                _d = (count/_samp_freq);
                std::cout<<count<<","<<_d<<std::endl;
                _prbs = false;
                _update_time = false;
            }
            if(_u  >= 1 && (now-t_prbs)>ros::Duration(_d)){
                _u = -1;
                _prbs = true;
                _update_time = true;
                
            }
            else if(_u < 1 && (now-t_prbs)>ros::Duration(_d)){
                _u = 1;
                _prbs = true;
                _update_time = true;
            }
            break;
    }
    return _u;
}

bool INPUT::update(){
    return _update_time;
}

INPUT::~INPUT()
{
}
