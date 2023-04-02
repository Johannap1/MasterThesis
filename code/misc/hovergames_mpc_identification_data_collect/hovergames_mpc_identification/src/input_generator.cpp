#include "input_generator.h"
#include "helpers.h"
#include <ros/ros.h>
#include <cstdlib>
#include <thread>
#include <chrono> 
#include <math.h> 

Input::Input():
    _d(0),
    _prbs(true),
    _u(0),
    _update_time(false),
    _t_sec(0)
{
}

void Input::initialize(int inp_type, double max_freq, double min_freq, double samp_freq, double step_time){
    _inp_type = inp_type;
    _max_freq = max_freq;
    _min_freq = min_freq;
    _samp_freq = samp_freq;
    _step_time = step_time;
}

int Input::prbs_gen(){
    int max_ticks = (int) _samp_freq/_min_freq;
    int min_ticks = (int) _samp_freq/_max_freq;
    int rand_diff = max_ticks - min_ticks + 1;
    int rand_t = (std::rand()%rand_diff)+min_ticks;
    return rand_t;
}


double Input::computeInput(ros::Time now, ros::Time t_prbs){
    switch(_inp_type){
        case 0:
        CONTROLLER_INFO_ONCE("Generating step signal");
            if((now-t_prbs)>ros::Duration(_step_time)){
                    _u = 1;
            }
            break;
        case 1:
        CONTROLLER_INFO_ONCE("Generating PRBS signal");
            if(_prbs){
                int count = prbs_gen();
                _d = (count/_samp_freq);
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
        case 2:
        CONTROLLER_INFO_ONCE("Generating random validation signal");
            if(_prbs){
                int count = prbs_gen();
                _d = (count/_samp_freq);
                _prbs = false;
                _update_time = false;
            }
            if((now-t_prbs)>ros::Duration(_d)){
                double randnumb = ((std::rand()%1000)-500.0)/500.0;
                _u = randnumb;
                _prbs = true;
                _update_time = true;
            }
            break;
        case 3:
        CONTROLLER_INFO_ONCE("Generating sinusoidal validation signal");
            _t_sec = (now-t_prbs).toSec();
            _u = sin(_max_freq*_t_sec);
            break;
        case 4:
        CONTROLLER_INFO_ONCE("Increasing input linearly");
            _t_sec = (now-t_prbs).toSec();
            _u = _t_sec*_max_freq;
            break;
        case 5:
        CONTROLLER_INFO_ONCE("Generating constant input signal");
            _u = 0;
            break;

    }
    return _u;
}

bool Input::update(){
    return _update_time;
}

Input::~Input()
{
}
