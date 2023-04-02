#ifndef MPC_IDENTIFICATION_EXP
#define MPC_IDENTIFICATION_EXP

//general includes

//custom includes
#include "pid_control.h"
#include "lqr_control.h"
#include "input_generator.h"
#include "helpers.h"

//ROS includes
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <hovergames_mpc_identification/MPCIdentificationGazeboConfig.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>

using namespace std;

class MPCIdentificationExp
{
    public:
        // Constructor and destructor
        MPCIdentificationExp(const ros::NodeHandle &nh): nh_(nh) {};
        ~MPCIdentificationExp() {};

        // Class initialization
        bool initialize();

        // Dynamic reconfiguration
        void reconfigureCallback(hovergames_mpc_identification::MPCIdentificationGazeboConfig& config, uint32_t level);

    private:
        /* Function declarations */
        // Init functions
        bool getRosParameters();
        bool initConnection();

        // Loop functions 
        void loop(const ros::TimerEvent &event);
        void missionFinishedCheck();
        // Callback functions
        void state_cb(const nav_msgs::Odometry::ConstPtr& msg);
        void acc_cb(const sensor_msgs::Imu::ConstPtr& msg);
        void batt_cb(const sensor_msgs::BatteryState::ConstPtr& msg);
        bool enableControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool disableControlCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        // Set streaming rates
        void SetStreamingRates(int loop_frequency_);

        /* Retrieve paramater, if it doesn't exist return false */
        template <typename T>
        bool retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value)
        {
            if (!nh.getParam(name, value)) {
                ROS_WARN_STREAM(ros::this_node::getName().c_str() << " Parameter " << name << " not set.");
                return false;
            } else {
                return true;
            }
        }

        /* Retrieve parameter, if it doesn't exist use the default */
        template <typename T>
        void retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value, const T &default_value)
        {
            if (!retrieveParameter(nh, name, value)) {
                ROS_WARN_STREAM(ros::this_node::getName().c_str() << " Setting " << name << " to default value: " << default_value << ".");
                value = default_value;
            }
        }

        // ROS initializations
        ros::NodeHandle nh_;
        ros::Subscriber state_sub;
        ros::Subscriber local_acc_sub;
        ros::Subscriber battery_sub;
        ros::Publisher local_pos_pub;
        ros::Publisher local_att_pub;
        ros::Publisher record_data_pub;
        ros::ServiceClient rate_client;
        ros::ServiceServer enable_control_sever_;
        ros::ServiceServer disable_control_server_;
        ros::ServiceClient mission_finished_client_;

        // Received data declarations
        mavros_msgs::State current_state;
        geometry_msgs::Pose current_pos;
        geometry_msgs::Twist current_vel;
        sensor_msgs::Imu current_acc;
        sensor_msgs::BatteryState battery_status;
        std_msgs::Bool toggle_record;
        double current_roll, current_pitch, current_yaw;
        bool first_state_received_;

        // ROS dynamic reconfiguration
        bool first_reconfig_cb_;
        bool enable_debug_;

        // Service declarations
        mavros_msgs::MessageInterval msg_interv;
        std_srvs::Trigger mission_finished_srv_;

        // Message declarations
        mavros_msgs::PositionTarget pose;
        mavros_msgs::AttitudeTarget cmd_att;

        // Experiment settings
        int inp_type, exp_type;
        string control_type;
        vector<double> exp_sp {vector<double>(3,0)};
        double pos_sp, vel_sp;
        double max_freq, min_freq;
        double exp_duration;

        // Attitude commands
        tf2::Quaternion q;
        double phi;
        double theta;
        double xi;

        // Streaming rates message_id
        unsigned int message_id[5] = {30,31,32,83,85};

        // Controller parameters
        double kp_pos, ki_pos, kp_vel, kd_vel, ki_vel;
        double Q_11, Q_22, Q_33, ki_LQR;
        double hover_thrust;
        const double battery_thrust_ratio = 8.6576;
        double dt; 
        double cmd_thrust;
        double max_vel = 1;
        double max_thrust = 0.9;
        double min_thrust = 0.1;
        PID_Control pid;
        LQR_Control lqr;

        // Input generator parameters
        double samp_freq, step_time;
        bool update_time;
        bool streamed;
        double u;
        Input input;

        // Experiment generator
        const double max_roll_default = 12; //deg
        const double max_pitch_default = 12; //deg
        double max_roll, max_pitch;
        const double max_yaw = 120*3.1415926535897932/180; //rad
        const double max_yaw_rate = 20*3.1415926535897932/180; //rad/s
        const double max_vz = 1; //m/s
        const double att_to_vel = 10;
        double max_pos_x, max_pos_y, max_pos_z;
        double max_vel_x, max_vel_y;
        double random_pos_x, random_pos_y; 
        double duration_;
        bool waiting_;


        // Indicate experiment or simulation
        bool is_sim_, is_sim_default_ = false;

        // Time declarations
        bool experiment_start;
        double loop_frequency_;
        const double loop_frequency_default_ = 50;
        ros::Timer loop_timer_;
        ros::Time last_request, last_time_;
        ros::Time tic, toc;
        ros::Time time_pid;
        ros::Time time_inp;
        bool timer_running_;
        ros::Duration print_timeout_;
        
        // Mission-related
        bool mission_finished_;
};

#endif //MPC_IDENTIFICATION_EXP
