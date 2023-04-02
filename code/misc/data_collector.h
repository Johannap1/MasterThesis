#ifndef DATA_COLLECTOR
#define DATA_COLLECTOR


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/AccelStamped.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <vector>

class DataCollector{

    public:
        // Constructor and destructor
        DataCollector(const ros::NodeHandle &nh): nh_(nh) {};
        ~DataCollector();

        // Class initialization
        bool initialize();

    private:
        // ROS initializations
        ros::NodeHandle nh_;
        ros::Subscriber state_sub;
        ros::Subscriber input_sub;
        ros::Subscriber save_trigger;
        ros::Subscriber data_toggle_sub;
        ros::Publisher prediction_pub;
        ros::Publisher wind_pub;

        nav_msgs::Odometry state_data;
        nav_msgs::Odometry state_prediction_msg;
        mavros_msgs::AttitudeTarget input_data;
        geometry_msgs::Pose current_pos;
        geometry_msgs::Twist current_vel;
        geometry_msgs::AccelStamped wind_msg;
        double current_roll, current_pitch, current_yaw;
        double input_roll, input_pitch, input_yaw;
        std::vector<double> current_state = {0,0,0,0,0,0,0,0,0};
        std::vector<double> current_input = {0,0,0,0};
        std::vector<double> state_prediction;

        std::vector<double> state_vel;
        std::vector<double> state_att;
        std::vector<double> input_cmd;
        std::vector<double> state_pred;
        std::vector<double> timing1;
        std::vector<double> timing2;

        bool toggle_;

        double now1;
        double now2;

        double tau_phi = 0.065;
        double k_phi = 0.954;
        double tau_theta = 0.074;
        double k_theta = 0.950;
        double g = 9.81;
        double dt = 0.05;

        double wind_x;
        double wind_y;

        void state_cb(const nav_msgs::Odometry::ConstPtr& msg);
        void input_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
        void save_cb(const std_msgs::Bool::ConstPtr& msg);
        void toggle_cb(const std_msgs::Bool::ConstPtr& msg);

        std::vector<double> multiplyvector(std::vector<double> v, double k);

        std::vector<double> addvector(std::vector<double> v1, std::vector<double> v2);

        std::vector<double> solve_rk4(std::vector<double> state_cur, std::vector<double> input, double dt);

        std::vector<double> continuous_model(double dt, std::vector<double> x, std::vector<double> u);
};

#endif