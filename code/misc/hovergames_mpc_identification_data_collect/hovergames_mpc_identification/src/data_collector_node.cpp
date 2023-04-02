#include <ros/ros.h>

#include "data_collector.h"

int main(int argc, char **argv)
{
  try {
    // Initialize ROS node
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh("data_collector");

    // Create PX4 control interface instance
    DataCollector* data_collector_ = new DataCollector(nh);

    // Initialize PX4 control interface and catch possible errors
    if (!data_collector_->initialize()) {
      ROS_ERROR_STREAM(ros::this_node::getName().c_str() << " MPC Identification experiment initialization failed!");
      exit(1);
    } else {
      ROS_INFO_STREAM(ros::this_node::getName().c_str() << " MPC Identification experiment initialization successful");
      ros::spin();
    }
  } catch (ros::Exception& e) {
    ROS_ERROR_STREAM(ros::this_node::getName().c_str() << " Error occurred: " << e.what() << "!");
    exit(1);
  }

  return 0;
}