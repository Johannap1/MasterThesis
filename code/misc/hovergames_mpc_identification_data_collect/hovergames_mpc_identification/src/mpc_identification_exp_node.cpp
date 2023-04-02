#include <ros/ros.h>

#include "mpc_identification_exp.h"

int main(int argc, char **argv)
{
  try {
    // Initialize ROS node
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh("mpc_identification_exp");

    // Create PX4 control interface instance
    MPCIdentificationExp* mpc_identification_exp_ = new MPCIdentificationExp(nh);

    // Create dynamic reconfiguration server
    dynamic_reconfigure::Server<hovergames_mpc_identification::MPCIdentificationGazeboConfig> server(nh);
    dynamic_reconfigure::Server<hovergames_mpc_identification::MPCIdentificationGazeboConfig>::CallbackType f;
    f = boost::bind(&MPCIdentificationExp::reconfigureCallback, mpc_identification_exp_, _1, _2);
    server.setCallback(f);

    // Initialize PX4 control interface and catch possible errors
    if (!mpc_identification_exp_->initialize()) {
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
