#include "fcs_interface/fcs_interface.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "fcs_interface");
  ros::NodeHandle nh;
  FCS_Interface interface {nh};
  interface.start();
  ros::spin();
}