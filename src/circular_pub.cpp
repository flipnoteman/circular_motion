#include "geometry_msgs/Point.h"
#include "ros/ros.h"

int main(int argc, char **argv) {

  // Initializes the ros node with arguments from ROS
  ros::init(argc, argv, "circular_mover");

  // Main access point to communications with the ROS system
  ros::NodeHandle n;

  ros::Publisher pub =
      n.advertise<geometry_msgs::Point>("/robot_1/desired_state", 10);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    geometry_msgs::Point point;
    point.x = 1.0;
    point.y = 2.0;
    point.z = 3.0;

    ROS_INFO("Next Point: { x: %d, y: %d, z: %d}", point.x, point.y, point.z);

    pub.publish(point);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
