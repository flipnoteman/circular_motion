#include "ros/ros.h"
#include "circular_motion/CalculateCirclePosition.h"
#include "geometry_msgs/Point.h"
#include "geometry_helpers.hpp"
#include <cmath>

void calculateCirclePosition(int iteration, geometry_msgs::Point *point) 
{
  double theta = 2.0 * M_PI * iteration / NUM_POINTS;
  point->x = center.x + RADIUS * cos(theta);
  point->y = center.y + RADIUS * sin(theta);
  point->z = center.z;
}

bool calculate(circular_motion::CalculateCirclePosition::Request &req,
               circular_motion::CalculateCirclePosition::Response &res)
{
  calculateCirclePosition(req.iteration, &res.point);
  return true;
}

int main(int argc, char **argv)
{
  double height = 1.0;
  if (argc > 1) {
    height = atof(argv[1]);
  }

  ros::init(argc, argv, "circle_position_server");
  ros::NodeHandle nh;

  center.x = 0.0;
  center.y = 0.0;
  center.z = height;

  ros::ServiceServer service = nh.advertiseService("calculate_circle_position", calculate);
  ROS_INFO("Ready to calculate circle positions.");
  ros::spin();

  return 0;
}
