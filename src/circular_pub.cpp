#include "geometry_msgs/Point.h"
#include "simulator_utils/Waypoint.h"
#include "ros/ros.h"
#include <cmath>

const double radius = 5.0;
const int numPoints = 50;
bool position_received = false;

geometry_msgs::Point *center = new geometry_msgs::Point();
geometry_msgs::Point currentState; 

void getCenter() {
  center->x = 0.0;
  center->y = 0.0;
  center->z = 1.0;
}

void currentStateCallback(const simulator_utils::Waypoint::ConstPtr& point) 
{
  // ROS_INFO("Current Location: [%f, %f, %f]", point->position.x, point->position.y, point->position.z);
  currentState.x = point->position.x;
  currentState.y = point->position.y;
  currentState.z = point->position.z;
  position_received = true;
  ROS_INFO("Current Position: [%f, %f, %f]", currentState.x, currentState.y, currentState.z);
}

void calculateCirclePosition(int iteration, geometry_msgs::Point *point) 
{
  double theta = 2.0 * M_PI * iteration / numPoints;
  point->x = center->x + radius * cos(theta);
  point->y = center->y + radius * sin(theta);
  point->z = center->z;
}

bool hasReachedTarget(const geometry_msgs::Point& target, double tolerance) { 
  double distance = std::sqrt(
    std::pow(currentState.x - target.x, 2) + 
    std::pow(currentState.y - target.y, 2) + 
    std::pow(currentState.z - target.z, 2)); 
  ROS_INFO("Distance to target: %f", distance);
  
  return distance < tolerance;
}

bool pointEquals(const geometry_msgs::Point lhs, const geometry_msgs::Point rhs, double tolerance) { 
  double distance = std::sqrt(
    std::pow(lhs.x - rhs.x, 2) + 
    std::pow(lhs.y - rhs.y, 2) + 
    std::pow(lhs.z - rhs.z, 2)); 
  ROS_INFO("Distance to target: %f", distance);
  
  return distance < tolerance;
}

int main(int argc, char **argv) {
  ROS_INFO("Starting");
  getCenter();

  // Initializes the ros node with arguments from ROS
  ros::init(argc, argv, "circular_mover");

  // Main access point to communications with the ROS system
  ros::NodeHandle n;

  // Subscriber to get current position of drone
  ros::Subscriber sub = n.subscribe("/robot_1/current_state", 10, currentStateCallback);

  // Publisher to tell the drone to move somewhere
  ros::Publisher pub =
      n.advertise<geometry_msgs::Point>("/robot_1/desired_state", 1);

  ros::Rate loop_rate(2); //10 Hz

  int i = 0;
  geometry_msgs::Point point; // Will hold our point value for calculation
  geometry_msgs::Point last_point;

  double tolerance = 1.3;
  calculateCirclePosition(i, &point);
  // pub.publish(point); 
  for (;;) {  
    last_point.x = point.x;
    last_point.y = point.y;
    last_point.z = point.z;

    if (hasReachedTarget(point, tolerance)) {
      ROS_INFO("Reached target location: [%f, %f, %f]", point.x, point.y, point.z);
      ++i;
      calculateCirclePosition(i, &point);
      ROS_INFO("Sent target location: [%f, %f, %f]", point.x, point.y, point.z); 
    }    

    // if (!pointEquals(last_point, point, tolerance)) {
      pub.publish(point); 
    // }

    ros::spinOnce();
    loop_rate.sleep();
    if (i > numPoints) i = 0;
  }
  // do {
  //   pub.publish(*center);
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // } while (!pointEquals(*currentState, *center));

  // calculateCirclePosition(i, &point);
  // pub.publish(point);

  // while (ros::ok()) {
  //   calculateCirclePosition(i, &point);
  //   if (pointEquals(*currentState, point)) {
  //     ROS_INFO("Next Point: { x: %f, y: %f, z: %f}", point.x, point.y, point.z);
  //     ++i;
  //   }
  //   pub.publish(point);

  //   ros::spinOnce();
  //   loop_rate.sleep();
    
  //   if (i > numPoints) i = 0;
  // }

  return 0;
}
