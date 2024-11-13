#include "geometry_msgs/Point.h"
#include "simulator_utils/Waypoint.h"
#include "ros/ros.h"
#include <cmath>

#define RADIUS 3.0
#define NUM_POINTS 20
#define TOLERANCE 1.3

bool position_received = false;
int i = 0;

void getCenter();
void calculateCirclePosition(int, geometry_msgs::Point*);
bool hasReachedTarget(geometry_msgs::Point);
void currentStateCallback(const simulator_utils::Waypoint::ConstPtr&); 

ros::Subscriber sub;   // Subscriber to get current position of drone
ros::Publisher pub;// Publisher to tell the drone to move somewhere
geometry_msgs::Point *center = new geometry_msgs::Point();
geometry_msgs::Point currentState; 
geometry_msgs::Point desiredState; 


int main(int argc, char **argv) {

  getCenter();
  ros::init(argc, argv, "circular_mover"); // Initializes the ros node with arguments from ROS
  ros::NodeHandle n;  // Main access point to communications with the ROS system
  sub = n.subscribe("/robot_1/current_state", 10, currentStateCallback);
  pub = n.advertise<geometry_msgs::Point>("/robot_1/desired_state", 10);  
  ros::Rate loop_rate(1); //10 Hz


  // geometry_msgs::Point last_point;
  calculateCirclePosition(i, &desiredState);
  pub.publish(desiredState);

  for (;;) { 
    if (!position_received) {
      pub.publish(desiredState);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void currentStateCallback(const simulator_utils::Waypoint::ConstPtr& point) 
{
  // ROS_INFO("Current Location: [%f, %f, %f]", point->position.x, point->position.y, point->position.z);
  currentState.x = point->position.x;
  currentState.y = point->position.y;
  currentState.z = point->position.z;

  if (hasReachedTarget(desiredState)) {
    ROS_INFO("Reached target location: [%f, %f, %f]", desiredState.x, desiredState.y, desiredState.z);
    ++i;
    if (i > NUM_POINTS) i = 0;
    calculateCirclePosition(i, &desiredState);
    pub.publish(desiredState); 
    ROS_INFO("Sent target location: [%f, %f, %f]", desiredState.x, desiredState.y, desiredState.z); 
  }

  position_received = true;
  ROS_INFO("Current Position: [%f, %f, %f]", currentState.x, currentState.y, currentState.z);
}

void getCenter() {
  center->x = 0.0;
  center->y = 0.0;
  center->z = 1.0;
}

void calculateCirclePosition(int iteration, geometry_msgs::Point *point) 
{
  double theta = 2.0 * M_PI * iteration / NUM_POINTS;
  point->x = center->x + RADIUS * cos(theta);
  point->y = center->y + RADIUS * sin(theta);
  point->z = center->z;
}

bool hasReachedTarget(geometry_msgs::Point target) { 
  double distance = std::sqrt(
    std::pow(currentState.x - target.x, 2) + 
    std::pow(currentState.y - target.y, 2) + 
    std::pow(currentState.z - target.z, 2)); 
  ROS_INFO("Distance to target: %f", distance);
  
  return distance < TOLERANCE;
}

bool pointEquals(const geometry_msgs::Point lhs, const geometry_msgs::Point rhs) { 
  double distance = std::sqrt(
    std::pow(lhs.x - rhs.x, 2) + 
    std::pow(lhs.y - rhs.y, 2) + 
    std::pow(lhs.z - rhs.z, 2)); 
  ROS_INFO("Distance to target: %f", distance);
  
  return distance < TOLERANCE;
}
