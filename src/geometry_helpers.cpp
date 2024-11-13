#include "geometry_helpers.hpp"

geometry_msgs::Point currentState; 
geometry_msgs::Point lastState;
geometry_msgs::Point desiredState; 
geometry_msgs::Point center;
PointList desiredStateList;
int posIteration = 0;

void getCenter() {
  center.x = 0.0;
  center.y = 0.0;
  center.z = 1.0;
}

void geoCopy(geometry_msgs::Point* destination, geometry_msgs::Point source) {
  destination->x = source.x;
  destination->y = source.y;
  destination->z = source.z;
}

void calculateCirclePosition(int iteration, geometry_msgs::Point *point) 
{
  double theta = 2.0 * M_PI * iteration / NUM_POINTS;
  point->x = center.x + RADIUS * cos(theta);
  point->y = center.y + RADIUS * sin(theta);
  point->z = center.z;
}

bool hasReachedTarget() { 
  double distance = distanceFrom(currentState, desiredState);
  // ROS_INFO("Distance to target: %f", distance);
  return distance < TOLERANCE;
}

bool isHalfwayToTarget() {
  return distanceFrom(currentState, desiredState) <= (0.5 * distanceFrom(lastState, desiredState));
}

double distanceFrom(geometry_msgs::Point source, geometry_msgs::Point destination) {
  double x = pow((destination.x - source.x), 2.0);
  double y = pow((destination.y - source.y), 2.0);
  double z = pow((destination.z - source.z), 2.0);
  return sqrt(x + y + z);
}