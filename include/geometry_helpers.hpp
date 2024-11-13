#ifndef GEOMETRY_HELPERS_HPP
#define GEOMETRY_HELPERS_HPP

#include "pointList.hpp"
#include "simulator_utils/Waypoint.h"
#include "ros/ros.h"
#include <cmath>

#define RADIUS 3.0
#define NUM_POINTS 20
#define TOLERANCE 1.3

extern geometry_msgs::Point currentState; 
extern geometry_msgs::Point lastState;
extern geometry_msgs::Point desiredState; 
extern geometry_msgs::Point center;
extern PointList desiredStateList;
extern int posIteration;

void getCenter();
void calculateCirclePosition(int, geometry_msgs::Point*);
void geoCopy(geometry_msgs::Point*, geometry_msgs::Point);
bool hasReachedTarget();
bool isHalfwayToTarget();
double distanceFrom(geometry_msgs::Point, geometry_msgs::Point);

#endif