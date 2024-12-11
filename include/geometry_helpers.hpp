#ifndef GEOMETRY_HELPERS_HPP
#define GEOMETRY_HELPERS_HPP

#include "ros/ros.h"
#include "simulator_utils/Waypoint.h"
#include <cmath>

#define RADIUS 3.0
#define NUM_POINTS 20
#define TOLERANCE 1.4
#define START_TOLERANCE 0.2

extern geometry_msgs::Point currentState;
extern geometry_msgs::Point lastState;
extern geometry_msgs::Point desiredState;
extern geometry_msgs::Point center;
extern int posIteration;

void getCenter();
// void calculateCirclePosition(int, geometry_msgs::Point*);
void geoCopy(geometry_msgs::Point *, geometry_msgs::Point);
bool hasReachedTarget(double);
bool isHalfwayToTarget();
double distanceFrom(geometry_msgs::Point, geometry_msgs::Point);

#endif
