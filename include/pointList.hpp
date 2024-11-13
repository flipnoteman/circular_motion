#ifndef POINTLIST_HPP
#define POINTLIST_HPP

#include "geometry_msgs/Point.h"

struct pointNode {
    geometry_msgs::Point point;
    pointNode* next;
};

class PointList {
    pointNode *head;
    int length;
public:
    void push(geometry_msgs::Point);
    bool exists(geometry_msgs::Point);
    int len();
    void popFront();

    geometry_msgs::Point const operator [] (int); 
};


#endif