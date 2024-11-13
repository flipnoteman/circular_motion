#include "pointList.hpp"

geometry_msgs::Point const PointList::operator[] (int i) {
    pointNode *temp = head;

    for (int j = 0; j <= i; ++j) {
        temp = temp->next;
    }

    return temp->point;
}

void PointList::push(geometry_msgs::Point newPoint) {
    pointNode* newNode = new pointNode();
    newNode->point.x = newPoint.x;
    newNode->point.y = newPoint.y;
    newNode->point.z = newPoint.z;
    newNode->next = nullptr;

    if (!head) {
        head = newNode;
    } else {
        pointNode* temp = head;

        while (temp->next) {
            temp = temp->next;
        }

        temp->next = newNode;
    }

    length++;
}

bool PointList::exists(geometry_msgs::Point point) {
    pointNode* temp = head;
    if (!head)
    {
        do {
            if (abs(temp->point.x - point.x) < 0.01
            && abs(temp->point.y - point.y) < 0.01
            && abs(temp->point.z - point.z) < 0.01)
            {
                return true;
            }

            temp = temp->next;
        } while (temp->next);
    }

    return false;
}

int PointList::len() {
    return length;
}

void PointList::popFront() {

    if (head) {
        pointNode* temp = head;
        head = temp->next;
        delete(temp);
    }

    length--;
}