#include "geometry_helpers.hpp"

bool positionReceived = false;

ros::Subscriber sub;   // Subscriber to get current position of drone
ros::Publisher pub;// Publisher to tell the drone to move somewhere

void currentStateCallback(const simulator_utils::Waypoint::ConstPtr&);
void checkDesiredStateListLength();

int main(int argc, char **argv) {

  getCenter();
  ros::init(argc, argv, "circular_mover"); // Initializes the ros node with arguments from ROS
  ros::NodeHandle n;  // Main access point to communications with the ROS system
  sub = n.subscribe("/robot_1/current_state", 10, currentStateCallback);
  pub = n.advertise<geometry_msgs::Point>("/robot_1/desired_state", 10);  
  ros::Rate loop_rate(1); //10 Hz

  calculateCirclePosition(posIteration, &desiredState);
  desiredStateList.push(desiredState);

  for (;;) { 
    if (!positionReceived) {
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
  geoCopy(&currentState, point->position);

  if (hasReachedTarget()) {
    // ROS_INFO("Reached target location: [%f, %f, %f]", desiredState.x, desiredState.y, desiredState.z);
    if (posIteration >= NUM_POINTS - 2) posIteration = 0; else ++posIteration;
    // geoCopy(&lastState, desiredState);
    if (desiredStateList.len() < NUM_POINTS) {
      calculateCirclePosition(posIteration, &desiredState);
      // ROS_INFO("Iteration Number: %d", posIteration);
      desiredStateList.push(desiredState);
    } else {
      geoCopy(&desiredState, desiredStateList[posIteration]);
    }
    pub.publish(desiredState);
   
    // ROS_INFO("Sent target location: [%f, %f, %f]", desiredState.x, desiredState.y, desiredState.z); 
    
  }

  positionReceived = true;
  ROS_INFO("Current Position: [%f, %f, %f]", currentState.x, currentState.y, currentState.z);
}

void checkDesiredStateListLength() {
  if (desiredStateList.len() > NUM_POINTS) {
    desiredStateList.popFront();
  }
}
