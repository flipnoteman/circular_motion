#include "geometry_helpers.hpp"
#include <string>
#include <std_msgs/Bool.h>
#include "circular_motion/State.h"

int id;

ros::Subscriber currentStateSub;   // Subscriber to get current position of drone
ros::Subscriber desiredStateSub;   // Subscriber to get current position of drone
ros::Publisher desiredStatePub;// Publisher to tell the drone to move somewhere
ros::Publisher sync_state;

void currentStateCallback(const simulator_utils::Waypoint::ConstPtr&);
void desiredStateCallback(const circular_motion::State::ConstPtr&);
void rdyStateCallback(const std_msgs::Bool::ConstPtr&);
bool is_number(const std::string&);
void pubInitialCoordinates();
int main(int argc, char **argv) {

  if (argc < 2) {
    id = 0;
  } else {
    id = std::stoi(argv[1]);
  }

  getCenter();
  ros::init(argc, argv, "circular_mover"); // Initializes the ros node with arguments from ROS
  ros::NodeHandle n;  // Main access point to communications with the ROS system
  std::string robot_name = "/robot_";
  robot_name.append(argv[1]);
  std::string robot_current_state;
  std::string robot_desired_state;
  robot_current_state.append(robot_name).append("/current_state");
  robot_desired_state.append(robot_name).append("/desired_state");

  currentStateSub = n.subscribe(robot_current_state, 100, currentStateCallback);
  desiredStateSub = n.subscribe("point_rdy", 100, desiredStateCallback);
  desiredStatePub = n.advertise<geometry_msgs::Point>(robot_desired_state, 100);  
  sync_state = n.advertise<circular_motion::State>("sync_state", 100);

  ros::Rate loop_rate(100); //10 Hz

  for (;;) { 
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

bool is_number(const std::string& s)
{
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
}


void currentStateCallback(const simulator_utils::Waypoint::ConstPtr& point) 
{
    // Publish current point
    circular_motion::State msg;
    msg.id = id;
    msg.point = point->position;
    sync_state.publish(msg);

    //ROS_INFO("Current Position: [%f, %f, %f]", currentState.x, currentState.y, currentState.z);
}

void desiredStateCallback(const circular_motion::State::ConstPtr& state) {
    int _id_rec = state->id;
    geometry_msgs::Point point = state->point;

    if (_id_rec == id) {
        ROS_INFO("Next state for %d = %f, %f, %f", _id_rec, point.x, point.y, point.z);
        geometry_msgs::Point msg;
        msg.x = point.x;
        msg.y = point.y;
        msg.z = point.z;
        desiredStatePub.publish(msg);
    }
}

void checkDesiredStateListLength() {
  if (desiredStateList.len() > NUM_POINTS) {
    desiredStateList.popFront();
  }
}

void rdyStateCallback(const std_msgs::Bool::ConstPtr& msg) {
}
