#include "geometry_helpers.hpp"
#include <string>

bool positionReceived = false;

ros::Subscriber sub;   // Subscriber to get current position of drone
ros::Publisher pub;// Publisher to tell the drone to move somewhere

void currentStateCallback(const simulator_utils::Waypoint::ConstPtr&);
bool is_number(const std::string&);
void checkDesiredStateListLength();

int main(int argc, char **argv) {

  if (argc < 2) {
    throw std::invalid_argument("Must supply an argument indicating robot_{id}");
    return -1;
  }

  if (!is_number(argv[1])) {
    throw std::invalid_argument("Value must be a number");
    return -1;
  }

  posIteration = std::stoi(argv[1]);

  getCenter();
  ros::init(argc, argv, "circular_mover"); // Initializes the ros node with arguments from ROS
  ros::NodeHandle n;  // Main access point to communications with the ROS system
  std::string robot_name = "/robot_";
  robot_name.append(argv[1]);
  std::string robot_current_state;
  std::string robot_desired_state;
  robot_current_state.append(robot_name).append("/current_state");
  robot_desired_state.append(robot_name).append("/desired_state");

  sub = n.subscribe(robot_current_state, 10, currentStateCallback);
  pub = n.advertise<geometry_msgs::Point>(robot_desired_state, 10);  
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

bool is_number(const std::string& s)
{
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
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
