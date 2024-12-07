#include <map>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_helpers.hpp"
#include "circular_motion/State.h"
#include "circular_motion/CalculateCirclePosition.h"

static int syncedEntities = 0;
static int numEntities = 0;
static int iteration = 0;
static std::map<int, geometry_msgs::Point> desiredStateBatch;

ros::Publisher point_rdy;
ros::Subscriber sync_state;
ros::ServiceClient circleCalculatorClient;
circular_motion::CalculateCirclePosition srv;

bool initialBatch = true;

void syncState_cb(const circular_motion::State::ConstPtr& msg) {
    // If the drone reaches halfway to its target, add to list* and when all drones have satisfied this: publish new positions
    srv.request.iteration = msg->id + iteration;
    if (circleCalculatorClient.call(srv)) {
        // ROS_INFO("Position: x=%f, y=%f, z=%f", srv.response.point.x, srv.response.point.y, srv.response.point.z);
        if (initialBatch || 
            (distanceFrom(srv.response.point, msg->point) < TOLERANCE && desiredStateBatch.find(msg->id) == desiredStateBatch.end())) {
            ROS_INFO("Drone %ld arrived", msg->id);
           desiredStateBatch[msg->id] = srv.response.point;
        }
    }
    // Now should calculate next positions for entities and then check to see if all drones are halfway (probably with a service)
    // Maybe look into doing it in batches, ie calculate new drone positions for all of them in the service and send all back at once
}

int main(int argc, char **argv) {
    // If argument is not presented, set numEntities to 0
    if (argc < 2) {
        numEntities = 0;
    } else {
        numEntities = atoi(argv[1]);
    }
	
    // Initialize ros node
	ros::init(argc, argv, "sync");
	ros::NodeHandle n;
    
    // TODO: Will be used to send back the calculated information 
	point_rdy = n.advertise<circular_motion::State>("point_rdy", 100);
    sync_state = n.subscribe("sync_state", 100, syncState_cb);

    circleCalculatorClient = n.serviceClient<circular_motion::CalculateCirclePosition>("calculate_circle_position");

	ros::Rate loop_rate(10);
	while (ros::ok()) {
        if (desiredStateBatch.size() == numEntities) {
            // Update drone states from batch
            for (const auto& node : desiredStateBatch) {
                circular_motion::State msg;
                msg.id = node.first;
                msg.point.x = node.second.x;
                msg.point.y = node.second.y;
                msg.point.z = node.second.z;
                point_rdy.publish(msg); // publish the new desiredState point value
            }
            iteration += 1;
            if (iteration == NUM_POINTS) {
                iteration = 0;   
            }
            if (initialBatch) {
                initialBatch = false;
            }
            desiredStateBatch.clear(); // Clear the map so we can use it again
        }
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
