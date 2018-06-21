#include "ros/ros.h"
#include "dsdm_msgs/dsdm_actuator_control_inputs.h"
#include "dsdm_msgs/dsdm_robot_control_inputs.h"

//  Dispatcher clas -----------------------------
class Dispatcher_Node{
private:

	// Node
	ros::NodeHandle n;
	
	// Sub-Pub
	ros::Subscriber sub; 
	ros::Publisher  pub_u0; // acutator #0
	ros::Publisher  pub_u1; // acutator #1
	ros::Publisher  pub_u2; // acutator #2
	
	
public:
// Functions
	void callback(const dsdm_msgs::dsdm_robot_control_inputs msg);
	
// Initialization
	Dispatcher_Node(){
		
		// Init Sub-pub
		sub   = n.subscribe("u", 1000, &Dispatcher_Node::callback, this );
		pub_u0 = n.advertise<dsdm_msgs::dsdm_actuator_control_inputs>("a0/u", 1000);
		pub_u1 = n.advertise<dsdm_msgs::dsdm_actuator_control_inputs>("a1/u", 1000);
		pub_u2 = n.advertise<dsdm_msgs::dsdm_actuator_control_inputs>("a2/u", 1000);
	}


};

// Class Functions source ---------------------

void Dispatcher_Node::callback(const dsdm_msgs::dsdm_robot_control_inputs msg){
	
	ROS_INFO("Dispatcher: u=[%f, %f] k=%d", (float)msg.F[0], (float)msg.F[1], (int)msg.k);
	
	// Create messages
	dsdm_msgs::dsdm_actuator_control_inputs msg_u0 ;
	dsdm_msgs::dsdm_actuator_control_inputs msg_u1 ;
	dsdm_msgs::dsdm_actuator_control_inputs msg_u2 ;
	
	// Build messages
	
	// Torques
	msg_u0.f  = msg.F[0];
	msg_u1.f  = msg.F[1];
	
	//Gear ratio selection
	switch ( msg.k )
	      {
	         case 0:
				msg_u0.k = 0;
				msg_u1.k = 0;
				break;
	         case 1:
				msg_u0.k = 1;
				msg_u1.k = 0;
				break;
	         case 2:
				msg_u0.k = 0;
				msg_u1.k = 1;
				break;
	         case 3:
				msg_u0.k = 1;
				msg_u1.k = 1;
				break;
	         default:
	        	// Error in decoding
	        	msg_u0.k = -1;
	        	msg_u1.k = -1;
	      }
	
	// Empty actuator #2

	msg_u2.f  = 0;
	msg_u2.k = -1;
	
	
	// Publish messages
	pub_u0.publish( msg_u0 );
	pub_u1.publish( msg_u1 );
	pub_u2.publish( msg_u2 );
}


//  --------- MAIN -----------------------------------
int main(int argc, char **argv){

	// Init ROS
	ros::init(argc, argv, "dispatcher");
	Dispatcher_Node dp_node;
	ros::spin();
	return 0;
}
