// example_part_fetcher_client: 
// wsn, Nov, 2016

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include<my_ps9/ps9Action.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

double conveyor_speed = 0.5; // m/s
double wait_location;
double approach_location;
double grasp_location;
double time_to_waitPosition = 0.9;
double time_to_approachPosition = 0.9;
double time_to_grasp = 0.2;
std_msgs::Float64 position;
trajectory_msgs::JointTrajectory linear_movements;

sensor_msgs::JointState current_joint_states_;
bool has_been_zeroed_ = false;

void joint_state_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg)
  {
    //ROS_INFO_STREAM_THROTTLE(1,"Joint States (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    current_joint_states_ = *joint_state_msg;

    if (!has_been_zeroed_) {
      has_been_zeroed_ = true;
      ROS_INFO("Sending arm to zero joint positions...");
      //send_arm_to_zero_state();
    }
  }

void set_example_object_frames(geometry_msgs::PoseStamped &object_poseStamped,
        geometry_msgs::PoseStamped &object_dropoff_poseStamped) {
    //hard code an object pose; later, this will come from perception
    //specify reference frame in which this pose is expressed:
    //will require that "system_ref_frame" is known to tf
    object_poseStamped.header.frame_id = "system_ref_frame"; //set object pose; ref frame must be connected via tf
    object_poseStamped.pose.position.x = -0.85;
    object_poseStamped.pose.position.y = 0.85;
    object_poseStamped.pose.position.z = 0.5622; //-0.125; //pose w/rt world frame
    object_poseStamped.pose.orientation.x = 0;
    object_poseStamped.pose.orientation.y = 0;
    object_poseStamped.pose.orientation.z = 0.0;
    object_poseStamped.pose.orientation.w = 1;
    object_poseStamped.header.stamp = ros::Time::now();

    object_dropoff_poseStamped = object_poseStamped; //specify desired drop-off pose of object
    object_dropoff_poseStamped.pose.position.x = 1;
    object_dropoff_poseStamped.pose.position.y = 0;
    object_dropoff_poseStamped.pose.position.z = 0.793; 
}

void set_pulley_object_frames(geometry_msgs::PoseStamped &object_poseStamped,
        geometry_msgs::PoseStamped &object_dropoff_poseStamped) {
    //hard code an object pose; later, this will come from perception
    //specify reference frame in which this pose is expressed:
    //will require that "system_ref_frame" is known to tf
    //object_poseStamped.header.frame_id = "system_ref_frame"; //set object pose; ref frame must be connected via tf
    object_poseStamped.header.frame_id = "base_link";
    object_poseStamped.pose.position.x = 0.9;//-0.5;
    object_poseStamped.pose.position.y = 0;
    object_poseStamped.pose.position.z = 0.3;//0.5622; //-0.125; //pose w/rt world frame
    object_poseStamped.pose.orientation.x = 0;
    object_poseStamped.pose.orientation.y = 0;
    object_poseStamped.pose.orientation.z = 0.0;
    object_poseStamped.pose.orientation.w = 1;
    object_poseStamped.header.stamp = ros::Time::now();

    object_dropoff_poseStamped = object_poseStamped; //specify desired drop-off pose of object
    object_dropoff_poseStamped.pose.position.x = 1;
    object_dropoff_poseStamped.pose.position.y = 0;
    object_dropoff_poseStamped.pose.position.z = 0.9;//0.793; 
}

void set_gasket_object_frames(geometry_msgs::PoseStamped &object_poseStamped,
        geometry_msgs::PoseStamped &object_dropoff_poseStamped) {
    //hard code an object pose; later, this will come from perception
    //specify reference frame in which this pose is expressed:
    //will require that "system_ref_frame" is known to tf
    /*object_poseStamped.header.frame_id = "system_ref_frame"; //set object pose; ref frame must be connected via tf
    object_poseStamped.pose.position.x = 0.0;
    object_poseStamped.pose.position.y = 0.85;
    object_poseStamped.pose.position.z = 0.54; //-0.125; //pose w/rt world frame
    object_poseStamped.pose.orientation.x = 0;
    object_poseStamped.pose.orientation.y = 0;
    object_poseStamped.pose.orientation.z = 0.0;
    object_poseStamped.pose.orientation.w = 1;
    */
    object_poseStamped.header.frame_id = "system_ref_frame"; //set object pose; ref frame must be connected via tf
    object_poseStamped.pose.position.x = 0.0;
    object_poseStamped.pose.position.y = 0.85;
    object_poseStamped.pose.position.z = 0.54; //-0.125; //pose w/rt world frame
    object_poseStamped.pose.orientation.x = 0;
    object_poseStamped.pose.orientation.y = 0;
    object_poseStamped.pose.orientation.z = 0.0;
    object_poseStamped.pose.orientation.w = 1;

    object_poseStamped.header.stamp = ros::Time::now();

    object_dropoff_poseStamped = object_poseStamped; //specify desired drop-off pose of object
    object_dropoff_poseStamped.pose.position.x = 1.0;
    object_dropoff_poseStamped.pose.position.y = 0.0;
    object_dropoff_poseStamped.pose.position.z = 0.9; 
}

void doneCb(const actionlib::SimpleClientGoalState& state,
        const my_ps9::ps9ResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got result rtn_val = %d", result->rtn_code);
}

void new_part_callback(const std_msgs::Float64::ConstPtr& y){

 position = *y;






}

void calc_distances(){

// calculate to the location of the robot at the wait position currently half of approach position

    double contact_location = position.data - (time_to_waitPosition + time_to_approachPosition)*conveyor_speed;

// conditional statements about contact_location being past the end


    double total_distance = abs(contact_location - current_joint_states_.position[1]);

// bunch of conditional statements about whether this is possible in terms of robot speeds

    if (contact_location > current_joint_states_.position[1]){

	wait_location = current_joint_states_.position[1] + (time_to_waitPosition)/(time_to_waitPosition + time_to_approachPosition)*total_distance;

    }
    else {

	wait_location = current_joint_states_.position[1] - (time_to_waitPosition)/(time_to_waitPosition + time_to_approachPosition)*total_distance;

    } 

    // approach position will always be the same as the contact location
    approach_location = contact_location;
    grasp_location = contact_location - time_to_grasp*conveyor_speed;



trajectory_msgs::JointTrajectory msg;
msg.joint_names = current_joint_states_.name;
msg.points.resize(1);
msg.points[0].positions.resize(current_joint_states_.name.size(), 0.0);
msg.points[0].positions[0] = wait_location;
msg.points[0].positions[1] = approach_location;
msg.points[0].positions[2] = grasp_location;
linear_movements = msg;

}








int main(int argc, char** argv) {
    ros::init(argc, argv, "part_fetcher_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle
    // here is a "goal" object compatible with the server, 
    my_ps9::ps9Goal goal;
    geometry_msgs::PoseStamped object_pickup_poseStamped;
    geometry_msgs::PoseStamped object_dropoff_poseStamped;
    //specify object pick-up and drop-off frames 
    set_pulley_object_frames(object_pickup_poseStamped, object_dropoff_poseStamped);
    position.data = 0.0;
    actionlib::SimpleActionClient<my_ps9::ps9Action> action_client("ps9_fetcher", true);
    ros::Subscriber joint_state_subscriber = nh.subscribe("/ariac/joint_states", 1,joint_state_callback);
    ros::Subscriber part_subscriber = nh.subscribe("/ariac/new/part", 1, new_part_callback);
    ros::Publisher linear_states = nh.advertise<trajectory_msgs::JointTrajectory>("linear_states", 1);

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    //bool server_exists = action_client.waitForServer(); //wait forever

    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }


    ROS_INFO("connected to action server"); // if here, then we connected to the server;

        // stuff a goal message:
        goal.object_id = ObjectIdCodes::ARIAC_PULLEY; 
        goal.object_frame = object_pickup_poseStamped;
        goal.desired_frame =  object_dropoff_poseStamped;       
       
        action_client.sendGoal(goal, &doneCb); // we could also name additional callback functions here, if desired
       

        bool finished_before_timeout = action_client.waitForResult(ros::Duration(20.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 0;
        } 

       while(ros::ok()){
	ros::spinOnce();
        ros::Duration(0.1).sleep(); //don't consume much cpu time if not actively working on a command
	ROS_INFO("SENDING LINEAR LOCATIONS");
	calc_distances();
	linear_states.publish(linear_movements);


	}


    return 0;
}

