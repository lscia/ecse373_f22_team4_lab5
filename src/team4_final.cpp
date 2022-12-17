//INCLUDES

//Ros Includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "sensor_msgs/JointState.h"

//OSRF Gear Includes
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/GetMaterialLocations.h"

//CPP Includes
#include <sstream>
#include <vector>
#include <algorithm>
#include <array>
#include <string>

//Other Includes
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "ur_kinematics/ur_kin.h"

//Action Service Includes
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"




//STORAGE OF DATA

//Order vector
std::vector<osrf_gear::Order> orders;

//Logical Cam Arrays
osrf_gear::LogicalCameraImage cam_bins[6];
osrf_gear::LogicalCameraImage cam_agvs[2];
osrf_gear::LogicalCameraImage cam_qual[2];

//Joint States Storage (original o and corrected c)
sensor_msgs::JointState joint_states_o;
sensor_msgs::JointState joint_states_c;



//CALLBACKS

//Order Callback
void orderCallback(const osrf_gear::Order::ConstPtr& order)
{
  ROS_INFO("Recieved Order [%s] : # Of Shipments : [%d]", order->order_id.c_str(), order->shipments.size());
  orders.push_back(*order);
}

//Bin Camera Callbacks
void bin1CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_bins[0] = *msg;
}
void bin2CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_bins[1] = *msg;
}
void bin3CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_bins[2] = *msg;
}
void bin4CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_bins[3] = *msg;
}
void bin5CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_bins[4] = *msg;
}
void bin6CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_bins[5] = *msg;
}

//AGV Camera Callbacks
void agv1CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_agvs[0] = *msg;
}
void agv2CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_agvs[1] = *msg;
}

//Quality Camera Callbacks
void qual1CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_qual[0] = *msg;
}
void qual2CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_qual[1] = *msg;
}

//Joint State Callback
void jointCallback(const sensor_msgs::JointState::ConstPtr& current_joint_states) {
  //Stores the original joint state with the wrong order of joints
  joint_states_o.name = current_joint_states->name;
  joint_states_o.position = current_joint_states->position;
  joint_states_o.velocity = current_joint_states->velocity;
  joint_states_o.effort = current_joint_states->effort;

  //makes a copy of the joint state with all joints in the CORRECT, SENSIBLE ORDER 
  joint_states_c.name = {"linear_arm_actuator_joint", "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  joint_states_c.position.clear();
  joint_states_c.position.resize(7);
  joint_states_c.velocity.clear();
  joint_states_c.velocity.resize(7);
  joint_states_c.effort.clear();
  joint_states_c.effort.resize(7);


  if(joint_states_o.name.size() > 0 && joint_states_o.position.size() > 0 && joint_states_o.velocity.size() > 0 && joint_states_o.effort.size() > 0){
    for(int i = 0; i < 7; i++){
      for(int j = 0; j<7; j++){
        if(joint_states_c.name.at(i) == joint_states_o.name.at(j)){
          joint_states_c.position.at(i) = joint_states_o.position.at(j);
          joint_states_c.velocity.at(i) = joint_states_o.velocity.at(j);
          joint_states_c.effort.at(i) = joint_states_o.effort.at(j);
          break;
        }
      }
    } 
  }
}




//MAIN FUNCTION

int main(int argc, char **argv)
{


  //VARIABLES

  //Create a pose to store the product pose with respect to (WRT) the camera
  geometry_msgs::Pose product_pose_wrt_camera;

  //Create a pose to store the product pose wrt the arm base
  geometry_msgs::Pose product_pose_wrt_arm;

  //Camera image for the current bin
  osrf_gear::LogicalCameraImage bin_image;

  //Transform for converting from the camera frame to the arm frame
  geometry_msgs::TransformStamped tfStamped;

  //String for camera frame name
  std::string cam_frame;

  //Kinematics variables
  double T_pose[4][4], T_des[4][4];
  double q_pose[6], q_des[8][6];

  //Joint Trajectory Message
  trajectory_msgs::JointTrajectory joint_trajectory;

  //Joint Trajectory Action Server
  control_msgs::FollowJointTrajectoryAction joint_trajectory_as;

  //joint_states temporary name holder
  std::vector<std::string> joint_names_temp;

  //joint states temp angle holder
  std::vector<double> joint_angles_temp;

  //Initial arm angles
  std::vector<double> q_int = {3.14, -1.13, 1.51, 3.77, -1.51, 0};

  //Bin Locations Along Linear Rail (determined through testing)
  std::vector<double> bin_locs = {-1.9, -1.15, -0.35, 0.35, 1.15, 1.9};

  //For getting IK sol
  bool foundSol = false;
  int q_des_indx = 0;

  //Clears all vectors
  orders.clear();
  joint_names_temp.clear();
  joint_angles_temp.clear();


  //ROS NODE SETUP

  //Ros Node 
  ros::init(argc, argv, "team4_final");
  ros::NodeHandle n;


  //Multi Threaded Spinner Stuff
  ros::AsyncSpinner spinner(4);
  spinner.start();


  //Subscribe to the Orders topic
  ros::Subscriber orders_sub = n.subscribe("/ariac/orders", 1000, orderCallback);

  //Subscribe to Logical Camera Topics
  ros::Subscriber cam_bin1_sub = n.subscribe("/ariac/logical_camera_bin1", 1000, bin1CamCallback);
  ros::Subscriber cam_bin2_sub = n.subscribe("/ariac/logical_camera_bin2", 1000, bin2CamCallback);
  ros::Subscriber cam_bin3_sub = n.subscribe("/ariac/logical_camera_bin3", 1000, bin3CamCallback);
  ros::Subscriber cam_bin4_sub = n.subscribe("/ariac/logical_camera_bin4", 1000, bin4CamCallback);
  ros::Subscriber cam_bin5_sub = n.subscribe("/ariac/logical_camera_bin5", 1000, bin5CamCallback);
  ros::Subscriber cam_bin6_sub = n.subscribe("/ariac/logical_camera_bin6", 1000, bin6CamCallback);
  
  ros::Subscriber cam_agv1_sub = n.subscribe("/ariac/logical_camera_agv1", 1000, agv1CamCallback);
  ros::Subscriber cam_agv2_sub = n.subscribe("/ariac/logical_camera_agv2", 1000, agv2CamCallback);
  
  ros::Subscriber cam_qual1_sub = n.subscribe("/ariac/quality_control_sensor_1", 1000, qual1CamCallback);
  ros::Subscriber cam_qual2_sub = n.subscribe("/ariac/quality_control_sensor_2", 1000, qual2CamCallback);

  //Subscribe to the Joint State Topic
  ros::Subscriber joint_states_sub = n.subscribe("/ariac/arm1/joint_states", 1000, jointCallback);

  //Service to start comp
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  
  //Service to get material locations
  ros::ServiceClient mat_loc_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  
  //ros::Publisher joint_trajectory_pub = n.advertise<trajectory_msgs::JointTrajectory>("", 1000);


  //Transform Buffer and Listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  //Action Server
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>trajectory_as("ariac/arm1/arm/follow_joint_trajectory", true);
  ROS_INFO("WAITING FOR ACTION SERVER");
  trajectory_as.waitForServer();
  ROS_INFO("ACTION SERVER STARTED");

  ros::Rate loop_rate(10);


  //STARTING THE COMPETITION

  //Services Stuff
  int service_call_succeeded;
  std_srvs::Trigger begin_comp;
  std_srvs::SetBool my_bool_var;
  my_bool_var.request.data = true;

  //call start_competition service
  begin_client.waitForExistence();
  service_call_succeeded = begin_client.call(begin_comp);
  if(!service_call_succeeded){
    ROS_ERROR("Competition service call failed");
    ros::shutdown();
  }
  if(begin_comp.response.success){
    ROS_INFO("Competiton service called successfully [%s]", begin_comp.response.message.c_str());
  }
  else{
    ROS_ERROR("Competition failed to start, service called");
  }
  
  //MAIN LOOP

  int count = 0;
  while (ros::ok())
  {

    
    //Iterate through all orders
    if(!orders.empty()){
    for(const auto& order : orders)
    {
      ROS_INFO("-------------");
      ROS_INFO("ORDER ID: [%s]", order.order_id.c_str());
      if(!order.shipments.empty())
      {
        //Iterate through all shipments in order
        for(const auto& shipment : order.shipments)
        {
          ROS_INFO("--------");
          ROS_INFO("SHIPMENT ID: [%s]", shipment.shipment_type.c_str());
          if(!shipment.products.empty())
          {
            //Iterate through all products in shipment
            for(const auto& product : shipment.products)
            {

              

              //Find the bin of the product using get_mat_loc client
              int n;
              osrf_gear::GetMaterialLocations get_mat_loc;
              get_mat_loc.request.material_type = product.type;
              mat_loc_client.call(get_mat_loc);
              std::string bin;
              n = 0;
              for(const auto& unit : get_mat_loc.response.storage_units){
                if(unit.unit_id.compare("belt") != 0){
                  n = unit.unit_id[3] - '0';
                  break;
                }
              }
              
              ROS_INFO("PRODUCT TYPE: [%s] IS IN BIN: [%i]", product.type.c_str(), n);



              //Find the camera for that bin
              bin_image = cam_bins[n-1];

              //Find all the materials from that cameras view
              for(const auto& model : bin_image.models)
              {
                //Search for the first desired product
                if(model.type == product.type)
                {
                  //Get the pose of the product from that camera image
                  product_pose_wrt_camera = model.pose;
                }
              }
              

              ROS_INFO("PRODUCT IS AT x: %s y: %s z: %s WRT CAMERA", std::to_string(product_pose_wrt_camera.position.x).c_str(), std::to_string(product_pose_wrt_camera.position.y).c_str(), std::to_string(product_pose_wrt_camera.position.z).c_str());
              

              //Create trajectory message and fill out header
              joint_trajectory.header.seq = count++;
              joint_trajectory.header.stamp = ros::Time::now();
              joint_trajectory.header.frame_id = "/world";

              //Fill out names of joints
              joint_trajectory.joint_names.clear();
              joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
              joint_trajectory.joint_names.push_back("shoulder_pan_joint");
              joint_trajectory.joint_names.push_back("shoulder_lift_joint");
              joint_trajectory.joint_names.push_back("elbow_joint");
              joint_trajectory.joint_names.push_back("wrist_1_joint");
              joint_trajectory.joint_names.push_back("wrist_2_joint");
              joint_trajectory.joint_names.push_back("wrist_3_joint");

              //Start and end points
              joint_trajectory.points.clear();
              joint_trajectory.points.resize(2);



              // Set the start point to the current position of the joints from joint_states_c.
              joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());

              for (int y = 0; y < joint_trajectory.joint_names.size(); y++) {
                for (int z = 0; z < joint_states_o.name.size(); z++) {
                  if (joint_trajectory.joint_names[y] == joint_states_o.name[z]) {
                    joint_trajectory.points[0].positions[y] = joint_states_o.position[z];
                    break;
                  }
                }
              }

              //When to start 
              joint_trajectory.points[0].time_from_start = ros::Duration(0.5);


              // Set the end point for the movement
              joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());

              // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
              joint_trajectory.points[1].positions[0] = joint_states_c.position[0];
              
              /*
              // The actuators are commanded in an odd order, enter the joint positions in the correct positions
              for (int y = 0; y < 6; y++) {
                joint_trajectory.points[1].positions[y + 1] = q_des[q_des_indx][y];
              }
              */



              //joint_trajectory.points[1].positions.clear();
              //joint_trajectory.points[1].positions={2.5, 0.0, 3.14, 3.14, 3.77, -1.51, 0}; 
              //Default Angles w/ Original Order
              //ELBOW, RAIL, SHOULDER_LIFT(do not go too low <3.14), SHOULDER_PAN, W1, W2, W3

              joint_trajectory.points[1].positions.clear();
              joint_trajectory.points[1].positions={0.0, 3.14, 3.14, 2.5, 3.77, -1.51, 0}; 
              joint_trajectory.points[1].positions[0] = bin_locs[n-1];
              //Default angles w/ corrected order
              //Linear, S_lift, S_pan, Elbow, W1, W2, W3


              /*
              //resorts it according to the local order of joints from joint_states_o
              //double loop of 7
              joint_angles_temp.clear();
              for(int i = 0; i < 7; i++){
                for(int j = 0; j<7; j++){
                  //check if name of original matches to name of corrected
                  if(joint_states_o.name.at(i) == joint_states_c.name.at(j)){
                    //sort from corrected position to original position in vector
                    joint_angles_temp.push_back(joint_trajectory.points[1].positions.at(j));
                    break;
                  }
                }
              }
              //copy the de-corrected order back over
              joint_trajectory.points[1].positions = joint_angles_temp;
              joint_angles_temp.clear();
              */



              

              // How long to take for the movement.
              joint_trajectory.points[1].time_from_start = ros::Duration(5);


              //Trajectory action server to make arm to move to goal pose
              joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
              joint_trajectory_as.action_goal.header.seq = count++;
              joint_trajectory_as.action_goal.header.stamp = ros::Time::now();
              joint_trajectory_as.action_goal.header.frame_id = "/world";
              joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
              joint_trajectory_as.action_goal.goal_id.id = std::to_string(count -1);

              ROS_INFO("MOVING TOWARDS BIN [%i] LOCATION", n);
              
              actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(10.0));
              
              ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());




              //Find the reference frame of the correct camera
              cam_frame = "logical_camera_bin";
              cam_frame.append(std::to_string(n));
              cam_frame.append("_frame");

              //Get the transform from that camera to the arm base
              try {
                tfStamped = tfBuffer.lookupTransform("arm1_base", cam_frame, ros::Time(0.0), ros::Duration(1.0));
                ROS_INFO("Transform to [%s] from [%s}]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
              } catch (tf2::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
              }

              //Apply the transform to get the goal pose and correct it slightly
              tf2::doTransform(product_pose_wrt_camera, product_pose_wrt_arm, tfStamped);
              
              product_pose_wrt_arm.position.z += 0.10; //Move 10 Cm above the part
              product_pose_wrt_arm.orientation.w = 0.707; //Rotate the effector 90 deg around the y axis
              product_pose_wrt_arm.orientation.x = 0.0;
              product_pose_wrt_arm.orientation.y = 0.707;
              product_pose_wrt_arm.orientation.z = 0.0;
              ROS_INFO("PRODUCT IS AT x: %s y: %s z: %s WRT ARM", std::to_string(product_pose_wrt_arm.position.x).c_str(), std::to_string(product_pose_wrt_arm.position.y).c_str(), std::to_string(product_pose_wrt_arm.position.z).c_str());
              



              //Send the goal pose to IK
              T_des[0][0] = 0.0;  T_des[0][1] = -1.0; T_des[0][2] = 0.0;  T_des[0][3] = product_pose_wrt_arm.position.x;
              T_des[1][0] = 0.0;  T_des[1][1] = 0.0;  T_des[1][2] = 1.0;  T_des[1][3] = product_pose_wrt_arm.position.y;
              T_des[2][0] = -1.0; T_des[2][1] = 0.0;  T_des[2][2] = 0.0;  T_des[2][3] = product_pose_wrt_arm.position.z;
              T_des[3][0] = 0.0;  T_des[3][1] = 0.0;  T_des[3][2] = 0.0;  T_des[3][3] = 1.0;
              int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);





              //ISSUES WITH IK POSE, IT RETURNS ZEROES OR RANDOM VALUES
              //filter IK results

              // Must select which of the num_sols solutions to use.  Just start with the first.
              

              //Go through all 8 returned sols
              foundSol = false;
              for(int x = 0; x <8; x++){
                ROS_INFO("THE [%d] SOL IS [%s] [%s] [%s] [%s] [%s] [%s]", x, std::to_string(q_des[x][0]).c_str(), std::to_string(q_des[x][1]).c_str(), std::to_string(q_des[x][2]).c_str(), std::to_string(q_des[x][3]).c_str(), std::to_string(q_des[x][4]).c_str(), std::to_string(q_des[x][5]).c_str());
                foundSol = true;
                //check if all angles are in a sensible range
                for(int y = 0; y<6; y++){
                  if(!(q_des[x][y] <= 6.28)){
                    foundSol = false;
                    break;
                  }
                }
                //check if they are not all zeros (broken IK)
                if((q_des[x][0] + q_des[x][1] + q_des[x][2] + q_des[x][3] + q_des[x][4] + q_des[x][5]) < 0.1){
                  foundSol = false;
                }
                if(foundSol){
                  q_des_indx = x;
                  break;
                }
              }



              //Create trajectory message and fill out header
              joint_trajectory.header.seq = count++;
              joint_trajectory.header.stamp = ros::Time::now();
              joint_trajectory.header.frame_id = "/world";

              //Start and end points
              joint_trajectory.points.clear();
              joint_trajectory.points.resize(2);



              // Set the start point to the current position of the joints from joint_states_c.
              joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());

              for (int y = 0; y < joint_trajectory.joint_names.size(); y++) {
                for (int z = 0; z < joint_states_o.name.size(); z++) {
                  if (joint_trajectory.joint_names[y] == joint_states_o.name[z]) {
                    joint_trajectory.points[0].positions[y] = joint_states_o.position[z];
                    break;
                  }
                }
              }

              //When to start 
              joint_trajectory.points[0].time_from_start = ros::Duration(0.5);


              // Set the end point for the movement
              joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());

              //q_des is in strange order
              //resolve it to actual order for it to work (j_s_o -> j_s_c)
              if(foundSol){
                /*
                for(int x = 0; x < 7; x++){
                  for (int y = 0; y < 7; y++) {
                    if(joint_states_o.name.at(y) == "linear_arm_actuator_joint"){
                      
                    }
                    else if(joint_states_o.name.at(y) == joint_states_c.name.at(x)){
                      joint_trajector.points[1].positions[] = q_des[q_des_indx][];
                    }
                  } 
                }
                */
                for(int y = 0; y < 6; y++){
                  joint_trajectory.points[1].positions[y + 1] = q_des[q_des_indx][y];
                }

              //get IK goal pose using forwards kinematics
              q_pose[0] = q_des[q_des_indx][0];
              q_pose[1] = q_des[q_des_indx][1];
              q_pose[2] = q_des[q_des_indx][2];
              q_pose[3] = q_des[q_des_indx][3];
              q_pose[4] = q_des[q_des_indx][4];
              q_pose[5] = q_des[q_des_indx][5];
              ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);
              ROS_INFO("ARM WILL MOVE TO x: %s y: %s z: %s WRT ARM", std::to_string(T_pose[0][3]).c_str(), std::to_string(T_pose[1][3]).c_str(), std::to_string(T_pose[2][3]).c_str());
              

                /*
                joint_trajectory.points[1].positions[1] = q_des[q_des_indx][1];
                joint_trajectory.points[1].positions[2] = q_des[q_des_indx][2];
                joint_trajectory.points[1].positions[3] = q_des[q_des_indx][0];
                joint_trajectory.points[1].positions[4] = q_des[q_des_indx][3];
                joint_trajectory.points[1].positions[5] = q_des[q_des_indx][4];
                joint_trajectory.points[1].positions[6] = q_des[q_des_indx][5];
                */

                
              }
              else if(!foundSol){
                joint_trajectory.points[1].positions.clear();
                joint_trajectory.points[1].positions={0.0, 3.14, 3.14, 2.5, 3.77, -1.51, 0}; 
              }
              joint_trajectory.points[1].positions[0] = bin_locs[n-1];



              // How long to take for the movement.
              joint_trajectory.points[1].time_from_start = ros::Duration(5);


              //Trajectory action server to make arm to move to goal pose
              joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
              joint_trajectory_as.action_goal.header.seq = count++;
              joint_trajectory_as.action_goal.header.stamp = ros::Time::now();
              joint_trajectory_as.action_goal.header.frame_id = "/world";
              joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
              joint_trajectory_as.action_goal.goal_id.id = std::to_string(count -1);
              ROS_INFO("GRABBING PART");
              state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(10.0));
              ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());










              //Create trajectory message and fill out header
              joint_trajectory.header.seq = count++;
              joint_trajectory.header.stamp = ros::Time::now();
              joint_trajectory.header.frame_id = "/world";

              //Fill out names of joints
              joint_trajectory.joint_names.clear();
              joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
              joint_trajectory.joint_names.push_back("shoulder_pan_joint");
              joint_trajectory.joint_names.push_back("shoulder_lift_joint");
              joint_trajectory.joint_names.push_back("elbow_joint");
              joint_trajectory.joint_names.push_back("wrist_1_joint");
              joint_trajectory.joint_names.push_back("wrist_2_joint");
              joint_trajectory.joint_names.push_back("wrist_3_joint");

              //Start and end points
              joint_trajectory.points.clear();
              joint_trajectory.points.resize(2);



              // Set the start point to the current position of the joints from joint_states_c.
              joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());

              for (int y = 0; y < joint_trajectory.joint_names.size(); y++) {
                for (int z = 0; z < joint_states_o.name.size(); z++) {
                  if (joint_trajectory.joint_names[y] == joint_states_o.name[z]) {
                    joint_trajectory.points[0].positions[y] = joint_states_o.position[z];
                    break;
                  }
                }
              }

              //When to start 
              joint_trajectory.points[0].time_from_start = ros::Duration(0.5);


              // Set the end point for the movement
              joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());

              // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
              joint_trajectory.points[1].positions[0] = joint_states_c.position[0];
              
              /*
              // The actuators are commanded in an odd order, enter the joint positions in the correct positions
              for (int y = 0; y < 6; y++) {
                joint_trajectory.points[1].positions[y + 1] = q_des[q_des_indx][y];
              }
              */



              //joint_trajectory.points[1].positions.clear();
              //joint_trajectory.points[1].positions={2.5, 0.0, 3.14, 3.14, 3.77, -1.51, 0}; 
              //Default Angles w/ Original Order
              //ELBOW, RAIL, SHOULDER_LIFT(do not go too low <3.14), SHOULDER_PAN, W1, W2, W3

              joint_trajectory.points[1].positions.clear();
              joint_trajectory.points[1].positions={0.0, 3.14, 3.14, 2.5, 3.77, -1.51, 0}; 
              joint_trajectory.points[1].positions[0] = bin_locs[n-1];
              //Default angles w/ corrected order
              //Linear, S_lift, S_pan, Elbow, W1, W2, W3


              /*
              //resorts it according to the local order of joints from joint_states_o
              //double loop of 7
              joint_angles_temp.clear();
              for(int i = 0; i < 7; i++){
                for(int j = 0; j<7; j++){
                  //check if name of original matches to name of corrected
                  if(joint_states_o.name.at(i) == joint_states_c.name.at(j)){
                    //sort from corrected position to original position in vector
                    joint_angles_temp.push_back(joint_trajectory.points[1].positions.at(j));
                    break;
                  }
                }
              }
              //copy the de-corrected order back over
              joint_trajectory.points[1].positions = joint_angles_temp;
              joint_angles_temp.clear();
              */



              

              // How long to take for the movement.
              joint_trajectory.points[1].time_from_start = ros::Duration(5);


              //Trajectory action server to make arm to move to goal pose
              joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
              joint_trajectory_as.action_goal.header.seq = count++;
              joint_trajectory_as.action_goal.header.stamp = ros::Time::now();
              joint_trajectory_as.action_goal.header.frame_id = "/world";
              joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
              joint_trajectory_as.action_goal.goal_id.id = std::to_string(count -1);

              ROS_INFO("RESETTING POSE");
              
              state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(10.0));
              
              ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());




              //Create trajectory message and fill out header
              joint_trajectory.header.seq = count++;
              joint_trajectory.header.stamp = ros::Time::now();
              joint_trajectory.header.frame_id = "/world";

              //Start and end points
              joint_trajectory.points.clear();
              joint_trajectory.points.resize(2);



              // Set the start point to the current position of the joints from joint_states_c.
              joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());

              for (int y = 0; y < joint_trajectory.joint_names.size(); y++) {
                for (int z = 0; z < joint_states_o.name.size(); z++) {
                  if (joint_trajectory.joint_names[y] == joint_states_o.name[z]) {
                    joint_trajectory.points[0].positions[y] = joint_states_o.position[z];
                    break;
                  }
                }
              }

              //When to start 
              joint_trajectory.points[0].time_from_start = ros::Duration(0.5);


              // Set the end point for the movement
              joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());

              joint_trajectory.points[1].positions.clear();
              joint_trajectory.points[1].positions={0.0, 3.14, 3.14, 2.5, 3.77, -1.51, 0}; 





              joint_trajectory.points[1].positions[0] = bin_locs[n-1];

              // How long to take for the movement.
              joint_trajectory.points[1].time_from_start = ros::Duration(5);


              //Trajectory action server to make arm to move to goal pose
              joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
              joint_trajectory_as.action_goal.header.seq = count++;
              joint_trajectory_as.action_goal.header.stamp = ros::Time::now();
              joint_trajectory_as.action_goal.header.frame_id = "/world";
              joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
              joint_trajectory_as.action_goal.goal_id.id = std::to_string(count -1);
              ROS_INFO("RETURNING TO HOME");
              state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(10.0));
              ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());

              }
            }
          }
        }
      }
    }

    //chatter_pub.publish(msg);
  
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
