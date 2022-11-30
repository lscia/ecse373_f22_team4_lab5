//ros includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Pose.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

//osrf_gear includes
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/GetMaterialLocations.h"

//c++ includes
#include <algorithm>
#include <vector>
#include <sstream>

//declare vectors for data
std::vector<osrf_gear::Order> order_vector;
std::vector<std::vector<osrf_gear::StorageUnit>> product_bin_vector;
std::vector<std::string> product_type_vector;

//service call stuff
int service_call_succeeded;
std_srvs::Trigger begin_comp;
std_srvs::SetBool my_bool_var;

//Logical Camera Image vectors
//Not the most space-efficient way of declaring but works good enough
std::vector<osrf_gear::LogicalCameraImage> cam_bins_vector(6, new osrf_gear::LogicalCameraImage);
std::vector<osrf_gear::LogicalCameraImage> cam_agvs_vector(2, new osrf_gear::LogicalCameraImage);
std::vector<osrf_gear::LogicalCameraImage> cam_qs_vector(2, new osrf_gear::LogicalCameraImage);
int currentCamera = 0;

//tf2 buffer
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);


//subscriber callbacks
void orderCallback(const osrf_gear::Order::ConstPtr& order)
{
  order_vector.push_back(*order);
  ROS_INFO("I recieved an order: [%s]", *order.order_id);
  // getting the order ID
}

//Logical Camera Callbacks
//Not the most space-efficient way of declaring but works good enough
void bin1CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_bins_vector.at(0) = *msg;
}
void bin2CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_bins_vector.at(1) = *msg;
}
void bin3CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_bins_vector.at(2) = *msg;
}
void bin4CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_bins_vector.at(3) = *msg;
}
void bin5CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_bins_vector.at(4) = *msg;
}
void bin6CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_bins_vector.at(5) = *msg;
}
void agv1CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_agvs_vector.at(0) = *msg;
}
void agv2CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_agvs_vector.at(1) = *msg;
}
void q1CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_qs_vector.at(0) = *msg;
}
void q2CamCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_qs_vector.at(0) = *msg;
}

geometry_msgs::PoseStamped part_pose, goal_pose;

//part_pose.pose = 
tf2::doTransform(part_pose, goal_pose, transformStamped);

//transforms between the arm and logical camera frames
geometry_mgs::TransformStamped get_transform(const std:String& targ_frame, const std:String& orig_frame){ 
  geometry_msgs::TransformStamped tfStamped
  try {
    tfStamped = tfBuffer.lookupTransform(*targ_frame, *orig_frame, ros::Time(0.0), ros::Duration(1.0));
    ROS_DEBUG("Transform to [%s] from [%s}]", tfStamped.header.frame_id.c_str(). tfStamped.child_frame_id.c_str());
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }
}

//get the pose of a product in a storage unit
geometry_msgs::Pose get_product_pose(const std:String& prod_type, const osrf_gear::StorageUnit &unit)
{
  //a vector of models for logical cam image data
  std::vector<osrf_gear::Model> cam_image_models;
  cam_image_models.clear();(need to rewrite logical camera subscribers to use vector or array
  //a pose to return
  geomtry_msgs::Pose product_pose;
  //store the unit id of the first bin for the product
  std::String bin = *unit.unit_id;
  //check if it isnt on the belt
  if(bin.compare("belt") != 0){
    //isolate just the bin number from the string, and convert it to an int for a vector position
    char n = bin[3];
    int b = n - '0';
    b = b -1;
    //get the models from that logical camera
    currentCamera = b;
    cam_image_models.assign(cam_bins_vector.at(b).models);
    //for each model, c(need to rewrite logical camera subscribers to use vector or arrayheck if it matches the product type
    for (const auto& model : cam_image_models){
      if(model.type.compare(*product.type) == 0){
        //if it matches, output the pose
        product_pose = model.pose;
        break;
      }
    }
  }
  //returns the pose
  return product_pose;
}


//get all the products required for an order
void get_products(const osrf_gear::Order& order)
{
  //go through all shipments in the order
  for (const auto& shipment : order.shipments){
    //go through all products in the shipment
    for (const auto& product : shipment.products){
      //adds the product type to product_type_vector
      product_type_vector.push_back(product.type);
      //requests the locations of the product
      osrf_gear::GetMaterialLocations get_mat_loc;
      get_mat_loc.request.material_type=product.type;
      mat_loc_client.call(get_mat_loc);
      //adds the vector of bins it can be found in to product_bin_vector
      product_bin_vector.push_back(get_mat_loc.response.storage_units);
      //ros_info outputs the info for each product
      geometry_msgs::Pose prod_pose = get_product_pose(&product.type, &get_mat_loc.response.storage_units.front())
      ROS_INFO("PRODUCT: [%s] BIN : [%s] POSE : [%s]", product.type, get_mat_loc.response.storage_units.unit_id, prod_pose);
    }
  }
}


//main loop from example wiki code for publisher/subscriber
int main(int argc, char **argv)
{
  //clear vectors
  order_vector.clear();
  product_bin_vector.clear();
  product_type_vector.clear();
  cam_image_vector.clear();get

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

  ros::init(argc, argv, "team4_lab5");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  //subscribes to all topics 
  ros::Subscriber orders = n.subscribe("/ariac/Orders", 1000, orderCallback);

  //Subscribes to all bin cameras, agv cameras, and quality cameras
  ros::Subscriber cam_bin1 = n.subscribe("/ariac/logical_camera_bin1", 1000, bin1CamCallback);
  ros::Subscriber cam_bin2 = n.subscribe("/ariac/logical_camera_bin2", 1000, bin2CamCallback);
  ros::Subscriber cam_bin3 = n.subscribe("/ariac/logical_camera_bin3", 1000, bin3CamCallback);
  ros::Subscriber cam_bin4 = n.subscribe("/ariac/logical_camera_bin4", 1000, bin4CamCallback);
  ros::Subscriber cam_bin5 = n.subscribe("/ariac/logical_camera_bin5", 1000, bin5CamCallback);
  ros::Subscriber cam_bin6 = n.subscribe("/ariac/logical_camera_bin6", 1000, bin6CamCallback);
  ros::Subscriber cam_agv1 = n.subscribe("/ariac/logical_camera_agv1", 1000, agv1CamCallback);
  ros::Subscriber cam_agv2 = n.subscribe("/ariac/logical_camera_agv2", 1000, agv2CamCallback);
  ros::Subscriber cam_quality1 = n.subscribe("/ariac/quality_control_sensor_1", 1000, q1CamCallback);
  ros::Subscriber cam_quality2 = n.subscribe("/ariac/quality_control_sensor_2", 1000, q2CamCallback);

  //creates clients for start_competition and material_locations services
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  ros::ServiceClient mat_loc_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  

  my_bool_var.request.data = true;

  //call start_competition service
  service_call_succeeded = begin_client.call(begin_comp);
  if(!service_call_succeeded){
    ROS_ERROR("Competition service call failed");
    ros::shutdown();
  }
  if(begin_comp.response.success){
    ROS_INFO("Competiton service called successfully %s", begin_comp.response.message.c_str());
  }
  else{
    ROS_ERROR("Competition failed to start, service called");
  }

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
//    chatter_pub.publish(msg);
    cam_image_vector.clear();
    ros::spinOnce();


    //nested for loop over all orders, shipments, and products to find locations with material_location service
    product_type_vector.clear();
    product_bin_vector.clear();
    for (const auto& order : order_vector){
      get_products(&order);
    }
    
    //ROS_INFO prints the first product type and first bin it is found in
    ROS_INFO("The first product is of type : %s,  and is located in the bin : %s", product_type_vector.front(), product_bin_vector.front().front().unit_id);
    
  

    loop_rate.sleep();
    ++count;
  }


  return 0;
}