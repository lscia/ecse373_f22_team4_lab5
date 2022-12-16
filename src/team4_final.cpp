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

//Geometry Includes
#include "geometry_msgs/Pose.h"

//CPP Includes
#include <sstream>
#include <vector>
#include <algorithm>
#include <array>




//STORAGE OF DATA

//Order vector
std::vector<osrf_gear::Order> orders;


//Logical Cam Arrays
osrf_gear::LogicalCameraImage cam_bins[6];
osrf_gear::LogicalCameraImage cam_agvs[2];
osrf_gear::LogicalCameraImage cam_qual[2];

//Joint States Storage





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




/* Gets all shipments in an order
* Inputs: Pointer to Order, Pointer to Shipment Vector
* Outputs length of shipment vector as integer
* Clears the shipments vector, fills it with shipments
* from given Order
*/
int getShipments(osrf_gear::Order &o, std::vector<osrf_gear::Shipment> &s)
{
  s.clear();
  if(!o.shipments.empty()){
    for (const auto& shipment : o.shipments){
      s.push_back(shipment);
    }
    return o.shipments.size();
  }
  else{
    return 0;
  }
}

/* Gets all products in a shipment
* Inputs: Pointer to Shipment, Pointer to Product Vector
* Outputs length of Product vector as integer
* Clears the Products vector, fills it with Products
* from given Shipment
*/
int getProducts(osrf_gear::Shipment &s, std::vector<osrf_gear::Product> &p)
{
  p.clear();
  if(!s.products.empty()){
    for (const auto& product : s.products){
      p.push_back(product);
    }
    return s.products.size();
  }
  else{
    return 0;
  }
}


/* Prints out all Orders, Shipments, and Products
* Inputs: None
* Outputs total number of products
*/
void allProducts()
{
  
}



/*
/* Finds the pose of a desired product type from a camera image
* Inputs: Camera #, pointer to product
* Outputs the pose with respect to the camera
*
int findProductBin(osrf_gear::Product& p)
{
}
*/




int main(int argc, char **argv)
{

  //Clears all vectors
  orders.clear();


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
  ros::init(argc, argv, "team4_final");



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




   
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);


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

  //Service to start comp
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  
  //Service to get material locations
  ros::ServiceClient mat_loc_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  

  ros::Rate loop_rate(10);



  
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
  



  
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {

    

      /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    //chatter_pub.publish(msg);


    

    //Iterate through all orders
    if(!orders.empty()){
    for(const auto& order : orders)
    {
      ROS_INFO("ORDER ID: [%s]", order.order_id.c_str());
      ROS_INFO("-------------");
      if(!order.shipments.empty())
      {
        //Iterate through all shipments in order
        for(const auto& shipment : order.shipments)
        {
          ROS_INFO("SHIPMENT ID: [%s]", shipment.shipment_type.c_str());
          ROS_INFO("-------------");
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
              
              ROS_INFO("PRODUCT TYPE: [%s] IS IN BIN: [%d]", product.type.c_str(), n);



              //Find the camera for that bin
              osrf_gear::LogicalCameraImage bin_image = cam_bins[n];
              geometry_msgs::Pose product_pose_wrt_camera;

              //Find all the materials from that cameras view
              for(const auto& model : bin_image.models)
              {
                //Get the pose of the product from that camera image
                if(model.type.compare(product.type) == 0)
                {
                  product_pose_wrt_camera = model.pose;
                }
              }

              ROS_INFO("PRODUCT IS AT x: %d y: %d z: %d WRT CAMERA", product_pose_wrt_camera.position.x, product_pose_wrt_camera.position.y, product_pose_wrt_camera.position.z);
              
              //Get the transform from that camera to the arm
              


              //Apply the transform to get the goal pose


              //Send the goal pose to IK


              //filter IK results


              //Command arm to move to goal pose










            }
          }
        }
      }
    }
  }
  
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
