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

//osrf_gear stuff
std::vector<osrf_gear::LogicalCameraImage> cam_image_vector;

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

void camCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  cam_image_vector.push_back(*msg);
}

//main loop from example wiki code for publisher/subscriber
int main(int argc, char **argv)
{
  //clear vectors
  order_vector.clear();
  product_bin_vector.clear();
  product_type_vector.clear();
  cam_image_vector.clear();

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
  ros::Subscriber cam_bin1 = n.subscribe("/ariac/logical_camera_bin1", 1000, camCallback);
  ros::Subscriber cam_bin2 = n.subscribe("/ariac/logical_camera_bin2", 1000, camCallback);
  ros::Subscriber cam_bin3 = n.subscribe("/ariac/logical_camera_bin3", 1000, camCallback);
  ros::Subscriber cam_bin4 = n.subscribe("/ariac/logical_camera_bin4", 1000, camCallback);
  ros::Subscriber cam_bin5 = n.subscribe("/ariac/logical_camera_bin5", 1000, camCallback);
  ros::Subscriber cam_bin6 = n.subscribe("/ariac/logical_camera_bin6", 1000, camCallback);
  ros::Subscriber cam_agv1 = n.subscribe("/ariac/logical_camera_agv1", 1000, camCallback);
  ros::Subscriber cam_agv2 = n.subscribe("/ariac/logical_camera_agv2", 1000, camCallback);
  ros::Subscriber cam_quality1 = n.subscribe("/ariac/quality_control_sensor_1", 1000, camCallback);
  ros::Subscriber cam_quality2 = n.subscribe("/ariac/quality_control_sensor_2", 1000, camCallback);

  //creates clients for services
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  ros::ServiceClient material_location = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  

  my_bool_var.request.data = true;

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
      for (const auto& shipment : order.shipments){
        for (const auto& product : shipment.products){
          //adds the product type to product_type_vector, and the vector of bins it can be found in to product_bin_vector
          product_type_vector.push_back(product.type);
          osrf_gear::GetMaterialLocations material_locations;
          material_locations.request.material_type=product.type;
          material_location.call(material_locations);
          product_bin_vector.push_back(material_locations.response.storage_units);

          std::vector<osrf_gear::Model> cam_image_models;
          geometry_msgs::Pose product_pose;
          cam_image_models.clear();

          switch(material_locations.response.storage_units.front().unit_id){
            case "bin1" :
              cam_image_models.assign(cam_image_vector.at(0).models);
              for (const auto& model : cam_image_models){
                if(model.type.compare(product.type) == 0){
                  product_pose = model.pose;
                  break;
                }
              }
              ROS_INFO("Product: %s Bin: bin1 Pose:%s", product.type, product_pose)
              break;

              case "bin2" :
              cam_image_models.assign(cam_image_vector.at(1).models);
              for (const auto& model : cam_image_models){
                if(model.type.compare(product.type) == 0){
                  product_pose = model.pose;
                  break;
                }
              }
              ROS_INFO("Product: %s Bin: bin2 Pose:%s", product.type, product_pose)
              break;

              case "bin3" :
              cam_image_models.assign(cam_image_vector.at(2).models);
              for (const auto& model : cam_image_models){
                if(model.type.compare(product.type) == 0){
                  product_pose = model.pose;
                  break;
                }
              }
              ROS_INFO("Product: %s Bin: bin3 Pose:%s", product.type, product_pose)
              break;

              case "bin4" :
              cam_image_models.assign(cam_image_vector.at(3).models);
              for (const auto& model : cam_image_models){
                if(model.type.compare(product.type) == 0){
                  product_pose = model.pose;
                  break;
                }
              }
              ROS_INFO("Product: %s Bin: bin4 Pose:%s", product.type, product_pose)
              break;

              case "bin5" :
              cam_image_models.assign(cam_image_vector.at(4).models);
              for (const auto& model : cam_image_models){
                if(model.type.compare(product.type) == 0){
                  product_pose = model.pose;
                  break;
                }
              }
              ROS_INFO("Product: %s Bin: bin5 Pose:%s", product.type, product_pose)
              break;

              case "bin6" :
              cam_image_models.assign(cam_image_vector.at(5).models);
              for (const auto& model : cam_image_models){
                if(model.type.compare(product.type) == 0){
                  product_pose = model.pose;
                  break;
                }
              }
              ROS_INFO("Product: %s Bin: bin6 Pose:%s", product.type, product_pose)
              break;
          }
        }
      }
    }
    //ROS_INFO prints the first product type and first bin it is found in
    ROS_INFO("The first product is of type : %s,  and is located in the bin : %s", product_type_vector.front(), product_bin_vector.front().front().unit_id);

  

    loop_rate.sleep();
    ++count;
  }


  return 0;
}