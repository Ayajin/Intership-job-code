#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <string>

#include "cpp_from_python_bicycle/control_node.h"
#include "cpp_from_python_bicycle/SetModelInput.h"

float angle_resolution = 0.1;
float throtle_resolution = 0.01;
float throttle = 0.0;
float steering = 0.0;
float throttle_new;
float steering_new;

int main(int argc, char** argv) {
  // init node
  ros::init(argc, argv, "control_node");
  ros::NodeHandle n;

  // subscribe arrow key
  ros::Subscriber sub = n.subscribe("arrow_string", 10, callbackKeyInput);

  // init publisher
  ros::Publisher pub_throttle =
    n.advertise<std_msgs::Float64>("/control_input/throtle", 10);

  ros::Publisher pub_steering =
    n.advertise<std_msgs::Float64>("/control_input/steering_angle", 10);

  ros::spin();

  return 0;
}

void calculateInput(float dthrottle, float dsteering) {
  // declare msg to publish
  std_msgs::Float64 msg_throttle;
  std_msgs::Float64 msg_steering;

  // calculate input
  throttle_new = throttle + dthrottle;
  steering_new = steering + dsteering;

  // init service client & publisher
  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<cpp_from_python_bicycle::SetModelInput>(
      "bicycle_model_srvs");
  ros::Publisher pub_throttle =
    n.advertise<std_msgs::Float64>("/control_input/throtle", 10);
  ros::Publisher pub_steering =
    n.advertise<std_msgs::Float64>("/control_input/steering_angle", 10);

  // ready to request service
  cpp_from_python_bicycle::SetModelInput srvs;
  srvs.request.throttle = throttle_new;
  srvs.request.steering_angle = steering_new;

  // publish throttle & steering when act in limitation degree
  if (client.call(srvs)) {
    throttle = throttle_new;
    msg_throttle.data = throttle;
    pub_throttle.publish(msg_throttle);

    steering = steering_new;
    msg_steering.data = steering;
    pub_steering.publish(msg_steering);
  } else {
    ROS_INFO("limitation_degree");
  }
}

void callbackKeyInput(const std_msgs::String::ConstPtr& msg) {
  // receive arrow key & act
  std::string input_key = msg->data.c_str();

  if (input_key == "up")
    calculateInput(throtle_resolution, 0.0);

  else if (input_key == "down")
    calculateInput(-1.0 * throtle_resolution, 0.0);

  else if (input_key == "right")
    calculateInput(0.0, angle_resolution);

  else if (input_key == "left")
    calculateInput(0.0, -1.0 * angle_resolution);
}