#include <ros/ros.h>

// #include <mutex>
#include <string>

#include "cpp_from_python_bicycle/bicycle_marker.h"

int main(int argc, char **argv) {
  BicycleMarker *markerptr = new BicycleMarker();

  ros::init(argc, argv, "marker_publisher");
  ros::NodeHandle n;

  ros::Publisher pub =
    n.advertise<visualization_msgs::MarkerArray>("bicycle_marker", 10);
  ros::Subscriber sub_throtle = n.subscribe(
    "/control_input/throtle", 10, &BicycleMarker::callbackThrottle, markerptr);
  ros::Subscriber sub_steering =
    n.subscribe("/control_input/steering_angle", 10,
                &BicycleMarker::callbackSteering, markerptr);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    markerptr->updateMarkers();
    pub.publish(markerptr->marray);

    // mutex_throttle.lock();
    // throttle = 0.f;
    // mutex_throttle.unlock();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

// Source Code of BicycleMarker Class

BicycleMarker::BicycleMarker() {
  this->throttle = 0.0;
  this->steering_angle = 0.0;
}

void BicycleMarker::callbackThrottle(const std_msgs::Float64::ConstPtr &msg) {
  // 1.
  // mutex_throttle.lock();
  this->throttle = msg->data;
  // mutex_throttle.unlock();

  // 2.
  // std::lock_guard<std::mutex> guard(mutex_throttle_);
  // this->throttle = msg->data;
}

void BicycleMarker::callbackSteering(const std_msgs::Float64::ConstPtr &msg) {
  this->steering_angle = msg->data;
}

void BicycleMarker::updateMarkers() {
  float wheel_size = 1.2;
  auto current_time = ros::Time::now();

  // make string(info)
  std::string info = "[bicycle info]\n - steering angle: ";
  info.append(std::to_string(this->steering_angle));
  info.append("deg\n - throttle: ");
  info.append(std::to_string(this->throttle));

  marray.markers.clear();

  visualization_msgs::Marker bar;
  bar.header.frame_id = "body_frame";
  bar.header.stamp = current_time;
  bar.id = 0;
  bar.type = visualization_msgs::Marker::CUBE;
  bar.action = visualization_msgs::Marker::ADD;
  bar.scale.x = 2.0;
  bar.scale.y = 0.02;
  bar.scale.z = 0.2;
  bar.color.a = 0.8;
  bar.color.r = 0.5;
  bar.color.g = 1.0;
  bar.color.b = 0.5;
  bar.pose.orientation.w = 1.0;
  bar.pose.position.z = wheel_size / 2;
  this->marray.markers.push_back(bar);

  visualization_msgs::Marker front_wheel;
  front_wheel.header.frame_id = "front_whl_frame";
  front_wheel.header.stamp = current_time;
  front_wheel.id = 1;
  front_wheel.type = visualization_msgs::Marker::SPHERE;
  front_wheel.action = visualization_msgs::Marker::ADD;
  front_wheel.scale.x = wheel_size;
  front_wheel.scale.y = 0.1;
  front_wheel.scale.z = wheel_size;
  front_wheel.color.a = 0.8;
  front_wheel.color.r = 1.0;
  front_wheel.color.g = 0.5;
  front_wheel.color.b = 0.0;
  front_wheel.pose.orientation.w = 1.0;
  front_wheel.pose.position.z = wheel_size / 2;
  this->marray.markers.push_back(front_wheel);

  visualization_msgs::Marker back_wheel;
  back_wheel.header.frame_id = "rear_whl_frame";
  back_wheel.header.stamp = current_time;
  back_wheel.id = 2;
  back_wheel.type = visualization_msgs::Marker::SPHERE;
  back_wheel.action = visualization_msgs::Marker::ADD;
  back_wheel.scale.x = wheel_size;
  back_wheel.scale.y = 0.1;
  back_wheel.scale.z = wheel_size;
  back_wheel.color.a = 0.8;
  back_wheel.color.r = 1.0;
  back_wheel.color.g = 0.5;
  back_wheel.color.b = 0.0;
  back_wheel.pose.orientation.w = 1.0;
  back_wheel.pose.position.z = wheel_size / 2;
  this->marray.markers.push_back(back_wheel);

  visualization_msgs::Marker text;
  text.header.frame_id = "body_frame";
  text.header.stamp = current_time;
  text.id = 3;
  text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::Marker::ADD;
  text.scale.x = text.scale.y = text.scale.z = 0.5;
  text.color.a = 1.0;
  text.color.r = 1.0;
  text.color.g = 1.0;
  text.color.b = 1.0;
  text.pose.orientation.w = 1.0;
  text.pose.position.z = wheel_size;

  text.text = (info);

  this->marray.markers.push_back(text);
}