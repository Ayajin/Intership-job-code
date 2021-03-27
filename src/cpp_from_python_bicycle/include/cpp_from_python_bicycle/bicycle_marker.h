#ifndef BICYCLE_MARKER_H
#define BICYCLE_MARKER_H

#include "std_msgs/Float64.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

class BicycleMarker {
private:
  float throttle;
  float steering_angle;
  // std::mutex mutex_throttle;
  // std::mutex mutex_steering_angle;
public:
  visualization_msgs::MarkerArray marray;
public:
  BicycleMarker();
  void callbackThrottle(const std_msgs::Float64::ConstPtr &msg);
  void callbackSteering(const std_msgs::Float64::ConstPtr &msg);
  void updateMarkers();
};

#endif