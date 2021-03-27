#ifndef BICYCLE_MODEL_H
#define BICYCLE_MODEL_H

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "cpp_from_python_bicycle/SetModelInput.h"

// bicycle model is imported from
// https://github.com/winstxnhdw/KinematicBicycleModel/blob/main/kinematic_model.py

class BicycleModel {
private:
  float x, y, yaw, vx, vy, omega;
  float x_dot, y_dot;
  float throttle, delta;

  float dt;
  float L, Lf, Lr;

  float c_r, c_a;
  float max_steer, max_throtle;

public:
  geometry_msgs::TransformStamped body_t;
  geometry_msgs::TransformStamped frontwheel_t;
  geometry_msgs::TransformStamped rearwheel_t;

  BicycleModel(float x = 0.0, float y = 0.0, float yaw = 0.0, float vx = 0.0,
               float vy = 0.0, float omega = 0.0);
  float clip(float n, float lower, float upper);
  bool setInput(float throttle, float steering_angle);
  void kinematic_model();
  void initTf();
  void broadcastTf();
  float normalise_angle(float angle);
  bool serviceBicycleModel(
    cpp_from_python_bicycle::SetModelInput::Request &req,
    cpp_from_python_bicycle::SetModelInput::Response &res);
};

#endif