#include <ros/ros.h>
#include <cmath>

#include "cpp_from_python_bicycle/bicycle_model.h"

int main(int argc, char **argv) {
  // make bicyclemodle object
  BicycleModel *modelptr = new BicycleModel();

  // init node
  ros::init(argc, argv, "bicycle_model_service");
  ros::NodeHandle n;

  // it need to serve service
  ros::ServiceServer service = n.advertiseService(
    "bicycle_model_srvs", &BicycleModel::serviceBicycleModel, modelptr);

  // initialize model's tf
  modelptr->initTf();

  ros::Rate looprate(100);

  while (ros::ok()) {
    modelptr->kinematic_model();
    modelptr->broadcastTf();

    ros::spinOnce();

    looprate.sleep();
  }
}

// Source Code of BicycleModel Class

// constructer, it initialize value that need for calculate bicycle model
BicycleModel::BicycleModel(float x, float y, float yaw, float vx, float vy,
                           float omega) {
  this->x = x;
  this->y = y;
  this->yaw = yaw;
  this->vx = vx;
  this->vy = vy;
  this->omega = omega;

  this->x_dot = 0.0;
  this->y_dot = 0.0;

  this->throttle = 0.0;
  this->delta = 0.0;

  this->dt = 0.01;

  this->L = 2.0;
  this->Lf = 1.0;
  this->Lr = L - Lf;

  this->c_r = 0.0;
  this->c_a = 0.0;
  this->max_steer = M_PI / 3;
  this->max_throtle = 1.0;
}

// function that act like np.clip
float BicycleModel::clip(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

// limit steering_angle & throttle
bool BicycleModel::setInput(float throttle, float steering_angle) {
  if (abs(steering_angle) >= this->max_steer) return false;
  if (abs(throttle) >= this->max_throtle) return false;

  this->throttle = throttle;
  this->delta = -steering_angle;

  return true;
}

// kinematic_model of bicycle model
void BicycleModel::kinematic_model() {
  // Compute the local velocity in the x-axis
  float f_load = this->vx * (this->c_r + this->c_a * this->vx);
  this->vx += this->dt * (this->throttle - f_load);

  // Compute radius and angular velocity of the kinematic bicycle model
  this->delta = this->clip(this->delta, -this->max_steer, this->max_steer);

  if (this->delta == 0.0)
    this->omega = 0.0;

  else {
    float R = this->L / tan(this->delta);
    this->omega = this->vx / R;
  }

  // Compute the state change rate
  this->x_dot = this->vx * cos(this->yaw);
  this->y_dot = this->vx * sin(this->yaw);

  // Compute the final state using the discrete time model
  this->x += this->x_dot * this->dt;
  this->y += this->y_dot * this->dt;
  this->yaw += this->omega * this->dt;
  this->yaw = this->normalise_angle(this->yaw);
}

// code of init frame to world
void BicycleModel::initTf() {
  tf2::Quaternion q;

  this->body_t.header.frame_id = "world";
  this->body_t.child_frame_id = "body_frame";
  this->body_t.transform.translation.x = this->x;
  this->body_t.transform.translation.y = this->y;
  this->body_t.transform.translation.z = 0.0;
  q.setRPY(0, 0, this->yaw);
  this->body_t.transform.rotation.x = q.x();
  this->body_t.transform.rotation.y = q.y();
  this->body_t.transform.rotation.z = q.z();
  this->body_t.transform.rotation.w = q.w();

  this->frontwheel_t.header.frame_id = "body_frame";
  this->frontwheel_t.child_frame_id = "front_whl_frame";
  this->frontwheel_t.transform.translation.x = this->Lf;
  this->frontwheel_t.transform.translation.y = 0.0;
  this->frontwheel_t.transform.translation.z = 0.0;
  q.setRPY(0, 0, this->delta);
  this->frontwheel_t.transform.rotation.x = q.x();
  this->frontwheel_t.transform.rotation.y = q.y();
  this->frontwheel_t.transform.rotation.z = q.z();
  this->frontwheel_t.transform.rotation.w = q.w();

  this->rearwheel_t.header.frame_id = "body_frame";
  this->rearwheel_t.child_frame_id = "rear_whl_frame";
  this->rearwheel_t.transform.translation.x = -this->Lr;
  this->rearwheel_t.transform.translation.y = 0.0;
  this->rearwheel_t.transform.translation.z = 0.0;
  q.setRPY(0, 0, 0);
  this->rearwheel_t.transform.rotation.x = q.x();
  this->rearwheel_t.transform.rotation.y = q.y();
  this->rearwheel_t.transform.rotation.z = q.z();
  this->rearwheel_t.transform.rotation.w = q.w();
}

// code of broadcast frame to world
void BicycleModel::broadcastTf() {
  static tf2_ros::TransformBroadcaster br;
  tf2::Quaternion q;

  // body(update posiont, yaw angle and timestamp)
  this->body_t.header.stamp = ros::Time::now();
  this->body_t.transform.translation.x = this->x;
  this->body_t.transform.translation.y = this->y;
  q.setRPY(0, 0, this->yaw);
  this->body_t.transform.rotation.x = q.x();
  this->body_t.transform.rotation.y = q.y();
  this->body_t.transform.rotation.z = q.z();
  this->body_t.transform.rotation.w = q.w();

  br.sendTransform(this->body_t);

  // front wheel (update steering angle and timestamp)
  this->frontwheel_t.header.stamp = ros::Time::now();
  q.setRPY(0, 0, this->delta);
  this->frontwheel_t.transform.rotation.x = q.x();
  this->frontwheel_t.transform.rotation.y = q.y();
  this->frontwheel_t.transform.rotation.z = q.z();
  this->frontwheel_t.transform.rotation.w = q.w();

  br.sendTransform(this->frontwheel_t);

  // rear wheel (only update timestamp)
  this->rearwheel_t.header.stamp = ros::Time::now();

  br.sendTransform(this->rearwheel_t);
}

// if angle over 2pi, normalise
float BicycleModel::normalise_angle(float angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;

  while (angle < -M_PI) angle += 2.0 * M_PI;

  return angle;
}

// serve service when control node request service
bool BicycleModel::serviceBicycleModel(
  cpp_from_python_bicycle::SetModelInput::Request &req,
  cpp_from_python_bicycle::SetModelInput::Response &res) {
  ROS_INFO("request: x=%f, y=%f", req.throttle, req.steering_angle);
  if (this->setInput(req.throttle, req.steering_angle))
    return true;
  else
    return false;
}