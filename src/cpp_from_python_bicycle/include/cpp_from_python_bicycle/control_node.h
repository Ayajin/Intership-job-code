#ifndef CONTROL_NODE_H
#define CONTROL_NODE_H

void calculateInput(float dthrottle, float dsteering);
void callbackKeyInput(const std_msgs::String::ConstPtr& msg);

#endif