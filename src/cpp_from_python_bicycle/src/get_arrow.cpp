#include <ros/ros.h>
#include <std_msgs/String.h>

#include "cpp_from_python_bicycle/get_arrow.h"

int main(int argc, char **argv) {
  try{
    // init screen
    initscr();
    keypad(stdscr, TRUE);

    // execute publisher node
    get_arrow_and_pub(argc, argv);
  }
  catch(...){
    // clean up when shut down
    keypad(stdscr, FALSE);
    clear();
    endwin();
    
    return -1;
  }

  return 0;
}

void get_arrow_and_pub(int argc, char **argv) {
  // init node & publisher
  ros::init(argc, argv, "get_arrow_and_pub");
  ros::NodeHandle n;
  ros::Publisher arrow_pub =
    n.advertise<std_msgs::String>("arrow_string", 1000);
  std_msgs::String msg;

  int input;  // input of key

  mvaddstr(0, 0,
           "Move with Arrow Key.\nAfter press Ctrl+c, Press any key to close "
           "terminal");

  ros::Rate loop_rate(100);

  while (ros::ok()) {
    input = getch();

    // get arrow key & publish key
    if (input == KEY_RIGHT)
      msg.data = "right";  // key right

    else if (input == KEY_LEFT)
      msg.data = "left";  // key left

    else if (input == KEY_UP)
      msg.data = "up";  // key up

    else if (input == KEY_DOWN)
      msg.data = "down";  // key down

    arrow_pub.publish(msg);

    loop_rate.sleep();
  }
}