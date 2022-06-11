//
// Created by ksals on 2022/6/11.
//

#include "mouse_callback/mouse_callback.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mouse_callback_node");
  ros::NodeHandle nh("~");
  MouseCB mouse(nh);
  return 0;
}