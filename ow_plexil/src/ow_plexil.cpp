// ROS
#include <ros/ros.h>
#include <ros/package.h>


int main()
{
  // This source is to facility building ow_plexil package that helps provide the message
  // header CurrentTask.h to rs_autonomy package
  // Otherwise, find_package() can not find the package ow_plexil due that there is no
  // target is built under the name, ow_plexil.
  return 0;
}

