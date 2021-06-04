// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// OW autonomy ROS node.

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>

// OW
#include "OwExecutive.h"
#include "OwInterface.h"

// C++: running a bash script
#include <stdlib.h>
#include <string.h>

void newPlanSubCallback(const std_msgs::Bool new_plan)
{
  if (new_plan.data) {
    ROS_INFO("A new plan is ready for execution.");
    // Execute the new PLEXIL plan
  }
}

int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "autonomy_node");

  //ros::NodeHandle nh;
  //ros::Subscriber new_plan_sub = nh.subscribe<std_msgs::Bool>("/NewPlan", 3, newPlanSubCallback);

  if (! OwExecutive::instance()->initialize()) {
    ROS_ERROR("Could not initialize OW executive, shutting down.");
    return 1;
  }

  OwInterface::instance()->initialize();

  // wait for the first proper clock message before running the plan
  ros::Rate warmup_rate(0.1);
  ros::Time begin = ros::Time::now();
  while (ros::Time::now() - begin == ros::Duration(0.0))
  {
    ros::spinOnce();
    warmup_rate.sleep();
  }
  
  /*
  // Run the specified plan
  if (argc == 2) {
    ROS_INFO ("Running plan %s", argv[1]);
    if (strcmp(argv[1], "Exca_Fault.plx") == 0) {
      system("/home/jsu/Projects/oceanwaters_ws/src/ow_autonomy/syn-plan/fault_case.bash");
    } else if (strcmp(argv[1], "Exca_NoFault.plx") == 0) {
      system("/home/jsu/Projects/oceanwaters_ws/src/ow_autonomy/syn-plan/no_fault_case.bash");
    }
    OwExecutive::instance()->runPlan (argv[1]); // asynchronous
  }
  else {
    ROS_ERROR("autonomy_node got %i args, expected 2", argc);
    return 1;
  }
  */

  // ROS Loop (runs concurrently with plan).  Note that once this loop starts,
  // this function (and node) is terminated with an interrupt.

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    ros::spinOnce();

    // Check if a new plan is ready
    // If it is ready, make sure the current plan is stopped or finished,
    // then start to execute the new plan;
    // Otherwise, let the current plan to continue
    if (OwInterface::instance()->getHasNewPlan()) {
      // ToDo: ensure the currently executing plan is
      //       finised or stopped before running the new plan.
      OwExecutive::instance()->stopResetExec();
      OwExecutive::instance()->restartExecAppInterface();

      std::string plan_name = OwInterface::instance()->getPlanName();
      ROS_INFO("[Autonomy Node] A new plan (%s) is ready for execution.", plan_name.c_str());
      // Execute the new PLEXIL plan asynchronously
      std::string plan_filename = plan_name + std::string(".plx");
      OwExecutive::instance()->runPlan (plan_filename.c_str()); // asynchronous

      OwInterface::instance()->setHasNewPlan(false);
    }


    rate.sleep();
  }

  return 0;  // We never actually get here!
}
