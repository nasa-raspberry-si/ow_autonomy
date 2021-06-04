// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// Planner ROS node.

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <ow_autonomy/NewPlan.h>
#include <ow_autonomy/PlanStatus.h>

// C++: running a bash script
#include <stdlib.h>
#include <string.h>

bool has_new_plan = false;

std::string plan_name = "Exca";
std::string new_plan_name = "Exca1";
int new_plan_numeric_id = 1;

void planning(std::string plan_name)
{
    // NOTE: this is the best we can do for now.
    std::string synPlanDir = ros::package::getPath("ow_autonomy") + "/syn-plan/";

    std::string cmd = synPlanDir + "/run.bash";
    std::string args = plan_name;
    std::string cmd_args = cmd + " " + args;
    system(cmd_args.c_str());
}

void planStatusSubCallback(const ow_autonomy::PlanStatus plan_status)
{
  // Currently, a 'true' value of 'plan_status' indicates a successful execution,
  // while a 'false' value indicates a failure, which requires a replanning.
  // ToDo: the message 'plan_status' will be customized to provided more information
  //       for supporting a complex planner in future.
  // Replanning: only excavation scenario is supported for now.
  if (!plan_status.plan_status) { // plan execution failed. a new plan is needed.
    ROS_INFO ("[Planner Node : PlanStatusSubCallback] %s Plan failed", plan_status.plan_name.c_str());
    new_plan_numeric_id++;
    new_plan_name = plan_name + std::to_string(new_plan_numeric_id);
    planning(new_plan_name);
   //system("/home/jsu/Projects/oceanwaters_ws/src/ow_autonomy/syn-plan/run.bash"); 

    ROS_INFO ("[Planner Node : planStatusSubCallabck] Re-planning is done");
    has_new_plan = true;
  } else {
    ROS_INFO ("[Planner Node : planStatusSubCallabck] Re-planning is not needed.");
  }

}

int main(int argc, char* argv[])
{
  // Initializations

  ros::init(argc, argv, "planner_node");

  ros::NodeHandle nh;

  ros::Publisher new_plan_pub = nh.advertise<ow_autonomy::NewPlan>("/NewPlan", 3);
  ros::Subscriber plan_status_sub = nh.subscribe<ow_autonomy::PlanStatus>("/PlexilPlanExecutionStatus",
                                                         3,
							 planStatusSubCallback);
  // For the initial plan
  planning(new_plan_name);
  has_new_plan = true;

  ros::Rate rate(1); // 1 Hz seems appropriate, for now.
  while (ros::ok()) {
    if (has_new_plan) {
      ow_autonomy::NewPlan new_plan_msg;
      new_plan_msg.has_new_plan = has_new_plan;
      new_plan_msg.plan_name = new_plan_name;
      //ROS_INFO ("[Planner] plan name: %s", new_plan_name.c_str());

      new_plan_pub.publish(new_plan_msg);

      has_new_plan = false;	
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;  // We never actually get here!
}
