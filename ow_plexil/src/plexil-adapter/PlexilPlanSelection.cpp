// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "PlexilPlanSelection.h"
#include "OwExecutive.h"
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

void PlexilPlanSelection::initialize(std::string initial_plan)
{
  ROS_INFO("Starting PLEXIL executive node...");
  m_genericNodeHandle = std::make_unique<ros::NodeHandle>();

  // wait for the first proper clock message before running the plan
  ros::Rate warmup_rate(0.1);
  ros::Time begin = ros::Time::now();
  while (ros::Time::now() - begin == ros::Duration(0.0))
  {
    ros::spinOnce();
    warmup_rate.sleep();
  }

  // if launch argument plan is given we add it
  if(initial_plan.compare("None") != 0) {
    plan_array.push_back(initial_plan);
  }
  //workaround for the getPlanState not returning correctly for first plan
  m_first_plan = true;

  // assist passing the plan termination signal from autonomy to plexil executive
  // to make the current exit (a.k.a, terminated cleanly)
  m_terminate_plan = false;

  //initialize service
  m_planSelectionService = std::make_unique<ros::ServiceServer>
      (m_genericNodeHandle->
       advertiseService("/plexil_plan_selection",
       &PlexilPlanSelection::planSelectionServiceCallback, this));

  //initialize publisher
  m_planSelectionStatusPublisher = std::make_unique<ros::Publisher>
      (m_genericNodeHandle->advertise<std_msgs::String>
       ("/plexil_plan_selection_status", 20));

  m_PlanTerminatePublisher = std::make_unique<ros::Publisher>
      (m_genericNodeHandle->advertise<std_msgs::Bool>
       ("/autonomy/terminate_plan", 20));


  ROS_INFO("Executive node started, ready for PLEXIL plans.");
}

void PlexilPlanSelection::start()
{
  ros::Rate rate(10); // 10 Hz for overall rate we are spinning
  int length = 0;
  while(ros::ok()){
    //if we have plans in our plan array we begin the control loop
    if(plan_array.size() > 0){
      length = plan_array.size();
      for(int i = 0; i < length; i++){
	// set the terminal plan signal to false to avoid
	// existing the plan without running
	std_msgs::Bool msg;
	setPlanTerminationSignal(false);
	msg.data = getPlanTerminationSignal();
	m_PlanTerminatePublisher->publish(msg);

        //trys to run the current plan
        runCurrentPlan();
        //waits until plan finishes running
        waitForPlan();
        //checks if plan_array has been cleared
        if(plan_array.size() == 0){
          break;
        }
      }
    }
    //if no plans we spinonce and sleep before checking again
    ros::spinOnce();
    rate.sleep();
  }
}

void PlexilPlanSelection::waitForPlan(){
  std_msgs::String status;
  ros::Rate rate(10); // 10 Hz for overall rate we are spinning
  //wait for current plan to finish before running next plan
  while(!OwExecutive::instance()->getPlanState() && m_first_plan == false){
    ros::spinOnce();
    rate.sleep();
  }

  //Once plan is finished set status to complete for GUI
  status.data = "COMPLETE";
  m_planSelectionStatusPublisher->publish(status);


  // TBD: Publish the plan status (COMPLETE or TERMINATED) to the monitor node
  // Need the message, PlanStatus: {plan_name/task_name, status}
  if (getPlanTerminationSignal()){
    ROS_INFO("[Plan Finishes] %s is directly terminated by the autonomy.", getCurrentPlanName());
  }
  else{
    ROS_INFO("[Plan Finishes] %s finishes natually.", getCurrentPlanName());
  }
}

void PlexilPlanSelection::runCurrentPlan(){
  std_msgs::String status;
  ros::Rate rate(10); // 10 Hz for overall rate we are spinning

  // set the current plan name
  setCurrentPlanName(plan_array[0]);

  //try to run the plan
  if(OwExecutive::instance()->runPlan(plan_array[0].c_str())){
    //workaround for getPlanState not working on first plan
    if(m_first_plan == true){
      m_first_plan = false;
    }
    // Times out after 3 seconds or the plan is registered as running.
    int timeout = 0;
    while(OwExecutive::instance()->getPlanState() && timeout < 30){
      ros::spinOnce();
      rate.sleep();
      timeout+=1;
      if(timeout % 10 == 0){
        ROS_ERROR("Plan not responding, timing out in %i seconds", (3 - timeout/10));
      }
    }
      //if timed out we set plan as failed for GUI
      if(timeout == 30){
        ROS_INFO ("Plan timed out, try again.");
        status.data = "FAILED:" + plan_array[0];
        m_planSelectionStatusPublisher->publish(status);
      }
      //otherwise we set it as running
      else{
          status.data = "SUCCESS:" + plan_array[0];
          m_planSelectionStatusPublisher->publish(status);
      }
  }
  //if error from run() we set as failed for GUI
  else{
      status.data = "FAILED:" + plan_array[0];
      m_planSelectionStatusPublisher->publish(status);
  }
  //delete the plan we just ran from plan array
  plan_array.erase(plan_array.begin());
}




bool PlexilPlanSelection::planSelectionServiceCallback(ow_plexil::PlanSelection::Request &req,
                                                       ow_plexil::PlanSelection::Response &res)
{
  //if command is ADD we add given plans to the plan_array
  if(req.command.compare("ADD") == 0){
    plan_array.insert(plan_array.end(), req.plans.begin(), req.plans.end());
    res.success = true;
  }
  //if command is RESET  delete all plans in the plan_array
  else if(req.command.compare("RESET") == 0){
    plan_array.clear();
    ROS_INFO ("Plan list cleared, current plan will finish execution before stopping");
    res.success = true;
  }
  // if command is SUSPEND, suspend the plexil executive app.
  // The SUSPEND command's intent is to suspend the current executing plan
  else if(req.command.compare("SUSPEND") == 0){
   std::string plan_name = req.plans[0]; // There is only one plan in the array of plans
   if(OwExecutive::instance()->suspendExec()){
     ROS_INFO("The suspension of the current plan (%s): Success", plan_name.c_str());
     res.success = true;
   }
   else{
     ROS_INFO("The suspension of the current plan (%s): Failure", plan_name.c_str());
     res.success = false;
   }
  }
  // if command is RESUME, suspend the plexil executive app.
  // The RESUME command's intent is to resume the currently suspended plan
  else if(req.command.compare("RESUME") == 0){
   std::string plan_name = req.plans[0]; // There is only one plan in the array of plans
   if(OwExecutive::instance()->resumeExec()){
     ROS_INFO("The resume of the current plan (%s): Success", plan_name.c_str());
     res.success = true;
   }
   else{
     ROS_INFO("The resume of the current plan (%s): Failure", plan_name.c_str());
     res.success = false;
   }
  }
  // if the command is TERMINATE, it terminates the currently executing plan by
  // relaying the signal to the rostopic, /autonomy/terminate_plan, which is subscribed
  // by the adapter to update the PLEXIL Lookup TerminatePlan in the PLEXIL plan.
  // The TerminatePlan Lookup is used as the ExitCondition in the PLEXIL plan.
  else if(req.command.compare("TERMINATE") == 0){
   std_msgs::Bool msg;
   setPlanTerminationSignal(true);
   msg.data = getPlanTerminationSignal();
   m_PlanTerminatePublisher->publish(msg);

   std::string plan_name = req.plans[0]; // There is only one plan in the array of plans
   ROS_INFO("The current plan (%s) is terminating...", plan_name.c_str());
   res.success = true; // always true here.
  }
  else{
    ROS_ERROR("Command %s not recognized", req.command.c_str());
    res.success = false;
  }
  return true;
}


void PlexilPlanSelection::setPlanTerminationSignal(bool value)
{
  m_terminate_plan = value;
}

bool PlexilPlanSelection::getPlanTerminationSignal()
{
  return m_terminate_plan;
}

void PlexilPlanSelection::setCurrentPlanName(std::string value)
{
  m_current_plan_name = value;
}

std::string PlexilPlanSelection::getCurrentPlanName()
{
  return m_current_plan_name;
}
