// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Plexil_Planner_H
#define Plexil_Planner_H

#include <ros/ros.h>
#include <ow_plexil/PlanSelection.h>
#include <ow_plexil/CurrentPlan.h>

class PlexilPlanSelection{
  public:
    PlexilPlanSelection() = default;
    ~PlexilPlanSelection() = default;
    void initialize(std::string initial_plan);
    void start();

    void setPlanTerminationSignal(bool value);
    bool getPlanTerminationSignal();

    void setCurrentPlanName(std::string value);
    std::string getCurrentPlanName();

    void setCurrentPlanStatus(std::string value);
    std::string getCurrentPlanStatus();

    void publishChangedPlexilPlanStatus(std::string new_status);

  private:
    PlexilPlanSelection(const PlexilPlanSelection&) = delete;
    PlexilPlanSelection& operator = (const PlexilPlanSelection&) = delete;
    bool planSelectionServiceCallback(ow_plexil::PlanSelection::Request&,
                                      ow_plexil::PlanSelection::Response&);
    void runCurrentPlan();
    void waitForPlan();

    std::unique_ptr<ros::NodeHandle> m_genericNodeHandle;
    std::unique_ptr<ros::ServiceServer> m_planSelectionService;
    std::unique_ptr<ros::Publisher> m_planSelectionStatusPublisher;
    std::vector<std::string> plan_array;
    bool m_first_plan;

    // It indicates whether the current plan should be terminated or not
    // It is just used to assist passing the plan termination signal from autonomy
    std::unique_ptr<ros::Publisher> m_PlanTerminatePublisher;
    bool m_terminate_plan = false;

    std::string m_current_plan_name = "";
    std::string m_current_plan_status = "";

    std::unique_ptr<ros::Publisher> m_planStatusPublisher;
    std::unique_ptr<ros::Subscriber> m_planStatusSubscriber;
    void currentPlanCallback(const ow_plexil::CurrentPlan msg);
};

#endif

