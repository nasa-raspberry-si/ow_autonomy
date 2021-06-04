// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Interface_H
#define Ow_Interface_H

// Interface to lander simulator.  Singleton, because only once instance will
// ever be needed in the current autonomy scheme, which has one autonomy
// executive per lander.

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ow_autonomy/GuardedMoveAction.h>
#include <control_msgs/JointControllerState.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <string>

#include <ow_autonomy/NewPlan.h>

using GuardedMoveActionClient = actionlib::SimpleActionClient<ow_autonomy::GuardedMoveAction>;

class OwInterface
{
 public:
  static OwInterface* instance();
  OwInterface ();
  ~OwInterface ();
  OwInterface (const OwInterface&) = delete;
  OwInterface& operator= (const OwInterface&) = delete;
  void initialize ();

  // Operational interface

  // The defaults currently match those of the activity.  When all are used,
  // this function matches guardedMoveDemo above.
  void guardedMove (double x, double y, double z,
                    double direction_x, double direction_y, double direction_z,
                    double search_distance, int id);
  void tiltAntenna (double degrees, int id);
  void panAntenna (double degrees, int id);
  void takePicture ();
  void digLinear (double x, double y, double depth, double length,
                  double ground_pos, int id);
  void digCircular (double x, double y, double depth,
                    double ground_pos, bool parallel, int id);
  void grind (double x, double y, double depth, double length,
              bool parallel, double ground_pos, int id);
  void stow (int id);
  void unstow (int id);
  void deliverSample (double x, double y, double z, int id);
  void takePanorama (double elev_lo, double elev_hi,
                     double lat_overlap, double vert_overlap);
  void updatePlexilPlanStatus(std::string& plan_name, bool exec_status);

  // Temporary, proof of concept for ROS Actions
  void guardedMoveActionDemo (const geometry_msgs::Point& start,
                              const geometry_msgs::Point& normal,
                              double search_distance,
                              int id);

  // State interface
  double getTilt () const;
  double getPanDegrees () const;
  double getPanVelocity () const;
  double getTiltVelocity () const;
  bool   imageReceived () const;
  double getVoltage () const;
  double getRemainingUsefulLife () const;
  bool   groundFound () const;
  double groundPosition () const;
  double getTimeNow() const;
  double getRandomProb() const;
  bool getDiggingSuccess(const double exca_prob) const;

  // These methods apply to all "lander operations", and hide their ROS
  // implementation, which could be services, actions, or ad hoc messaging.
  bool running (const std::string& name) const;
  bool finished (const std::string& name) const;
  bool hardTorqueLimitReached (const std::string& joint_name) const;
  bool softTorqueLimitReached (const std::string& joint_name) const;

  // Command feedback
  void setCommandStatusCallback (void (*callback) (int, bool));

  // Support re-planning
  bool getHasNewPlan();
  void setHasNewPlan(bool has_new_plan);
  std::string getPlanName();
  void setPlanName(std::string plan_name);


 private:
  // Temporary, support for public version above
  void guardedMoveActionDemo1 (const geometry_msgs::Point& start,
                               const geometry_msgs::Point& normal,
                               double search_distance,
                               int id);

  bool operationRunning (const std::string& name) const;
  bool operationFinished (const std::string& name) const;

  void jointStatesCallback (const sensor_msgs::JointState::ConstPtr&);
  void tiltCallback (const control_msgs::JointControllerState::ConstPtr& msg);
  void panCallback (const control_msgs::JointControllerState::ConstPtr& msg);
  void managePanTilt (const std::string& opname,
                      double position, double velocity,
                      double current, double goal,
                      const ros::Time& start);
  void new_plan_callback(const ow_autonomy::NewPlan new_plan);

  static OwInterface* m_instance;
  ros::NodeHandle* m_genericNodeHandle;

  // Publishers and subscribers

  ros::Publisher*  m_antennaTiltPublisher;
  ros::Publisher*  m_antennaPanPublisher;
  ros::Publisher*  m_leftImageTriggerPublisher;
  ros::Publisher*  m_plexilPlanStatusPublisher;

  ros::Subscriber* m_antennaPanSubscriber;
  ros::Subscriber* m_antennaTiltSubscriber;
  ros::Subscriber* m_jointStatesSubscriber;
  ros::Subscriber* m_cameraSubscriber;
  ros::Subscriber* m_socSubscriber;
  ros::Subscriber* m_rulSubscriber;
  ros::Subscriber* m_guardedMoveSubscriber;
  ros::Subscriber* m_newPlanSubscriber;

  // Action clients
  std::unique_ptr<GuardedMoveActionClient> m_guardedMoveClient;

  // Antenna state - note that pan and tilt can be concurrent.
  double m_currentPan, m_currentTilt;
  double m_goalPan, m_goalTilt;      // commanded pan/tilt values
  ros::Time m_panStart, m_tiltStart; // pan/tilt start times

  // Re-planning support
  bool m_has_new_plan;
  std::string m_plan_name;

};

#endif
