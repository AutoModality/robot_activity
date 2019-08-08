#ifndef FUNCTIONAL_ROBOT_ACTIVITY_H
#define FUNCTIONAL_ROBOT_ACTIVITY_H

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <std_srvs/Empty.h>
#include <robot_activity_msgs/State.h>
#include <robot_activity_msgs/Error.h>

#include <robot_activity/robot_activity.h>
#include <robot_activity/resource/resource_manager.h>

namespace robot_activity
{

/**
 * @brief Managed RobotActivity class, which adds further functionality to the
 *        RobotActivity class
 * @details ManagedRobotActivity manages ROS Subscribers and ServiceServers.
 *          It automatically pauses all Subscribers and ServiceServers
 *          during PAUSED state and resumes them when transitioning to the
 *          RUNNING state. It also shutdowns them in STOPPED state and
 *          re-acquires them (by re-subscribing or re-advertising) when
 *          transitioning from STOPPED to PAUSED.
 */
class FunctionalRobotActivity
{
public:

  static void onCreate() ;
  static void onTerminate() ;
  static bool onConfigure() ;
  static bool onUnconfigure() ;
  static bool onStart() ;
  static bool onStop() ;
  static bool onPause() ;
  static bool onResume() ;
  static void useNamespace(std::string ns );

  static ros::Publisher process_state_pub_;
  static ros::NodeHandlePtr node_handle_private_;
  static std::string node_namespace_;
  
private:
  static void checkPublisher();
  static robot_activity_msgs::State getState();

};

}

#endif
