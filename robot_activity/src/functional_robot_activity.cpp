#include <robot_activity/functional_robot_activity.h>
#include <robot_activity/managed_robot_activity.h>
#include <robot_activity_msgs/State.h>
#include <robot_activity_msgs/Error.h>

namespace robot_activity
{

  void FunctionalRobotActivity::useNamespace(std::string ns ) {
    node_namespace_ = ns ;
  }

  robot_activity_msgs::State  FunctionalRobotActivity::getState()
  {
          robot_activity_msgs::State state;
          state.node_name = node_namespace_;
          return state;
  }
  
  void FunctionalRobotActivity::checkPublisher() {
    if ( ! node_handle_private_ ) {
      node_handle_private_ = ros::NodeHandlePtr(new ros::NodeHandle("~" + node_namespace_));
      process_state_pub_ = node_handle_private_->advertise<robot_activity_msgs::State>("/heartbeat", 0, true);
    }
  }
  
  void FunctionalRobotActivity::onCreate()
  {
          ROS_DEBUG("onCreate");
          FunctionalRobotActivity::checkPublisher();
          robot_activity_msgs::State state = FunctionalRobotActivity::getState();
          state.state = robot_activity_msgs::State::LAUNCHING;
          process_state_pub_.publish(state);
  }

  void FunctionalRobotActivity::onTerminate()
  {
    ROS_DEBUG("onTerminate");
    FunctionalRobotActivity::checkPublisher();
    robot_activity_msgs::State state = FunctionalRobotActivity::getState();
    state.state = robot_activity_msgs::State::TERMINATED;
    process_state_pub_.publish(state);
    // onManagedTerminate();
  }

  bool FunctionalRobotActivity::onConfigure()
  {
    FunctionalRobotActivity::checkPublisher();
    robot_activity_msgs::State state = FunctionalRobotActivity::getState();
    state.state = robot_activity_msgs::State::STOPPED;
    process_state_pub_.publish(state);
    return true;
  }

  bool FunctionalRobotActivity::onUnconfigure()
  {
    FunctionalRobotActivity::checkPublisher();
    robot_activity_msgs::State state = FunctionalRobotActivity::getState();
    state.state = robot_activity_msgs::State::UNCONFIGURED;
    process_state_pub_.publish(state);
    return true;
  }

  bool FunctionalRobotActivity::onStart()
  {
    ROS_DEBUG("onStart");
    FunctionalRobotActivity::checkPublisher();
    robot_activity_msgs::State state = FunctionalRobotActivity::getState();
    state.state = robot_activity_msgs::State::RUNNING;
    process_state_pub_.publish(state);
    return true;
  }

  bool FunctionalRobotActivity::onStop()
  {
    ROS_DEBUG("onStart");
    FunctionalRobotActivity::checkPublisher();
    robot_activity_msgs::State state = FunctionalRobotActivity::getState();
    state.state = robot_activity_msgs::State::STOPPED;
    process_state_pub_.publish(state);
    return true;
  }

  bool FunctionalRobotActivity::onPause()
  {
    ROS_DEBUG("onPause");
    FunctionalRobotActivity::checkPublisher();
    robot_activity_msgs::State state = FunctionalRobotActivity::getState();
    state.state = robot_activity_msgs::State::PAUSED;
    process_state_pub_.publish(state);
    return true;
  }

  bool FunctionalRobotActivity::onResume()
  {
    ROS_DEBUG("onResume");
    robot_activity_msgs::State state = FunctionalRobotActivity::getState();
    state.state = robot_activity_msgs::State::RUNNING;
    process_state_pub_.publish(state);
    return true;
  }

  ros::Publisher FunctionalRobotActivity::process_state_pub_;
  ros::NodeHandlePtr FunctionalRobotActivity::node_handle_private_;
  std::string FunctionalRobotActivity::node_namespace_;

  
}  // namespace robot_activity
