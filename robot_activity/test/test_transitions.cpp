#include <iostream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <sstream>
#include <robot_activity/functional_robot_activity.h>
#include <boost/circular_buffer.hpp>

struct AnyHelper
{
    boost::circular_buffer<robot_activity_msgs::State> events;

    AnyHelper() : events(5)
    {
    }
    
    void cb(const robot_activity_msgs::StateConstPtr& msg)
    {
        ROS_ERROR("Got Callback");
        events.push_back(*msg);
    }
};

  
struct StateTransitions : public ::testing::Test {
    
    ros::NodeHandle *nh = nullptr;
    boost::shared_ptr< boost::asio::io_service > ioService;// (new boost::asio::io_service);
    boost::shared_ptr< boost::asio::io_service::work > work;
    boost::thread_group threadpool;
    int num_nodes = 2;
    ros::Subscriber subscriber;  
    std::map<std::string,robot_activity_msgs::State> stats;
    AnyHelper h;
    StateTransitions() : ioService(new boost::asio::io_service), work(new boost::asio::io_service::work( *ioService )) {};
    virtual void SetUp() {
        ROS_DEBUG("CALLING SETUP");
        nh = new ros::NodeHandle("~");

        for ( int i = 1; i <= num_nodes ; i ++ ) {
          threadpool.create_thread(
                                   boost::bind(&boost::asio::io_service::run, ioService)
                                   );
        }
    }

    virtual void TearDown()
    {
        ROS_DEBUG("CALLING TEARDOWN");
        delete nh;
    }

    void subCallBack(const robot_activity_msgs::StateConstPtr& msg )
    {
        ROS_ERROR("CALLING subCallBack");
        stats[msg->node_name] = *msg;
        h.events.push_back(*msg);
        ROS_ERROR_STREAM("Pushed back" << h.events.size());
    }
   
};

void StandaloneDelayFunction()
{
    robot_activity::FunctionalRobotActivity::onCreate();
}

  //----------------------------------------------------------------------------
  // Test csae   @TODO
  // 1. Subscribe to the /heartbeat
  // 2. Make the first event be a call to robot_activity::FunctionalRobotActivity::onCreate()
  // 3. Verify that we've received this message
  //----------------------------------------------------------------------------
TEST_F(StateTransitions,OnCreate)
{
    TearDown();
    SetUp();
    ros::NodeHandle nh("~");
    ros::Subscriber subscriber = nh.subscribe<robot_activity_msgs::State>("/heartbeat",
                                                                           60,
                                                                           boost::bind(&StateTransitions::subCallBack,this,_1));
    ros::spinOnce();
    ros::Rate loop_rate(1000);
    ros::Time start = ros::Time::now();    
    robot_activity::FunctionalRobotActivity::useNamespace("this_test");

    ioService->post( boost::bind(StandaloneDelayFunction) );
    int counter  = 0;

    while ( ros::ok() && (counter++) < 5) {
      ros::spinOnce();
      loop_rate.sleep();
    }
    
    ros::spinOnce();
    ASSERT_EQ(h.events.size(), 1);
    robot_activity_msgs::State st = h.events.back();
    ASSERT_EQ( st.state , robot_activity_msgs::State::LAUNCHING );
    work.reset();
    threadpool.join_all();
}


TEST_F(StateTransitions,OnConfigure)
{
    ros::NodeHandle nh("~");
    ros::Subscriber subscribers = nh.subscribe<robot_activity_msgs::State>("/heartbeat",
                                                                           60,
                                                                           boost::bind(&StateTransitions::subCallBack,this,_1));
    ros::spinOnce();
    ros::Rate loop_rate(1000);
    ros::Time start = ros::Time::now();    
    robot_activity::FunctionalRobotActivity::useNamespace("this_test");

    ioService->post( [](void){
                       robot_activity::FunctionalRobotActivity::onConfigure();
                     } );
    int counter  = 0;

    while ( ros::ok() && (counter++) < 5) {
      ros::spinOnce();
      loop_rate.sleep();
    }

    robot_activity_msgs::State st = h.events.back();
    ASSERT_EQ( st.state     , robot_activity_msgs::State::STOPPED);
    ASSERT_EQ( st.node_name , "this_test");
   
}

TEST_F(StateTransitions,OnStart)
{
    ros::NodeHandle nh("~");
    ros::Subscriber subscribers = nh.subscribe<robot_activity_msgs::State>("/heartbeat",
                                                                           60,
                                                                           boost::bind(&StateTransitions::subCallBack,this,_1));
    ros::spinOnce();
    ros::Rate loop_rate(1000);
    ros::Time start = ros::Time::now();    
    robot_activity::FunctionalRobotActivity::useNamespace("this_test");

    ioService->post( [](void){
                       robot_activity::FunctionalRobotActivity::onStart();
                     } );
    int counter  = 0;

    while ( ros::ok() && (counter++) < 5) {
      ros::spinOnce();
      loop_rate.sleep();
    }

    robot_activity_msgs::State st = h.events.back();
    ASSERT_EQ( st.state , robot_activity_msgs::State::RUNNING);

    ioService->post( [](void){
                       robot_activity::FunctionalRobotActivity::onStop();
                     } );
    
    counter = 0;
    while ( ros::ok() && (counter++) < 5) {
      ros::spinOnce();
      loop_rate.sleep();
    }

    st = h.events.back();
    ASSERT_EQ( st.state , robot_activity_msgs::State::STOPPED);
    ASSERT_EQ( st.node_name , "this_test");
}
  
#include <ros/console.h>
#include <log4cxx/logger.h>

int
main(int argc, char *argv[] )
{

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "delaystats" );
  return RUN_ALL_TESTS();
}

