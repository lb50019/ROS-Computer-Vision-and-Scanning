#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <boxor_msgs/idenjeAction.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

class Stop
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<boxor_msgs::idenjeAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  boxor_msgs::idenjeFeedback feedback_;
  boxor_msgs::idenjeResult result_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

public:

  Stop(std::string name) :
    as_(nh_, name, boost::bind(&Stop::executeCB, this, _1), false),
    action_name_(name)
  {
  	//Topic you want to publish
    pub_ = nh_.advertise<geometry_msgs::Twist>("robot1/cmd_vel", 1000);

    //Topic you want to subscribe
    sub_ = nh_.subscribe("robot1/scan", 1000, &Stop::laser_callback, this);
    
    as_.start();
  }

  ~Stop(void)
  {
  }
  
  void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    feedback_.tren_raz = msg->ranges[0];
    // publish the feedback
    as_.publishFeedback(feedback_);
  }

  void executeCB(const boxor_msgs::idenjeGoalConstPtr &goal)
  {
    // helper variables
    bool success = true;
    
    geometry_msgs::Twist move;
    
    ros::Rate rate(30);
	
    // start executing the action
    while(1)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      
      if(feedback_.tren_raz <= goal->raz)
      {
        move.linear.x = 0;
        pub_.publish(move);
        break;
      }
     
    }

    if(success)
    {
      result_.kon_raz = feedback_.tren_raz;
      ROS_INFO("%s: uspio", action_name_.c_str());
      // set the action state to succeeded
      move.linear.x = 0;
      pub_.publish(move);
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "stop_node");

  Stop stop("cpp_action_server");
  ros::spin();

  return 0;
}
