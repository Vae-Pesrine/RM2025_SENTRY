#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include "mbf_msgs/MoveBaseAction.h"

class MbfBridge
{
public: 
  MbfBridge() {
    ros::NodeHandle action_nh("move_base_flex/move_base");
    action_goal_pub_ = action_nh.advertise<mbf_msgs::MoveBaseActionGoal>("goal", 1);

    ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MbfBridge::goalCB, this, _1));
    
    ros::NodeHandle nh("/");
  }

  ~MbfBridge() = default;

private:
  
  /**
   * @brief change the move base goal to mbf action
   */
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    tf2::Quaternion q;
    tf2::fromMsg(goal->pose.orientation, q);
    q.normalize();

    mbf_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose.header = goal->header;
    action_goal.goal.target_pose.pose.position = goal->pose.position;
    action_goal.goal.target_pose.pose.orientation = tf2::toMsg(q);

    action_goal_pub_.publish(action_goal);
    ROS_INFO("the goal has been sent to the action server");
  }

private:
  ros::Publisher action_goal_pub_;
  ros::Subscriber goal_sub_;
};


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "mbf_bridge");

  MbfBridge _bridge;
  ros::spin();

  return 0;
}