#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_goal");

    MoveBaseClient ac("/move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up!");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 8;
    goal.target_pose.pose.position.y = 8;
    goal.target_pose.pose.position.z = 0;
    goal.target_pose.pose.orientation.w = 1;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Mission complete!");
    else    
        ROS_INFO("Mission falied!");

    return 0;
}
