#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "send_goal");
    MoveBaseClient ac("move_base", true);
    tf::TransformListener listener;

    // 等待 move_base 启动
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    tf::StampedTransform transform;
    try {
        listener.waitForTransform("base_link", "map", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("base_link", "map", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return -1;
    }

    float current_x = transform.getOrigin().x();
    float current_y = transform.getOrigin().y();
    float yaw = tf::getYaw(transform.getRotation());

    ROS_INFO("Current position: x=%.2f, y=%.2f, yaw=%.2f", current_x, current_y, yaw);

    goal.target_pose.pose.position.x = current_x + 2.0 * cos(yaw);
    goal.target_pose.pose.position.y = current_y + 2.0 * sin(yaw);
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    
    ROS_INFO("Sending goal: x=%.2f, y=%.2f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Reached goal!");
    else
        ROS_INFO("Failed to reach goal");

    return 0;
}
