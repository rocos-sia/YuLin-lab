#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


void pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& message_holder)
{
std::cout<<*message_holder<<std::endl;
}

int main(int argc,char **argv )
{
    ros::init(argc,argv,"sub_pose");

    ros::NodeHandle n;

    ros::Subscriber sub_pose=n.subscribe<geometry_msgs::PoseStamped>("aruco_single/pose",1000,pose_Callback);

    ros::spin();

}