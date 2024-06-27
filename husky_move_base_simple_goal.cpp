#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <vector>
#include <tf/transform_broadcaster.h>

nav_msgs::Odometry pose_msg;

void odomCallback(const nav_msgs::Odometry::ConstPtr& data)
{
    pose_msg = *data;   
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_publisher");
    ros::NodeHandle nh;

    ros::Subscriber subOdom = nh.subscribe("/odometry/filtered", 10, odomCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    ros::Rate rate(10.0);
    std::vector<std::pair<int, int>> points = {{20,0}, {40,0}, {67,0}, {67,20}, {40,20}, {20,20}, {0,20}, {0, 0}};
    
    for (int i = 0; i < points.size(); i++) 
    {
    	geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map"; 
	
        goal.pose.position.x = points[i].first;
        goal.pose.position.y = points[i].second;
        goal.pose.position.z = 0.0;
    
    	double Dx = points[i].first - pose_msg.pose.pose.position.x;
        double Dy = points[i].second - pose_msg.pose.pose.position.y;
        
        double Dtheta = std::atan2(Dy, Dx);
        tf::Quaternion q = tf::createQuaternionFromYaw(Dtheta);

        goal.pose.orientation.x = q.x();
        goal.pose.orientation.y = q.y();
        goal.pose.orientation.z = q.z();
        goal.pose.orientation.w = q.w();

        pub.publish(goal);
        
        ros::spinOnce();
        rate.sleep();
        
        while (ros::ok())
        {
           double Dx = points[i].first - pose_msg.pose.pose.position.x;
           double Dy = points[i].second - pose_msg.pose.pose.position.y;
           double dist = std::sqrt(Dx * Dx + Dy * Dy);
           
           if (dist < 0.5) 
           {
              break;
           }
        
        ros::spinOnce();
        rate.sleep();
        } 

    }

    return 0;
  
}    
