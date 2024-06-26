#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <vector>
#include <tf/transform_datatypes.h>

std::vector<std::pair<double, double>> positions;

nav_msgs::Odometry pose_msg;

double theta = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& data)
{
    pose_msg = *data;   
    
    tf::Quaternion rot_q;
    tf::quaternionMsgToTF(data->pose.pose.orientation, rot_q);
    
    double roll, pitch, yaw;
    tf::Matrix3x3(rot_q).getRPY(roll,pitch,yaw);
    theta = yaw;
}

double normalize(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "speed_controller");
    ros::NodeHandle nh;

    ros::Subscriber subOdom = nh.subscribe("/odometry_node/odometry", 10, odomCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    geometry_msgs::Twist speed;
    ros::Rate rate(10.0);
    std::vector<std::pair<int, int>> points = {{60, 0}, {60, 20}, {0, 20}, {0, 0}};
    int i = 0;

    while (ros::ok())
    {

        double Dx = points[i].first - pose_msg.pose.pose.position.x;
        double Dy = points[i].second - pose_msg.pose.pose.position.y;

        double Dtheta = std::atan2(Dy, Dx) - theta;
        Dtheta = normalize(Dtheta);

        double dist = std::sqrt(Dx * Dx + Dy * Dy);

	if (Dtheta < 0.4) 
	{ 
		speed.angular.z = 0.0;
	} 
	else {
		speed.angular.z = std::copysign(0.15, Dtheta) ;
	}
	
        speed.linear.x = 0.15; 

        pub.publish(speed);

        if (dist < 1.0 && i < points.size() - 1)
        {
            i++;
        }

        if (i == points.size() - 1 && std::abs(pose_msg.pose.pose.position.x) < 1.0 && std::abs(pose_msg.pose.pose.position.y) < 1.0)
        {
            speed.linear.x = 0;
            speed.angular.z = 0;
            pub.publish(speed);
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
