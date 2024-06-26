#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>
#include <cmath>
#include <vector>
#include <fstream>

double x = 0.0;
double y = 0.0;
double theta = 0.0;

std::vector<std::pair<double, double>> positions;

float covariance_[36];
for (int i = 0; i < 36; i++){
	covariance[i] = 0;
}

covariance_[0] = 0.5;  // x
covariance_[8] = 0.5;  // y
covariance_[22] = 0.5; // z
covariance_[29] = 0.0872; // theta

geometry_msgs::Quaternion orientation_q = data->pose.pose.orientation; 

geometry_msgs::PoseWithCovarianceStamped pose_msg;

void odomCallback(const nav_msgs::Odometry::ConstPtr& data)
{
    pose_msg.pose.pose.position.x = data->pose.pose.position.x;
    pose_msg.pose.pose.position.y = y = data->pose.pose.position.y;
    pose_msg.pose.covariance = covariance_;
    pose_msg.pose.orientation = data.pose.pose.orientation;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "speed_controller");
    ros::NodeHandle nh;

    ros::Subscriber subOdom = nh.subscribe("/odometry_node/odometry", 10, odomCallback);
    ros::Publisher pub = ros::NodeHandle().advertise<geometry_msgs::PoseWithCovarianceStamped>("lidar/kiss_icp", 10);

    geometry_msgs::Twist speed;
    ros::Rate rate(50); 

    std::vector<std::pair<double, double>> points = {{1, 0}, {1, 1}, {0, 1}, {0, 0}};
    int i = 0;

    while (ros::ok())
    {
        double Dx = points[i].first - x;
        double Dy = points[i].second - y;

        double Dtheta = std::atan2(Dy, Dx) - theta;

        double dist = std::sqrt(Dx * Dx + Dy * Dy);

        if (std::atan2(Dy, Dx) < 0 && i > 1)
        {
            Dtheta = std::atan2(Dy, Dx) + 2 * M_PI - theta;
            if (theta < 0)
            {
                Dtheta = std::abs(-std::atan2(Dy, Dx) + theta);
            }
        }

        speed.angular.z = 0.5 * Dtheta;
        speed.linear.x = 0.1 * dist;

        pubCmdVel.publish(speed);

        if (dist < 0.3 && i < points.size() - 1)
        {
            i++;
        }

        if (i == points.size() - 1 && std::abs(x) < 0.1 && std::abs(y) < 0.1)
        {
            speed.linear.x = 0;
            speed.angular.z = 0;
            pubCmdVel.publish(speed);
            break;
        }
        
	pub.publish(pose_msg);
	ros::spinOnce();
	rate.sleep();
    }

    return 0;
}
