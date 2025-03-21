#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <cmath>

using namespace std;

double goal_x[] = {2.0, 2.0, 4.0, 2.0, 6.0, 2.0, 8.0, 2.0, 10.0, 2.0};
double goal_y[] = {9.0, 2.0, 9.0, 2.0, 9.0, 2.0, 9.0, 2.0, 9.0, 2.0};
double goal_theta[] = {0, M_PI/2, 0, M_PI/2, 0, M_PI/2, 0, M_PI/2, 0, M_PI/2};
int step = 0;

double current_x = 0.0;
double current_y = 0.0;
double current_theta = 0.0;

void poseCallback(const turtlesim::Pose::ConstPtr &msg)
{
    current_x = msg->x;
    current_y = msg->y;
    current_theta = msg->theta;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_final_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::Duration(5).sleep();

    // Kill turtle 1 to spawn it again in (2, 2), facing up
    // Spawn a new turtle2 at (2, 5), facing up
    // Spawn a new turtle3 at (2, 8), facing up

    ros::ServiceClient kill_client = nh.serviceClient<turtlesim::Kill>("kill");
    turtlesim::Kill kill_srv;
    kill_srv.request.name = "turtle1";
    kill_client.call(kill_srv);
    cout<<"Killed turtle1"<<endl;

    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 2.0;
    spawn_srv.request.y = 2.0;
    spawn_srv.request.theta = M_PI/2;
    spawn_srv.request.name = "turtle1";
    spawn_client.call(spawn_srv);

    spawn_srv.request.x = 5.0;
    spawn_srv.request.y = 2.0;
    spawn_srv.request.theta = 0.0;
    spawn_srv.request.name = "turtle2";
    spawn_client.call(spawn_srv);

    spawn_srv.request.x = 8.0;
    spawn_srv.request.y = 2.0;
    spawn_srv.request.theta = 0.0;
    spawn_srv.request.name = "turtle3";
    spawn_client.call(spawn_srv);

    ros::Subscriber pose_sub = nh.subscribe("turtle1/pose", 10, poseCallback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    geometry_msgs::Twist vel_msg;

    while (ros::ok())
    {
        double d_error = sqrt(pow(goal_x[step] - current_x, 2) + pow(goal_y[step] - current_y, 2));
        double desired_theta = atan2(goal_y[step] - current_y, goal_x[step] - current_x);
        double error_theta = desired_theta - current_theta;

        if (error_theta > M_PI)
        {
            error_theta -= 2 * M_PI;
        }
        else if (error_theta < -M_PI)
        {
            error_theta += 2 * M_PI;
        }

        double linear_vel = 1.5 * d_error;
        double angular_vel = 1.5 * error_theta;

        if (fabs(error_theta) > 0.02)
        {
            vel_msg.linear.x = 0.0;
        }
        else
        {
            vel_msg.linear.x = linear_vel;
        }
        vel_msg.angular.z = angular_vel;

        if (fabs(d_error) < 0.02) {
            error_theta = goal_theta[step] - current_theta;
            while ( fabs(error_theta) > 0.02) {
                angular_vel = 1.5 * error_theta;
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = angular_vel;
                error_theta = goal_theta[step] - current_theta;
                vel_pub.publish(vel_msg);
                ros::spinOnce();
                loop_rate.sleep();
            }
            //sleep for 5 seconds
            ros::Duration(5).sleep();

            step++;
            if (step == 10)
            {
                break;
            }
        }

        vel_pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}