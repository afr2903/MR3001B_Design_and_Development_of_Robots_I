#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <cmath>
#include <std_msgs/Float64.h>

using namespace std;

// Differential drive parameters
const double WHEEL_RADIUS = 0.05; // m
const double WHEEL_SEPARATION = 0.20; // m

double goal_x[] = {2.0, 2.0, 4.0, 2.0, 6.0, 2.0, 8.0, 2.0, 10.0, 2.0};
double goal_y[] = {9.0, 2.0, 9.0, 2.0, 9.0, 2.0, 9.0, 2.0, 9.0, 2.0};
double goal_theta[] = {0, M_PI/2, 0, M_PI/2, 0, M_PI/2, 0, M_PI/2, 0, M_PI/2};
int step = 0;
bool turning = false;
bool waiting = false;
double waiting_start = 0.0;

double current_x = 0.0;
double current_y = 0.0;
double current_theta = 0.0;

// Repeat for turtle 2
double goal_x2[] = {4.0, 5.0, 6.0, 5.0, 8.0, 5.0, 10.0, 5.0, 2.0, 5.0};
double goal_y2[] = {9.0, 2.0, 9.0, 2.0, 9.0, 2.0, 9.0, 2.0, 9.0, 2.0};
double goal_theta2[] = {0, M_PI/2, 0, M_PI/2, 0, M_PI/2, 0, M_PI/2, 0, M_PI/2};
int step2 = 0;
bool turning2 = false;
bool waiting2 = false;
double waiting_start2 = 0.0;

double current_x2 = 0.0;
double current_y2 = 0.0;
double current_theta2 = 0.0;

// Repeat for turtle 3
double goal_x3[] = {6.0, 8.0, 8.0, 8.0, 10.0, 8.0, 2.0, 8.0, 4.0, 8.0};
double goal_y3[] = {9.0, 2.0, 9.0, 2.0, 9.0, 2.0, 9.0, 2.0, 9.0, 2.0};
double goal_theta3[] = {0, M_PI/2, 0, M_PI/2, 0, M_PI/2, 0, M_PI/2, 0, M_PI/2};
int step3 = 0;
bool turning3 = false;
bool waiting3 = false;
double waiting_start3 = 0.0;

double current_x3 = 0.0;
double current_y3 = 0.0;
double current_theta3 = 0.0;

void poseCallback(const turtlesim::Pose::ConstPtr &msg) {
    current_x = msg->x;
    current_y = msg->y;
    current_theta = msg->theta;
}

void poseCallback2(const turtlesim::Pose::ConstPtr &msg) {
    current_x2 = msg->x;
    current_y2 = msg->y;
    current_theta2 = msg->theta;
}

void poseCallback3(const turtlesim::Pose::ConstPtr &msg) {
    current_x3 = msg->x;
    current_y3 = msg->y;
    current_theta3 = msg->theta;
}

int main(int argc, char **argv) {
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
    spawn_srv.request.theta = M_PI/2;
    spawn_srv.request.name = "turtle2";
    spawn_client.call(spawn_srv);

    spawn_srv.request.x = 8.0;
    spawn_srv.request.y = 2.0;
    spawn_srv.request.theta = M_PI/2;
    spawn_srv.request.name = "turtle3";
    spawn_client.call(spawn_srv);

    ros::Subscriber pose_sub = nh.subscribe("turtle1/pose", 10, poseCallback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    geometry_msgs::Twist vel_msg;
    
    // Publishers for wheel velocities of turtle1
    ros::Publisher left_wheel_pub1 = nh.advertise<std_msgs::Float64>("turtle1/left_wheel_vel", 10);
    ros::Publisher right_wheel_pub1 = nh.advertise<std_msgs::Float64>("turtle1/right_wheel_vel", 10);
    std_msgs::Float64 left_wheel_msg1;
    std_msgs::Float64 right_wheel_msg1;

    ros::Subscriber pose_sub2 = nh.subscribe("turtle2/pose", 10, poseCallback2);
    ros::Publisher vel_pub2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
    geometry_msgs::Twist vel_msg2;
    
    // Publishers for wheel velocities of turtle2
    ros::Publisher left_wheel_pub2 = nh.advertise<std_msgs::Float64>("turtle2/left_wheel_vel", 10);
    ros::Publisher right_wheel_pub2 = nh.advertise<std_msgs::Float64>("turtle2/right_wheel_vel", 10);
    std_msgs::Float64 left_wheel_msg2;
    std_msgs::Float64 right_wheel_msg2;

    ros::Subscriber pose_sub3 = nh.subscribe("turtle3/pose", 10, poseCallback3);
    ros::Publisher vel_pub3 = nh.advertise<geometry_msgs::Twist>("turtle3/cmd_vel", 10);
    geometry_msgs::Twist vel_msg3;
    
    // Publishers for wheel velocities of turtle3
    ros::Publisher left_wheel_pub3 = nh.advertise<std_msgs::Float64>("turtle3/left_wheel_vel", 10);
    ros::Publisher right_wheel_pub3 = nh.advertise<std_msgs::Float64>("turtle3/right_wheel_vel", 10);
    std_msgs::Float64 left_wheel_msg3;
    std_msgs::Float64 right_wheel_msg3;

    while (ros::ok()) {
        if (turning == false && waiting == false) {
            cout<<"Step: "<<step<<endl;
            double d_error = sqrt(pow(goal_x[step] - current_x, 2) + pow(goal_y[step] - current_y, 2));
            double desired_theta = atan2(goal_y[step] - current_y, goal_x[step] - current_x);
            double error_theta = desired_theta - current_theta;

            if (error_theta > M_PI){
                error_theta -= 2 * M_PI;
            }
            else if (error_theta < -M_PI){
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
                turning = true;
            }
        } else if (turning == true) {
            cout<<"Turning"<<endl;
            double error_theta = goal_theta[step] - current_theta;
            double angular_vel = 1.5 * error_theta;
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = angular_vel;

            if (fabs(error_theta) < 0.02) {
                waiting = true;
                turning = false;
                waiting_start = ros::Time::now().toSec();
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0;
            }
        }
        if (waiting == true) {
            cout<<"Waiting"<<endl;
            if (ros::Time::now().toSec() - waiting_start > 3.0) {
                step++;
                turning = false;
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0;
                waiting = false;
                if (step >= 10) {
                    vel_msg.linear.x = 0.0;
                    vel_msg.angular.z = 0.0;
                    waiting = true;
                    turning = false;
                }
            }
        }

        // Publish turtle1 velocity
        vel_pub.publish(vel_msg);
        
        // Calculate and publish differential drive wheel velocities for turtle1
        left_wheel_msg1.data = (vel_msg.linear.x - (vel_msg.angular.z * WHEEL_SEPARATION / 2)) / WHEEL_RADIUS;
        right_wheel_msg1.data = (vel_msg.linear.x + (vel_msg.angular.z * WHEEL_SEPARATION / 2)) / WHEEL_RADIUS;
        left_wheel_pub1.publish(left_wheel_msg1);
        right_wheel_pub1.publish(right_wheel_msg1);

        // Repeat for turtle 2
        if (turning2 == false && waiting2 == false) {
            cout<<"Step2: "<<step2<<endl;
            double d_error2 = sqrt(pow(goal_x2[step2] - current_x2, 2) + pow(goal_y2[step2] - current_y2, 2));
            double desired_theta2 = atan2(goal_y2[step2] - current_y2, goal_x2[step2] - current_x2);
            double error_theta2 = desired_theta2 - current_theta2;

            if (error_theta2 > M_PI){
                error_theta2 -= 2 * M_PI;
            }
            else if (error_theta2 < -M_PI){
                error_theta2 += 2 * M_PI;
            }

            double linear_vel2 = 1.7 * d_error2;
            double angular_vel2 = 1.7 * error_theta2;

            if (fabs(error_theta2) > 0.02)
            {
                vel_msg2.linear.x = 0.0;
            }
            else
            {
                vel_msg2.linear.x = linear_vel2;
            }
            vel_msg2.angular.z = angular_vel2;

            if (fabs(d_error2) < 0.02) {
                turning2 = true;
            }
        } else if (turning2 == true) {
            cout<<"Turning2"<<endl;
            double error_theta2 = goal_theta2[step2] - current_theta2;
            double angular_vel2 = 1.5 * error_theta2;
            vel_msg2.linear.x = 0.0;
            vel_msg2.angular.z = angular_vel2;

            if (fabs(error_theta2) < 0.02) {
                waiting2 = true;
                turning2 = false;
                waiting_start2 = ros::Time::now().toSec();
                vel_msg2.linear.x = 0.0;
                vel_msg2.angular.z = 0.0;
            }
        } 
        if (waiting2 == true) {
            cout<<"Waiting2"<<endl;
            if (ros::Time::now().toSec() - waiting_start2 > 3.0) {
                step2++;
                turning2 = false;
                vel_msg2.linear.x = 0.0;
                vel_msg2.angular.z = 0.0;
                waiting2 = false;
                if (step2 >= 10) {
                    vel_msg2.linear.x = 0.0;
                    vel_msg2.angular.z = 0.0;
                    turning2 = false;
                    waiting2 = true;
                }
            }
        }

        // Publish turtle2 velocity
        vel_pub2.publish(vel_msg2);
        
        // Calculate and publish differential drive wheel velocities for turtle2
        left_wheel_msg2.data = (vel_msg2.linear.x - (vel_msg2.angular.z * WHEEL_SEPARATION / 2)) / WHEEL_RADIUS;
        right_wheel_msg2.data = (vel_msg2.linear.x + (vel_msg2.angular.z * WHEEL_SEPARATION / 2)) / WHEEL_RADIUS;
        left_wheel_pub2.publish(left_wheel_msg2);
        right_wheel_pub2.publish(right_wheel_msg2);

        // Repeat for turtle 3
        if (turning3 == false && waiting3 == false) {
            cout<<"Step3: "<<step3<<endl;
            double d_error3 = sqrt(pow(goal_x3[step3] - current_x3, 2) + pow(goal_y3[step3] - current_y3, 2));
            double desired_theta3 = atan2(goal_y3[step3] - current_y3, goal_x3[step3] - current_x3);
            double error_theta3 = desired_theta3 - current_theta3;

            if (error_theta3 > M_PI){
                error_theta3 -= 2 * M_PI;
            }
            else if (error_theta3 < -M_PI){
                error_theta3 += 2 * M_PI;
            }

            double linear_vel3 = 1.9 * d_error3;
            double angular_vel3 = 1.9 * error_theta3;

            if (fabs(error_theta3) > 0.02)
            {
                vel_msg3.linear.x = 0.0;
            }
            else
            {
                vel_msg3.linear.x = linear_vel3;
            }
            vel_msg3.angular.z = angular_vel3;

            if (fabs(d_error3) < 0.02) {
                turning3 = true;
            }
        } else if (turning3 == true) {
            cout<<"Turning3"<<endl;
            double error_theta3 = goal_theta3[step3] - current_theta3;
            double angular_vel3 = 1.5 * error_theta3;
            vel_msg3.linear.x = 0.0;
            vel_msg3.angular.z = angular_vel3;

            if (fabs(error_theta3) < 0.02) {
                waiting3 = true;
                turning3 = false;
                waiting_start3 = ros::Time::now().toSec();
                vel_msg3.linear.x = 0.0;
                vel_msg3.angular.z = 0.0;
            }
        }
        if (waiting3 == true) {
            cout<<"Waiting3"<<endl;
            if (ros::Time::now().toSec() - waiting_start3 > 3.0) {
                step3++;
                if (step3 == 10) {
                    vel_msg3.linear.x = 0.0;
                    vel_msg3.angular.z = 0.0;
                }
                turning3 = false;
                vel_msg3.linear.x = 0.0;
                vel_msg3.angular.z = 0.0;
                waiting3 = false;
                if (step3 >= 10) {
                    vel_msg3.linear.x = 0.0;
                    vel_msg3.angular.z = 0.0;
                    turning3 = false;
                    waiting3 = true;
                }
            }
        }

        // Publish turtle3 velocity
        vel_pub3.publish(vel_msg3);
        
        // Calculate and publish differential drive wheel velocities for turtle3
        left_wheel_msg3.data = (vel_msg3.linear.x - (vel_msg3.angular.z * WHEEL_SEPARATION / 2)) / WHEEL_RADIUS;
        right_wheel_msg3.data = (vel_msg3.linear.x + (vel_msg3.angular.z * WHEEL_SEPARATION / 2)) / WHEEL_RADIUS;
        left_wheel_pub3.publish(left_wheel_msg3);
        right_wheel_pub3.publish(right_wheel_msg3);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}