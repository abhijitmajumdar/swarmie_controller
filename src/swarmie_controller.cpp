// ROS includes
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "tf/tf.h"

// System includes
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>   //sqrt,pow,atan2,fabs
#include <string.h>

static ros::Subscriber odometer_subsciber;
static ros::Subscriber imu_subsciber;
static ros::Publisher velocity_publisher;
static float tolerance;
static turtlesim::Pose myPose;

void initialize(float myTolerance,std::string pubTopic,std::string subTopicOdom,std::string subTopicImu);
void move2goal(float x,float y);
//void callback_for_odometry(const nav_msgs::Odometry::ConstPtr& msg);
void callback_for_imu(const sensor_msgs::Imu::ConstPtr& msg);
float get_distance(float goal_distance,float goal_angle);
float wrapPi(float angle);
float constrain(float val,float min_val,float max_val);

void initialize(float myTolerance, std::string pubTopic, std::string subTopicOdom, std::string subTopicImu)
{
    // Creating our publisher and subscriber
    ros::NodeHandle n;
    //odometer_subsciber = n.subscribe(subTopicOdom, 10, callback_for_odometry);
    imu_subsciber = n.subscribe(subTopicImu, 10, callback_for_imu);
    velocity_publisher = n.advertise<geometry_msgs::Twist>(pubTopic, 10);
    tolerance = myTolerance;
    myPose.x = 0;
    myPose.y = 0;
    myPose.theta = 0;
}

/*
void callback_for_odometry(const nav_msgs::Odometry::ConstPtr& msg)
{
    myPose.x = msg->pose.pose.position.x;
    myPose.y = msg->pose.pose.position.y;
    myPose.theta = tf::getYaw(msg->pose.pose.orientation);
}
*/

void callback_for_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
    myPose.theta = tf::getYaw(msg->orientation);
}

float get_distance(float goal_x,float goal_y)
{
    return sqrt(pow((goal_x - myPose.x), 2) + pow((goal_y - myPose.y), 2));
}

float wrapPi(float angle)
{
    if(angle > 0){
        if(angle > M_PI) angle = angle - (2*M_PI);
    }
    else{
        if(angle < -M_PI) angle = angle + (2*M_PI);
    }
    return angle;
}

float constrain(float val,float min_val,float max_val)
{
    return std::min(max_val, std::max(min_val, val));
}

void move2goal(float x,float y)
{
    turtlesim::Pose goal_pose;
    goal_pose.x = x;
    goal_pose.y = y;
    geometry_msgs::Twist vel_msg;
    float dist_to_goal = get_distance(goal_pose.x, goal_pose.y);
    float angle_to_goal = 0;
    ros::Rate loop_rate(20);

    while ((dist_to_goal>=tolerance) & ros::ok())
    {
        ros::spinOnce();
        float goal_angle = atan2(goal_pose.y - myPose.y, goal_pose.x - myPose.x);
        myPose.theta = wrapPi(myPose.theta);
        angle_to_goal = goal_angle - myPose.theta;
        angle_to_goal = wrapPi(angle_to_goal);

        // Turn
        if((fabs(angle_to_goal)) > 0.15)
        {
            vel_msg.angular.z = 1 * angle_to_goal;
            vel_msg.linear.x = 0;
        }
        // Move
        else
        {
            vel_msg.linear.x = 0.6 * dist_to_goal;
            vel_msg.angular.z = 0.6 * angle_to_goal;

        }
        vel_msg.linear.x = constrain(vel_msg.linear.x,-0.25,0.25);
        vel_msg.angular.z = constrain(vel_msg.angular.z,-0.3,0.3);
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;
        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;

        // Publishing our vel_msg
        velocity_publisher.publish(vel_msg);
        dist_to_goal -= vel_msg.linear.x * 0.05;
        //myPose.x += (vel_msg.linear.x * 0.05 * sin(myPose.theta));
        //myPose.y += (vel_msg.linear.x * 0.05 * cos(myPose.theta));
        //std::cout<< myPose.x << '\t' << myPose.y << '\t' << myPose.theta << '\t' << dist_to_goal << '\t' << angle_to_goal << '\n';
        loop_rate.sleep();
    }
    // Stopping our robot when destination reached
    myPose.x = goal_pose.x;
    myPose.y = goal_pose.y;
    std::cout<< myPose.x << ',' << myPose.y << '\n';
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
    loop_rate.sleep();
    velocity_publisher.publish(vel_msg);
    loop_rate.sleep();
    velocity_publisher.publish(vel_msg);
    loop_rate.sleep();
    velocity_publisher.publish(vel_msg);
}

int main(int argc, char** argv)
{
    // Register node with master
    ros::init(argc, argv, "swarmie_controller");
    // Initialize
    initialize(0.08, "/chappie/driveControl", "/chappie/odom", "/chappie/imu");

    if(argc == 3)
    {
        // Go to position
        move2goal(std::stoi(argv[1]),std::stoi(argv[2]));
    }
    else
    {
        // Testing: Make a square
        move2goal(1,0);
        move2goal(1,1);
        move2goal(0,1);
        move2goal(0,0);
    }

    return 0;
}
