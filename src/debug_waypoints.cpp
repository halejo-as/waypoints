#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <fstream>
using namespace std;

ofstream myfile;
double time_init;

std::string path;
std::string nameFile;

void writeGoal(const std_msgs::String::ConstPtr& msg){
        myfile.open(path + "/" +  nameFile, ios::app);
        double time = ros::Time::now().toSec()-time_init;
        myfile << time << ",";
        sensor_msgs::BatteryState bat = *ros::topic::waitForMessage<sensor_msgs::BatteryState>("/battery_state");
        myfile << bat.percentage << ",";
        geometry_msgs::PoseWithCovarianceStamped pose = *ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose");
        myfile << pose.pose.pose.position.x <<","<< pose.pose.pose.position.y <<","<<pose.pose.pose.orientation.z ;
        myfile << msg->data << "\n";
        myfile.close();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "debug_waypoints");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    path = ros::package::getPath("waypoints");
    nameFile = "report_navigation.txt";

    if (argc > 1){
        nameFile = argv[1];
        ROS_INFO("Got nameFile : %s", nameFile.c_str());
    }
    else
        ROS_INFO("/report_navigation.txt default");
    
    myfile.open(path +"/"+ nameFile);
    myfile << "time (sec),  battery, x, y , theta, wp, STATUS \n";
    myfile.close();
    double time_init = ros::Time::now().toSec();
    while (ros::ok())
    {
        myfile.open(path + "/" +  nameFile, ios::app);
        double time = ros::Time::now().toSec()-time_init;
        myfile << time << ",";
        sensor_msgs::BatteryState bat = *ros::topic::waitForMessage<sensor_msgs::BatteryState>("/battery_state");
        myfile << bat.percentage << ",";
        geometry_msgs::PoseWithCovarianceStamped pose = *ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose");
        myfile << pose.pose.pose.position.x <<","<< pose.pose.pose.position.y <<","<<pose.pose.pose.orientation.z << "\n";
      /*  std_msgs::String msg_debug = *ros::topic::waitForMessage<std_msgs::String>("/waypoint_server/statusGoal");
        myfile << msg_debug.data << "\n"; */
        myfile.close();
        ros::spinOnce();
    }

    return 0;
}
