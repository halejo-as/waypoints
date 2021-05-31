#ifndef WAYPOINTSERVER_H
#define WAYPOINTPANEL_H

#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <ros/package.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>

#include "waypoints/waypoint_msg.h"
#include "waypoints/waypoint_group.h"
#include "waypoints/waypoint_array.h"
#include "waypoints/Save_Wp.h"
#include "waypoints/Load_Wp.h"
#include "waypoints/Run_Wp.h"
#include "waypoints/Stop_Wp.h"
#include "waypoints/Delete_Wp.h"
#include "waypoints/Groups_Wp.h"
#include "waypoints/Wp_2_Group.h"

#define foreach BOOST_FOREACH

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class WaypointServer
{
public:
    WaypointServer();
    bool SaveWp(waypoints::Save_Wp::Request &req, waypoints::Save_Wp::Response &res);
    bool LoadWp(waypoints::Load_Wp::Request &req, waypoints::Load_Wp::Response &res);
    void CallbackGr(waypoints::waypoint_group gr_msg);
    void CallbackWp(waypoints::waypoint_msg wp_msg);
    void PublishWp();
    bool RunWp(waypoints::Run_Wp::Request &req, waypoints::Run_Wp::Response &res);
    bool StopWp(waypoints::Stop_Wp::Request &req, waypoints::Stop_Wp::Response &res);
    bool DeleteWp(waypoints::Delete_Wp::Request &req, waypoints::Delete_Wp::Response &res);
    bool GroupOptionWp(waypoints::Groups_Wp::Request &req, waypoints::Groups_Wp::Response &res);
    bool WpGroup(waypoints::Wp_2_Group::Request &req, waypoints::Wp_2_Group::Response &res);
    void onLoop(int state);

protected:
    ros::NodeHandle n;

    //services
    ros::ServiceServer srv_save;
    ros::ServiceServer srv_load;
    ros::ServiceServer srv_run_wp;
    ros::ServiceServer srv_stop_wp;
    ros::ServiceServer srv_delete_wp;
    ros::ServiceServer srv_groups_wp;
    ros::ServiceServer srv_wp_2_group;
    // for save/ load files
    std::string path;

    bool stop;

    //internal variables
    std::map<std::string, waypoints::waypoint_msg> wp_map;
    std::map<std::string, waypoints::waypoint_group> groups;

    ros::Subscriber sub_wp;
    ros::Subscriber sub_gr;
    ros::Publisher pub_wp;
    ros::Publisher pub_gr;
    ros::Publisher pub_debug;
};

#endif
