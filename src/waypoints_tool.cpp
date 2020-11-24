#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include "waypoints/waypoint_msg.h"
#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"

#include "waypoints_tool.h"
#include <QString>
#include <QInputDialog>
#include <QLineEdit>
#include <cstddef>

namespace waypoints
{

  WaypointTool::WaypointTool()
  {
    shortcut_key_ = 'w';

    topic_property_ = new rviz::StringProperty("Topic", "waypoint",
                                               "The topic on which to publish navigation waypoints.",
                                               getPropertyContainer(), SLOT(updateTopic()), this);
  }
  void WaypointTool::onInitialize()
  {
    PoseTool::onInitialize();
    setName("Waypoints");
    updateTopic();
  }

  void WaypointTool::updateTopic()
  {
    try
    {
      pub_ = nh_.advertise<waypoints::waypoint_msg>("/waypoint", 1);
    }
    catch (const ros::Exception &e)
    {
      ROS_ERROR_STREAM_NAMED("WaypointTool", e.what());
    }
  }

  void WaypointTool::onPoseSet(double x, double y, double theta)
  {
    std::string name_wp = QInputDialog::getText(nullptr, "Wayypoint Name", "Name:", QLineEdit::Normal, "").toStdString();
    if (name_wp.empty())
    {
      ROS_INFO("Put a valid name");
    }
    else
    {
      tf::Quaternion quat;
      quat.setRPY(0.0, 0.0, theta);
      tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), name_wp);
      geometry_msgs::PoseStamped waypoint;
      tf::poseStampedTFToMsg(p, waypoint);
      ROS_INFO("\nSetting waypoint %s:\n  Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) , Angle: %.3f\n",
               name_wp.c_str(), waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z,
               waypoint.pose.orientation.x, waypoint.pose.orientation.y, waypoint.pose.orientation.z, waypoint.pose.orientation.w, theta);
      waypoints::waypoint_msg msg;
      msg.name = name_wp;
      msg.pose.orientation = waypoint.pose.orientation; 
      msg.pose.position = waypoint.pose.position;

      pub_.publish(msg);
    }
  }

} // end namespace waypoints

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoints::WaypointTool, rviz::Tool)
