#ifndef WAYPOINTS_TOOL_H
#define WAYPOINTS_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>
# include <ros/ros.h>
# include "rviz/default_plugin/tools/pose_tool.h"
#endif

namespace rviz
{
	class StringProperty;
}
namespace waypoints
{
	class WaypointTool: public rviz::PoseTool
	{
		Q_OBJECT
		public:
		  WaypointTool();
		  virtual ~WaypointTool() {}
		  virtual void onInitialize();

		protected:
		  virtual void onPoseSet(double x, double y, double theta);


		private Q_SLOTS:
		  void updateTopic();

		private:
		  ros::NodeHandle nh_;
		  ros::Publisher pub_;
		  rviz::StringProperty* topic_property_;
	};
}

#endif
