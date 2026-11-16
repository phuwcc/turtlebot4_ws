#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_default_plugins/tools/goal_pose/goal_tool.hpp>

namespace tb4_multi_bringup {
// Keep RViz's standard click/drag interaction, with a fixed robot per toolbar tool.
class RobotGoalTool : public rviz_default_plugins::tools::GoalTool {
public:
  explicit RobotGoalTool(int robot):robot_(robot) { shortcut_key_='0'+robot; }
  void onInitialize() override {
    rviz_default_plugins::tools::GoalTool::onInitialize();
    setName(QString("Goal robot%1").arg(robot_));
    setDescription(QString("Click and drag on the map to navigate robot%1").arg(robot_));
    getPropertyContainer()->subProp("Topic")->setValue(
      QString("/tb4_view/robot%1/goal_pose").arg(robot_));
  }
private:
  int robot_;
};
class GoalRobot1 : public RobotGoalTool { public: GoalRobot1():RobotGoalTool(1) {} };
class GoalRobot2 : public RobotGoalTool { public: GoalRobot2():RobotGoalTool(2) {} };
class GoalRobot3 : public RobotGoalTool { public: GoalRobot3():RobotGoalTool(3) {} };
}
PLUGINLIB_EXPORT_CLASS(tb4_multi_bringup::GoalRobot1, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(tb4_multi_bringup::GoalRobot2, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(tb4_multi_bringup::GoalRobot3, rviz_common::Tool)
