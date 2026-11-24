#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_default_plugins/tools/goal_pose/goal_tool.hpp>
#include <rviz_default_plugins/tools/pose_estimate/initial_pose_tool.hpp>

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
// Standard 2D Pose Estimate drag, with a fixed robot per toolbar tool: the
// gateway applies /tb4_view/robotN/initialpose to that robot only.
class RobotPoseTool : public rviz_default_plugins::tools::InitialPoseTool {
public:
  explicit RobotPoseTool(int robot):robot_(robot) { shortcut_key_=robot<4 ? '6'+robot : '0'; }
  void onInitialize() override {
    rviz_default_plugins::tools::InitialPoseTool::onInitialize();
    setName(QString("Pose robot%1").arg(robot_));
    setDescription(QString("Click and drag on the map to set the pose of robot%1").arg(robot_));
    getPropertyContainer()->subProp("Topic")->setValue(
      QString("/tb4_view/robot%1/initialpose").arg(robot_));
  }
private:
  int robot_;
};
class GoalRobot1 : public RobotGoalTool { public: GoalRobot1():RobotGoalTool(1) {} };
class GoalRobot2 : public RobotGoalTool { public: GoalRobot2():RobotGoalTool(2) {} };
class GoalRobot3 : public RobotGoalTool { public: GoalRobot3():RobotGoalTool(3) {} };
class GoalRobot4 : public RobotGoalTool { public: GoalRobot4():RobotGoalTool(4) {} };
class PoseRobot1 : public RobotPoseTool { public: PoseRobot1():RobotPoseTool(1) {} };
class PoseRobot2 : public RobotPoseTool { public: PoseRobot2():RobotPoseTool(2) {} };
class PoseRobot3 : public RobotPoseTool { public: PoseRobot3():RobotPoseTool(3) {} };
class PoseRobot4 : public RobotPoseTool { public: PoseRobot4():RobotPoseTool(4) {} };
}
PLUGINLIB_EXPORT_CLASS(tb4_multi_bringup::GoalRobot1, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(tb4_multi_bringup::GoalRobot2, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(tb4_multi_bringup::GoalRobot3, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(tb4_multi_bringup::PoseRobot1, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(tb4_multi_bringup::PoseRobot2, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(tb4_multi_bringup::PoseRobot3, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(tb4_multi_bringup::GoalRobot4, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(tb4_multi_bringup::PoseRobot4, rviz_common::Tool)
