#include <rviz_default_plugins/tools/goal_pose/goal_tool.hpp>
#include <rviz_default_plugins/tools/point/point_tool.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool_manager.hpp>
#include <QTimer>
namespace tb4_multi_bringup {
#define GOAL_TOOL(Type, Label, Key) \
class Type : public rviz_default_plugins::tools::GoalTool { \
public: Type() {shortcut_key_=Key;} \
void onInitialize() override {GoalTool::onInitialize();setName(Label);} };
#define POINT_TOOL(Type, Label, Key) \
class Type : public rviz_default_plugins::tools::PointTool { \
public: Type() {shortcut_key_=Key;} \
void onInitialize() override {PointTool::onInitialize();setName(Label);} \
void activate() override { \
geometry_msgs::msg::PointStamped msg;msg.header.frame_id="map";msg.header.stamp=clock_->now(); \
publisher_->publish(msg);setStatus(Label " requested"); \
QTimer::singleShot(0,this,[this](){auto manager=context_->getToolManager(); \
manager->setCurrentTool(manager->getDefaultTool());}); } \
int processMouseEvent(rviz_common::ViewportMouseEvent &) override {return Finished;} };
GOAL_TOOL(Robot1Goal, "R1 Goal", '1')
GOAL_TOOL(Robot2Goal, "R2 Goal", '2')
POINT_TOOL(Preview, "MARCO Preview", 'p')
POINT_TOOL(Execute, "MARCO Execute", 'e')
POINT_TOOL(Cancel, "MARCO Cancel", 'c')
}
PLUGINLIB_EXPORT_CLASS(tb4_multi_bringup::Robot1Goal, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(tb4_multi_bringup::Robot2Goal, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(tb4_multi_bringup::Preview, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(tb4_multi_bringup::Execute, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(tb4_multi_bringup::Cancel, rviz_common::Tool)
