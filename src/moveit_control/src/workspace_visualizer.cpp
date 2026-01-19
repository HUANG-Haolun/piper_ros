#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <random>
#include <moveit/robot_state/conversions.h>     // sometimes useful, optional here
#include <Eigen/Geometry>                         // for Isometry3d

class WorkspaceVisualizer : public rclcpp::Node
{
public:
  WorkspaceVisualizer() : Node("workspace_visualizer")
  {
    // Publisher for visualization markers
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("workspace_marker", 10);

    // Timer to compute and publish workspace periodically
    timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&WorkspaceVisualizer::computeAndPublishWorkspace, this));
  }

  // Call this once after construction (e.g. right after make_shared)
  void initialize()
  {
    // Now safe: the shared_ptr owning "this" already exists
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      shared_from_this(), "robot_description");

    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_) {
      RCLCPP_FATAL(get_logger(), "Failed to load robot model from /robot_description");
      rclcpp::shutdown();
      return;
    }

    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  }

private:
  void computeAndPublishWorkspace()
  {
    // Parameters
    std::vector<std::string> groups = {"left_arm", "right_arm"};
    std::vector<std::string> ee_links = {"left_link6", "right_link6"};  // Replace with actual EE link names from your URDF/SRDF

    for (size_t arm_idx = 0; arm_idx < groups.size(); ++arm_idx)
    {
      const std::string& group_name = groups[arm_idx];
      const std::string& ee_link = ee_links[arm_idx];

      const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name);
      if (!jmg)
      {
        RCLCPP_ERROR(get_logger(), "Invalid group: %s", group_name.c_str());
        continue;
      }

      // Sample points
      std::vector<geometry_msgs::msg::Point> points;
      std::default_random_engine generator;
      collision_detection::CollisionRequest collision_request;
      collision_detection::CollisionResult collision_result;

      const int num_samples = 5000;  // Adjust for density
      for (int i = 0; i < num_samples; ++i)
      {
        moveit::core::RobotState state(robot_model_);
        state.setToRandomPositions(jmg);

        // Check bounds and collisions
        if (state.satisfiesBounds(jmg))
        {
          planning_scene_->checkSelfCollision(collision_request, collision_result, state);
          if (!collision_result.collision)
          {
            state.update();
            Eigen::Isometry3d ee_pose = state.getGlobalLinkTransform(ee_link);
            geometry_msgs::msg::Point point;
            point.x = ee_pose.translation().x();
            point.y = ee_pose.translation().y();
            point.z = ee_pose.translation().z();
            points.push_back(point);
          }
        }
        collision_result.clear();
      }

      // Create marker
      visualization_msgs::msg::Marker marker;
      //marker.header.frame_id = robot_model_->getModelFrame();  // Usually "base_link" or "world"
      marker.header.frame_id = "base_link";  // Usually "base_link" or "world"
      marker.ns = group_name + "_workspace";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.01;  // Point size
      marker.scale.y = 0.01;
      marker.color.a = 0.5;   // Alpha
      marker.color.r = (arm_idx == 0 ? 0.0 : 1.0);  // Red for right, blue for left? Adjust colors
      marker.color.g = 0.0;
      marker.color.b = (arm_idx == 0 ? 1.0 : 0.0);
      marker.points = points;
      marker.lifetime = rclcpp::Duration::from_seconds(0);  // Persistent

      marker_pub_->publish(marker);
      RCLCPP_INFO(get_logger(), "Published workspace for %s with %zu points", group_name.c_str(), points.size());
    }
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WorkspaceVisualizer>();
  node->initialize();  // ← crucial line
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}