#include <memory>
#include <future>  // Added for std::future
#include <thread>  // Added for std::thread
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_control",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_control");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;

  // Create MoveGroup interfaces for both arms 
  MoveGroupInterface left_arm(node, "left_arm"); 
  MoveGroupInterface right_arm(node, "right_arm"); 

  // Define two target poses (forward and back) 
  auto make_pose = [](double x, double y, double z) { 
    geometry_msgs::msg::Pose msg; 
    msg.orientation.w = 1.0; // neutral orientation 
    msg.position.x = x; 
    msg.position.y = y; 
    msg.position.z = z; 
    return msg; 
  }; 
    
  geometry_msgs::msg::Pose left_forward = make_pose(0.5, 0.0, 0.58); 
  geometry_msgs::msg::Pose left_backward = make_pose(0.5, 0.0, 0.18); 
  
  geometry_msgs::msg::Pose right_forward = make_pose(-0.5, 0.0, 0.58); 
  geometry_msgs::msg::Pose right_backward = make_pose(-0.5, 0.0, 0.18); 
  
  // Loop back and forth 
  for (int i = 0; rclcpp::ok(); ++i) { 
    bool forward = (i % 2 == 0); 

    auto left_target = forward ? left_forward : left_backward; 
    auto right_target = forward ? right_forward : right_backward; 
    
    left_arm.setPoseTarget(left_target); 
    right_arm.setPoseTarget(right_target); 
    
    // Plan for both arms 
    MoveGroupInterface::Plan left_plan, right_plan; 
    bool left_ok = static_cast<bool>(left_arm.plan(left_plan)); 
    bool right_ok = static_cast<bool>(right_arm.plan(right_plan)); 
    
    if (left_ok && right_ok) {
      // Execute both arms in parallel using separate threads for synchronous execution
      std::thread left_thread([&left_arm, &left_plan, &logger]() {
        auto result = left_arm.execute(left_plan);
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          RCLCPP_ERROR(logger, "Left arm execution failed!");
        }
      });
      std::thread right_thread([&right_arm, &right_plan, &logger]() {
        auto result = right_arm.execute(right_plan);
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          RCLCPP_ERROR(logger, "Right arm execution failed!");
        }
      });

      // Wait for both threads to complete
      left_thread.join();
      right_thread.join();
    } else {
      RCLCPP_ERROR(logger, "Planning failed for one or both arms!");
    }

    // Small pause between cycles 
    rclcpp::sleep_for(std::chrono::seconds(2)); 
  }


  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
