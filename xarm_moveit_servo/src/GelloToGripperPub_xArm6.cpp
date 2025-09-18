// Subscribe to a gripper width percentage and command MoveIt gripper targets
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <dag_interfaces/msg/float32_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "gello_to_moveit_gripper",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("gello_to_moveit_gripper");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group_interface(node, "xarm_gripper");

    // Make motions as fast as allowed by limits
    move_group_interface.setMaxVelocityScalingFactor(1.0);
    move_group_interface.setMaxAccelerationScalingFactor(1.0);
    move_group_interface.setNumPlanningAttempts(1);
    // Slightly relaxed joint tolerance can help finish sooner (tune as needed)
    move_group_interface.setGoalJointTolerance(0.01);

    // Track last commanded state to avoid redundant plans
    // true = closed, false = open, std::nullopt = unknown/not set yet
    std::optional<bool> last_is_closed = std::nullopt;

    // Subscription callback to command gripper based on threshold
    auto sub = node->create_subscription<dag_interfaces::msg::Float32Stamped>(
        "gello/gripper_width_percent", 10,
        [&](const dag_interfaces::msg::Float32Stamped & msg)
        {
            const float v = msg.data;
            // Below threshold -> close, above threshold -> open.
            // At exactly 0.5, keep current state.
            if (v < 0.5f) {
                if (!last_is_closed.has_value() || !(*last_is_closed)) {
                    RCLCPP_INFO(logger, "Gripper width %.3f < 0.5 -> closing", v);
                    move_group_interface.setNamedTarget("close");
                    (void)move_group_interface.move();
                    last_is_closed = true;
                }
            } else if (v > 0.5f) {
                if (!last_is_closed.has_value() || *last_is_closed) {
                    RCLCPP_INFO(logger, "Gripper width %.3f > 0.5 -> opening", v);
                    move_group_interface.setNamedTarget("open");
                    (void)move_group_interface.move();
                    last_is_closed = false;
                }
            } else {
                // v == 0.5f, do nothing
            }
        }
    );

    // Spin to process incoming messages
    rclcpp::spin(node);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}