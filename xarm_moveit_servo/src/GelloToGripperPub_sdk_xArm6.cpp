// Subscribe to a gripper width percentage and command xArm gripper services directly
#include <memory>
#include <optional>
#include <utility>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <dag_interfaces/msg/float32_stamped.hpp>

// xArm service interfaces
#include <xarm_msgs/srv/gripper_move.hpp>
#include <xarm_msgs/srv/set_int16.hpp>
#include <xarm_msgs/srv/set_float32.hpp>

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "gello_to_xarm_gripper",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("gello_to_xarm_gripper");

    // Create gripper-related service clients
    using xarm_msgs::srv::GripperMove;
    using xarm_msgs::srv::SetInt16;
    using xarm_msgs::srv::SetFloat32;

    auto gripper_enable_cli = node->create_client<SetInt16>("/xarm/set_gripper_enable");
    auto gripper_mode_cli   = node->create_client<SetInt16>("/xarm/set_gripper_mode");
    auto gripper_speed_cli  = node->create_client<SetFloat32>("/xarm/set_gripper_speed");
    auto gripper_pos_cli    = node->create_client<GripperMove>("/xarm/set_gripper_position");

    auto wait_for = [&](auto &cli, const char *name) {
        if (!cli->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(logger, "Service %s not available after waiting", name);
            return false;
        }
        return true;
    };

    // Wait for services
    if (!wait_for(gripper_enable_cli, "/xarm/set_gripper_enable") ||
        !wait_for(gripper_mode_cli,   "/xarm/set_gripper_mode")   ||
        !wait_for(gripper_speed_cli,  "/xarm/set_gripper_speed")  ||
        !wait_for(gripper_pos_cli,    "/xarm/set_gripper_position"))
    {
        rclcpp::shutdown();
        return 1;
    }

    auto call_sync = [&](auto &cli, auto request, const char *op_name) -> bool {
        auto future = cli->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(logger, "Failed to call %s (no response)", op_name);
            return false;
        }
        auto resp = future.get();
        int ret = resp->ret;  // xarm_msgs responses have 'ret'
        if (ret != 0) {
            RCLCPP_WARN(logger, "%s returned ret=%d", op_name, ret);
        }
        return ret == 0;
    };

    // Enable gripper
    {
        auto req = std::make_shared<SetInt16::Request>();
        req->data = 1;
        if (!call_sync(gripper_enable_cli, req, "set_gripper_enable(1)")) {
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(logger, "xArm gripper enabled");
    }

    // Set gripper mode to 0 (position)
    {
        auto req = std::make_shared<SetInt16::Request>();
        req->data = 0;
        if (!call_sync(gripper_mode_cli, req, "set_gripper_mode(0)")) {
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(logger, "xArm gripper mode set to 0 (Position)");
    }

    // Set gripper speed to 1500 (consistent with policy runner)
    {
        auto req = std::make_shared<SetFloat32::Request>();
        req->data = 1500.0f;
        if (!call_sync(gripper_speed_cli, req, "set_gripper_speed(1500)")) {
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(logger, "xArm gripper speed set to 1500");
    }

    // Track last commanded state to avoid redundant requests
    // true = closed, false = open, std::nullopt = unknown/not set yet
    std::optional<bool> last_is_closed = std::nullopt;

    auto send_gripper_pos_async = [&](float pos, const char *reason) {
        auto req = std::make_shared<GripperMove::Request>();
        req->pos = pos;  // 0 = close, 850 = open for xArm API
        auto future = gripper_pos_cli->async_send_request(req);
        (void)future; // optionally attach a callback if needed
        RCLCPP_INFO(logger, "GripperMove pos=%.1f (%s)", pos, reason);
    };

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
                    send_gripper_pos_async(0.0f, "close");
                    last_is_closed = true;
                }
            } else if (v > 0.5f) {
                if (!last_is_closed.has_value() || *last_is_closed) {
                    send_gripper_pos_async(850.0f, "open");
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