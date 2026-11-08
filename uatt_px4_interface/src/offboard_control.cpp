/**
 * @file offboard_control.cpp
 * @author Jorge Vargas
 * @brief Streams the offboard heartbeat and a fixed position setpoint to PX4 at 10 Hz.
 * @date Nov 8, 2026
 *
 * --------------------------------------------------------
 * Subscription Topics:
 *  None
 * --------------------------------------------------------
 * Publishing Topics:
 *  /fmu/in/offboard_control_mode - px4_msgs/msg/OffboardControlMode
 *  /fmu/in/trajectory_setpoint - px4_msgs/msg/TrajectorySetpoint
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <limits>

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        // PX4 subscribes to /fmu/in/* with best-effort reliability; a best-effort
        // publisher is compatible and avoids useless retransmissions of stale setpoints.
        auto qos = rclcpp::SensorDataQoS();
        
        offboard_control_mode_publisher_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
        trajectory_setpoint_publisher_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);
        
        // PX4 requires the offboard stream at > 2 Hz and exits offboard after
        // COM_OF_LOSS_T (1 s by default) without it. 10 Hz leaves a safe margin.
        timer_ = create_wall_timer(100ms,
            std::bind(&OffboardControl::timerCallback, this)
        );

        RCLCPP_INFO(get_logger(), "Streaming offboard heartbeat and setpoints.");
    }

private:

    void timerCallback()
    {
        // Unused fields MUST be NaN: default-initialized zeros would be interpreted
        // as real setpoints/feedforward (e.g. a position of (0, 0, 0) on the ground).
        const auto nan = std::numeric_limits<float>::quiet_NaN();
        // PX4 expects timestamps in microseconds; one value is shared by both
        // messages since they belong to the same control cycle.
        const uint64_t timestamp_us = get_clock()->now().nanoseconds() / 1000;

        auto mode_msg = OffboardControlMode();
        auto trajectory_msg = TrajectorySetpoint();

        mode_msg.timestamp = timestamp_us;
        mode_msg.position = true;
        mode_msg.velocity = false;
        mode_msg.acceleration = false;
        mode_msg.attitude = false;
        mode_msg.body_rate = false;
        mode_msg.thrust_and_torque = false;
        mode_msg.direct_actuator = false;

        offboard_control_mode_publisher_->publish(mode_msg);        

        trajectory_msg.timestamp = timestamp_us;
        // NED frame: z points down, so -2.0 m means 2 m above the origin.
        trajectory_msg.position = {0.0f, 0.0f, -2.0f}; 
        trajectory_msg.velocity = {nan, nan, nan};
        trajectory_msg.acceleration = {nan, nan, nan};
        trajectory_msg.jerk = {nan, nan, nan};
        // NaN yaw: PX4 keeps the current heading instead of turning to a fixed one.
        trajectory_msg.yaw = nan;
        trajectory_msg.yawspeed = nan;

        trajectory_setpoint_publisher_->publish(trajectory_msg);
    }

    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto offboard_control_node = std::make_shared<OffboardControl>();
    rclcpp::spin(offboard_control_node);

    rclcpp::shutdown();
    return 0;
}