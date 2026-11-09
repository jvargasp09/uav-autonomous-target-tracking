/**
 * @file offboard_control.cpp
 * @author Jorge Vargas
 * @brief Switches PX4 to offboard mode, arms the vehicle and holds a fixed
 *        position setpoint (2 m above the origin) at 10 Hz.
 * @date Nov 8, 2026
 *
 * --------------------------------------------------------
 * Subscription Topics:
 *  /fmu/out/vehicle_status_v1 - px4_msgs/msg/VehicleStatus
 *  /fmu/out/vehicle_local_position_v1 - px4_msgs/msg/VehicleLocalPosition
 * --------------------------------------------------------
 * Publishing Topics:
 *  /fmu/in/offboard_control_mode - px4_msgs/msg/OffboardControlMode
 *  /fmu/in/trajectory_setpoint - px4_msgs/msg/TrajectorySetpoint
 *  /fmu/in/vehicle_command - px4_msgs/msg/VehicleCommand
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <limits>
#include <optional>

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleLocalPosition;

using namespace std::chrono_literals;

namespace
{
    // MAVLink MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: param1 of DO_SET_MODE selects PX4 custom modes.
    constexpr float kCustomModeEnabled = 1.0f;
    // PX4_CUSTOM_MAIN_MODE_OFFBOARD (src/modules/commander/px4_custom_mode.h): param2 of DO_SET_MODE.
    constexpr float kPx4MainModeOffboard = 6.0f;
    // param1 of VEHICLE_CMD_COMPONENT_ARM_DISARM.
    constexpr float kArm = 1.0f;

    // MAVLink IDs of the target vehicle (MAV_SYS_ID) and its autopilot component.
    // Hardcoded for a single simulated vehicle; to be replaced by a ROS 2 parameter.
    constexpr uint8_t kVehicleSystemId = 1;
    constexpr uint8_t kAutopilotComponentId = 1;
}


class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        // PX4's uXRCE-DDS bridge is best-effort in both directions (/fmu/in/* subscribers
        // and /fmu/out/* publishers). SensorDataQoS is compatible with both and avoids
        // retransmitting stale setpoints.
        auto qos = rclcpp::SensorDataQoS();
        
        offboard_control_mode_publisher_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
        trajectory_setpoint_publisher_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);
        vehicle_command_publisher_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);

        vehicle_status_subscription_ = create_subscription<VehicleStatus>(
            "/fmu/out/vehicle_status_v1", 
            qos, 
            std::bind(&OffboardControl::vehicleStatusCallback, this, std::placeholders::_1)
        );
        vehicle_local_position_subscription_ = create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", 
            qos, 
            std::bind(&OffboardControl::localPositionCallback, this, std::placeholders::_1)
        );
        
        // PX4 requires the offboard stream at > 2 Hz and exits offboard after
        // COM_OF_LOSS_T (1 s by default) without it. 10 Hz leaves a safe margin.
        timer_ = create_wall_timer(100ms,
            std::bind(&OffboardControl::timerCallback, this)
        );

        RCLCPP_INFO(get_logger(), "Streaming offboard heartbeat and setpoints.");
    }

private:

    enum class MissionState
    {
        WaitingForSetpoints,   // Stream setpoints before requesting offboard
        RequestingOffboard,    // Send DO_SET_MODE until PX4 confirms OFFBOARD
        Arming,                // Send ARM until PX4 confirms ARMED
        TakingOff,             // Climb to the takeoff altitude above the start position
        FollowingWaypoints,    // Fly through the square waypoints
        Landing,               // NAV_LAND sent; wait for PX4 to land and disarm
        Done,                  // Mission completed; node idle
        Aborted                // PX4 left OFFBOARD; stop commanding, PX4 has control
    };

    void timerCallback()
    {
        // Unused fields MUST be NaN: default-initialized zeros would be interpreted
        // as real setpoints/feedforward (e.g. a position of (0, 0, 0) on the ground).
        const auto nan = std::numeric_limits<float>::quiet_NaN();
        // One value is shared by both messages since they belong to the same control cycle.
        const uint64_t timestamp_us = timestampUs();

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
        
        // PX4 only accepts the switch to offboard if setpoints are already streaming,
        // so wait ~1 s of setpoints before sending the mode and arm commands.
        if (!offboard_cmd_sent_ || !arm_cmd_sent_) 
        {
            if (setpoint_cycles_ > 10)
            {
                if (!offboard_cmd_sent_)
                {
                    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, kCustomModeEnabled, kPx4MainModeOffboard);
                    offboard_cmd_sent_ = true;
                }
                else if (!arm_cmd_sent_)
                {
                    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, kArm);
                    arm_cmd_sent_ = true;
                }
            }
            else 
            {
                setpoint_cycles_++;
            }
        }
    }

    void publishVehicleCommand(uint32_t command, float param1, float param2=0.0f)
    {        
        auto vehicle_cmd_msg = VehicleCommand();

        vehicle_cmd_msg.timestamp = timestampUs();
        vehicle_cmd_msg.command = command;
        vehicle_cmd_msg.param1 = param1;
        vehicle_cmd_msg.param2 = param2;
        vehicle_cmd_msg.target_system = kVehicleSystemId;
        vehicle_cmd_msg.target_component = kAutopilotComponentId;
        vehicle_cmd_msg.source_system = kVehicleSystemId;
        vehicle_cmd_msg.source_component = kAutopilotComponentId;
        vehicle_cmd_msg.from_external = true;

        vehicle_command_publisher_->publish(vehicle_cmd_msg);
    }

    void vehicleStatusCallback(const VehicleStatus::UniquePtr msg)
    {
        vehicle_status_msg_ = *msg;
    }

    void localPositionCallback(const VehicleLocalPosition::UniquePtr msg)
    {
        vehicle_local_position_msg_ = *msg;
    }

    uint64_t timestampUs()
    {
        // PX4 expects timestamps in microseconds.
        return get_clock()->now().nanoseconds() / 1000;
    }

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;    
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscription_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;

    rclcpp::TimerBase::SharedPtr timer_;

    int setpoint_cycles_ = 0;
    bool offboard_cmd_sent_ = false;
    bool arm_cmd_sent_ = false;

    MissionState state_ = MissionState::WaitingForSetpoints;
    std::optional<VehicleStatus> vehicle_status_msg_;
    std::optional<VehicleLocalPosition> vehicle_local_position_msg_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto offboard_control_node = std::make_shared<OffboardControl>();
    rclcpp::spin(offboard_control_node);

    rclcpp::shutdown();
    return 0;
}