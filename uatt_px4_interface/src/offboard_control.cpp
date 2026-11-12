/**
 * @file offboard_control.cpp
 * @author Jorge Vargas
 * @brief Runs an offboard mission state machine: requests offboard mode, arms,
 *        takes off to 2 m above the start position, flies a 5 m square in the
 *        north-east plane and lands back at the start position.
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
#include <array>
#include <cmath>

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

    // Vertical distance to the takeoff target at which the climb is considered done.
    constexpr float kAltitudeToleranceM = 0.2f;

    // Flight altitude above the start position for takeoff and waypoints.
    constexpr float kTakeoffAltitudeM = 2.0f;

    // Side length of the square flown in the north-east plane.
    constexpr float kSquareSideM = 5.0f;

    // 3D distance to the current waypoint at which it is considered reached.
    constexpr float kWaypointToleranceM = 0.3f;

    enum class MissionState
    {
        WaitingForSetpoints,   // Stream setpoints before requesting offboard
        RequestingOffboard,    // Send DO_SET_MODE until PX4 confirms OFFBOARD
        Arming,                // Send ARM until PX4 confirms ARMED
        TakingOff,             // Climb to the takeoff altitude above the start position
        FollowingWaypoints,    // Fly through the square waypoints
        RequestingLanding,     // Send VEHICLE_CMD_NAV_LAND until PX4 confirms AUTO_LAND
        Landing,               // PX4 is landing; wait for it to touch down and disarm
        Done,                  // Mission completed; node idle
        Aborted                // PX4 left OFFBOARD; stop commanding, PX4 has control
    };

    const char * missionStateToString(MissionState state)
    {
        switch (state)
        {
            case MissionState::WaitingForSetpoints:
                return "WaitingForSetpoints";
            case MissionState::RequestingOffboard:
                return "RequestingOffboard";
            case MissionState::Arming:
                return "Arming";
            case MissionState::TakingOff:
                return "TakingOff";
            case MissionState::FollowingWaypoints:
                return "FollowingWaypoints";
            case MissionState::RequestingLanding:
                return "RequestingLanding";
            case MissionState::Landing:
                return "Landing";
            case MissionState::Done:
                return "Done";
            case MissionState::Aborted:
                return "Aborted";
        }
        return "UNKNOWN";
    }
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
        
        offboard_control_mode_publisher_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
        trajectory_setpoint_publisher_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);
        vehicle_command_publisher_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);        
        
        // PX4 requires the offboard stream at > 2 Hz and exits offboard after
        // COM_OF_LOSS_T (1 s by default) without it. 10 Hz leaves a safe margin.
        timer_ = create_wall_timer(100ms,
            std::bind(&OffboardControl::timerCallback, this)
        );

        RCLCPP_INFO(get_logger(), "Streaming offboard heartbeat and setpoints.");
    }

private:

    void vehicleStatusCallback(const VehicleStatus::UniquePtr msg)
    {
        vehicle_status_msg_ = *msg;
    }

    void localPositionCallback(const VehicleLocalPosition::UniquePtr msg)
    {
        vehicle_local_position_msg_ = *msg;
    }

    void timerCallback()
    {        
        // One value is shared by both messages since they belong to the same control cycle.
        const uint64_t timestamp_us = timestampUs();    

        const MissionState state_before = state_;      

        switch (state_)
        {
            case MissionState::WaitingForSetpoints:
                if (isReadyToStart())
                {
                    start_x_ = vehicle_local_position_msg_->x;
                    start_y_ = vehicle_local_position_msg_->y;
                    start_z_ = vehicle_local_position_msg_->z;
                    
                    // Computed once from the start position, so the square is anchored where
                    // the vehicle took off. NED frame: +x north, +y east, z down.
                    // Square: north, north-east, east, back to start.
                    waypoints_[0] = {start_x_ + kSquareSideM, 
                                     start_y_, 
                                     start_z_ - kTakeoffAltitudeM};
                    waypoints_[1] = {start_x_ + kSquareSideM, 
                                     start_y_ + kSquareSideM, 
                                     start_z_ - kTakeoffAltitudeM};
                    waypoints_[2] = {start_x_, 
                                     start_y_ + kSquareSideM, 
                                     start_z_ - kTakeoffAltitudeM};
                    waypoints_[3] = {start_x_, 
                                     start_y_, 
                                     start_z_ - kTakeoffAltitudeM};

                    target_ = {start_x_, start_y_, start_z_};
                    transitionTo(MissionState::RequestingOffboard);
                }
                else if (hasValidPosition())
                {
                    target_ = {vehicle_local_position_msg_->x,
                               vehicle_local_position_msg_->y,
                               vehicle_local_position_msg_->z};
                }
                break;

            case MissionState::RequestingOffboard:
                if (state_cycles_ % 10 == 0)
                {                    
                    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 
                                          kCustomModeEnabled, 
                                          kPx4MainModeOffboard);
                }
                if (vehicle_status_msg_.has_value() &&
                    vehicle_status_msg_->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD)
                {
                    transitionTo(MissionState::Arming);
                }
                break;

            case MissionState::Arming:
                if (state_cycles_ % 10 == 0)
                {                    
                    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 
                                          kArm);
                }
                if (vehicle_status_msg_.has_value() &&
                    vehicle_status_msg_->arming_state == VehicleStatus::ARMING_STATE_ARMED)
                {
                    // NED frame: z points down, so subtracting the altitude places the
                    // target above the start position.
                    target_ = {start_x_, start_y_, start_z_ - kTakeoffAltitudeM};
                    transitionTo(MissionState::TakingOff);
                }
                break;

            case MissionState::TakingOff:
                if (vehicle_status_msg_.has_value() &&
                    vehicle_status_msg_->nav_state != VehicleStatus::NAVIGATION_STATE_OFFBOARD)
                {
                    transitionTo(MissionState::Aborted);
                }
                else if (hasValidPosition() &&
                         std::abs(vehicle_local_position_msg_->z - target_[2]) < kAltitudeToleranceM)
                {
                    // On entry to FollowingWaypoints: start from the first corner of the square.
                    waypoint_index_ = 0;                    
                    target_ = waypoints_[waypoint_index_];
                    transitionTo(MissionState::FollowingWaypoints);
                }
                break;

            case MissionState::FollowingWaypoints:
                if (vehicle_status_msg_.has_value() &&
                    vehicle_status_msg_->nav_state != VehicleStatus::NAVIGATION_STATE_OFFBOARD)
                {
                    transitionTo(MissionState::Aborted);
                }
                else if (hasValidPosition())
                {
                    const float distance_m = std::hypot(vehicle_local_position_msg_->x - target_[0],
                                                        vehicle_local_position_msg_->y - target_[1],
                                                        vehicle_local_position_msg_->z - target_[2]);

                    // Advance to the next waypoint once within tolerance; land after the last one.
                    if (distance_m < kWaypointToleranceM)
                    {           
                        RCLCPP_INFO(get_logger(), "Reached waypoint %zu/%zu", 
                                    waypoint_index_ + 1, waypoints_.size());

                        waypoint_index_++;

                        if (waypoint_index_ == waypoints_.size())
                        {
                            transitionTo(MissionState::RequestingLanding);
                        }
                        else
                        {                            
                            target_ = waypoints_[waypoint_index_];
                        }                    
                    }                  
                }
                break;

            case MissionState::RequestingLanding:
                if (state_cycles_ % 10 == 0)
                {                    
                    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND);
                }
                if (vehicle_status_msg_.has_value() &&
                    vehicle_status_msg_->nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LAND)
                {
                    transitionTo(MissionState::Landing);
                }
                break;
                
            case MissionState::Landing:
                if (vehicle_status_msg_.has_value() &&
                    vehicle_status_msg_->arming_state == VehicleStatus::ARMING_STATE_DISARMED)
                {
                    transitionTo(MissionState::Done);
                }
                break;

            case MissionState::Done:
            case MissionState::Aborted:
                // Terminal states: PX4 has control, nothing left to do.
                break;
        }

        if (isCommandingVehicle())
        {
            // Stream the heartbeat and the current target only while this node is in control.
            // In Landing, Done and Aborted PX4 has control, so the stream stops.
            publishHeartbeat(timestamp_us);

            if (hasValidPosition())
            {
                publishPositionSetpoint(timestamp_us, target_);
            }
        }        

        // Don’t count the cycle in which a transition happened, so the new
        // state starts at 0 and on-entry actions run on its first cycle.
        if (state_ == state_before)
        {
            state_cycles_++;
        }
    }

    void publishHeartbeat(const uint64_t timestamp_us)
    {
        auto mode_msg = OffboardControlMode();        

        mode_msg.timestamp = timestamp_us;
        mode_msg.position = true;
        mode_msg.velocity = false;
        mode_msg.acceleration = false;
        mode_msg.attitude = false;
        mode_msg.body_rate = false;
        mode_msg.thrust_and_torque = false;
        mode_msg.direct_actuator = false;

        offboard_control_mode_publisher_->publish(mode_msg);
    }

    void publishPositionSetpoint(const uint64_t timestamp_us, const std::array<float, 3> & position)
    {
        // Unused fields MUST be NaN: default-initialized zeros would be interpreted
        // as real setpoints/feedforward (e.g. a position of (0, 0, 0) on the ground).
        const auto nan = std::numeric_limits<float>::quiet_NaN();

        auto trajectory_msg = TrajectorySetpoint();

        trajectory_msg.timestamp = timestamp_us;
        // Position in the NED frame (z points down).
        trajectory_msg.position = position;
        trajectory_msg.velocity = {nan, nan, nan};
        trajectory_msg.acceleration = {nan, nan, nan};
        trajectory_msg.jerk = {nan, nan, nan};
        // NaN yaw: PX4 keeps the current heading instead of turning to a fixed one.
        trajectory_msg.yaw = nan;
        trajectory_msg.yawspeed = nan;

        trajectory_setpoint_publisher_->publish(trajectory_msg);
    }

    void publishVehicleCommand(uint32_t command, float param1=0.0f, float param2=0.0f)
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

        RCLCPP_DEBUG(get_logger(), 
                     "Sent vehicle command %u (param1=%.1f, param2=%.1f)", 
                     command, param1, param2);
    }

    void transitionTo(MissionState next)
    {
        RCLCPP_INFO(get_logger(),
                    "State: %s -> %s", 
                    missionStateToString(state_), 
                    missionStateToString(next));
        state_ = next;
        state_cycles_ = 0;
    }    

    uint64_t timestampUs()
    {
        // PX4 expects timestamps in microseconds.
        return get_clock()->now().nanoseconds() / 1000;
    }

    bool isReadyToStart() const
    {
        return state_cycles_ >= 10 &&
               vehicle_status_msg_.has_value() &&
               vehicle_status_msg_->arming_state == VehicleStatus::ARMING_STATE_DISARMED &&
               hasValidPosition();
    }

    bool hasValidPosition() const
    {
        return vehicle_local_position_msg_.has_value() &&
               vehicle_local_position_msg_->xy_valid &&
               vehicle_local_position_msg_->z_valid;
    }

    bool isCommandingVehicle() const
    {
        switch (state_)
        {
            case MissionState::WaitingForSetpoints:
            case MissionState::RequestingOffboard:
            case MissionState::Arming:
            case MissionState::TakingOff:
            case MissionState::FollowingWaypoints:
            case MissionState::RequestingLanding:
                return true;

            case MissionState::Landing:
            case MissionState::Done:
            case MissionState::Aborted:
                return false;
        }

        return false;
    }

    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscription_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;    
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;    

    rclcpp::TimerBase::SharedPtr timer_;

    MissionState state_ = MissionState::WaitingForSetpoints;
    std::optional<VehicleStatus> vehicle_status_msg_;
    std::optional<VehicleLocalPosition> vehicle_local_position_msg_;

    int state_cycles_ = 0;

    float start_x_ = 0.0f;
    float start_y_ = 0.0f;
    float start_z_ = 0.0f;

    std::array<float, 3> target_{};
    std::array<std::array<float, 3>, 4> waypoints_{};
    std::size_t waypoint_index_ = 0;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto offboard_control_node = std::make_shared<OffboardControl>();
    rclcpp::spin(offboard_control_node);

    rclcpp::shutdown();
    return 0;
}