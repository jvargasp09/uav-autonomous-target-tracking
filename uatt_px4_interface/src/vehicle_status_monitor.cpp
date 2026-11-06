/**
 * @file vehicle_status_monitor.cpp
 * @author Jorge Vargas
 * @brief Monitors the status of the vehicle
 * @date Nov 5, 2026
 * 
 * --------------------------------------------------------
 * Subscription Topics:
 *  /fmu/out/vehicle_status_v1 - px4_msgs/msg/VehicleStatus
 * --------------------------------------------------------
 * Publishing Topics:
 *  None
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace
{
    const char * armingStateToString(uint8_t state)
    {
        using px4_msgs::msg::VehicleStatus;

        switch (state) 
        {
            case VehicleStatus::ARMING_STATE_DISARMED:
                return "DISARMED";
            case VehicleStatus::ARMING_STATE_ARMED:
                return "ARMED";
            default:
                return "UNKNOWN";
        }
    }

    const char * navStateToString(uint8_t state)
    {
        using px4_msgs::msg::VehicleStatus;

        switch (state)
        {
            case VehicleStatus::NAVIGATION_STATE_MANUAL:
                return "MANUAL";
            case VehicleStatus::NAVIGATION_STATE_ALTCTL:
                return "ALTCTL";
            case VehicleStatus::NAVIGATION_STATE_POSCTL:
                return "POSCTL";
            case VehicleStatus::NAVIGATION_STATE_AUTO_MISSION:
                return "AUTO_MISSION";
            case VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
                return "AUTO_LOITER";
            case VehicleStatus::NAVIGATION_STATE_AUTO_RTL:
                return "AUTO_RTL";
            case VehicleStatus::NAVIGATION_STATE_POSITION_SLOW:
                return "POSITION_SLOW";
            case VehicleStatus::NAVIGATION_STATE_ALTITUDE_CRUISE:
                return "ALTITUDE_CRUISE";
            case VehicleStatus::NAVIGATION_STATE_ACRO:
                return "ACRO";
            case VehicleStatus::NAVIGATION_STATE_DESCEND:
                return "DESCEND";
            case VehicleStatus::NAVIGATION_STATE_TERMINATION:
                return "TERMINATION";
            case VehicleStatus::NAVIGATION_STATE_OFFBOARD:
                return "OFFBOARD";
            case VehicleStatus::NAVIGATION_STATE_STAB:
                return "STAB";
            case VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
                return "AUTO_TAKEOFF";
            case VehicleStatus::NAVIGATION_STATE_AUTO_LAND:
                return "AUTO_LAND";
            case VehicleStatus::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
                return "AUTO_FOLLOW_TARGET";
            case VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND:
                return "AUTO_PRECLAND";
            case VehicleStatus::NAVIGATION_STATE_ORBIT:
                return "ORBIT";
            case VehicleStatus::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
                return "AUTO_VTOL_TAKEOFF";
            case VehicleStatus::NAVIGATION_STATE_EXTERNAL1:
                return "EXTERNAL1";
            case VehicleStatus::NAVIGATION_STATE_EXTERNAL2:
                return "EXTERNAL2";
            case VehicleStatus::NAVIGATION_STATE_EXTERNAL3:
                return "EXTERNAL3";
            case VehicleStatus::NAVIGATION_STATE_EXTERNAL4:
                return "EXTERNAL4";
            case VehicleStatus::NAVIGATION_STATE_EXTERNAL5:
                return "EXTERNAL5";
            case VehicleStatus::NAVIGATION_STATE_EXTERNAL6:
                return "EXTERNAL6";
            case VehicleStatus::NAVIGATION_STATE_EXTERNAL7:
                return "EXTERNAL7";
            case VehicleStatus::NAVIGATION_STATE_EXTERNAL8:
                return "EXTERNAL8";
            default:
                return "UNKNOWN";
        }
    }

    const char * armDisarmReasonToString(uint8_t reason)
    {
        using px4_msgs::msg::VehicleStatus;

        switch (reason)
        {
            case VehicleStatus::ARM_DISARM_REASON_TRANSITION_TO_STANDBY:
                return "TRANSITION_TO_STANDBY";
            case VehicleStatus::ARM_DISARM_REASON_STICK_GESTURE:
                return "STICK_GESTURE";
            case VehicleStatus::ARM_DISARM_REASON_RC_SWITCH:
                return "RC_SWITCH";
            case VehicleStatus::ARM_DISARM_REASON_COMMAND_INTERNAL:
                return "COMMAND_INTERNAL";
            case VehicleStatus::ARM_DISARM_REASON_COMMAND_EXTERNAL:
                return "COMMAND_EXTERNAL";
            case VehicleStatus::ARM_DISARM_REASON_MISSION_START:
                return "MISSION_START";
            case VehicleStatus::ARM_DISARM_REASON_SAFETY_BUTTON:
                return "SAFETY_BUTTON";
            case VehicleStatus::ARM_DISARM_REASON_AUTO_DISARM_LAND:
                return "AUTO_DISARM_LAND";
            case VehicleStatus::ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT:
                return "AUTO_DISARM_PREFLIGHT";
            case VehicleStatus::ARM_DISARM_REASON_KILL_SWITCH:
                return "KILL_SWITCH";
            case VehicleStatus::ARM_DISARM_REASON_LOCKDOWN:
                return "LOCKDOWN";
            case VehicleStatus::ARM_DISARM_REASON_FAILURE_DETECTOR:
                return "FAILURE_DETECTOR";
            case VehicleStatus::ARM_DISARM_REASON_SHUTDOWN:
                return "SHUTDOWN";
            case VehicleStatus::ARM_DISARM_REASON_UNIT_TEST:
                return "UNIT_TEST";
            default:
                return "UNKNOWN";
        }
    }

    using ToStringFn = const char * (*)(uint8_t);

    const char * boolToString(uint8_t value)
    {
        return value ? "YES" : "NO";
    }
}  

class VehicleStatusMonitor : public rclcpp::Node
{
public: 

    explicit VehicleStatusMonitor() : Node("vehicle_status_monitor")
    {

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        subscription_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", 
            qos, 
            std::bind(&VehicleStatusMonitor::onVehicleStatus, this, std::placeholders::_1)
        );
    }

private:   
    
    enum class LogLevel {Info, Warn, Error};

    void onVehicleStatus(const px4_msgs::msg::VehicleStatus::UniquePtr msg)
    {
        
        if (is_first_msg_)
        {
            RCLCPP_INFO(
                get_logger(), 
                "arming_state: %s | nav_state: %s | failsafe: %s | pre_flight_checks_pass: %s | gcs_connection_lost: %s | latest_arming_reason: %s | latest_disarming_reason: %s",
                armingStateToString(msg->arming_state),
                navStateToString(msg->nav_state),
                boolToString(msg->failsafe),
                boolToString(msg->pre_flight_checks_pass),
                boolToString(msg->gcs_connection_lost),
                armDisarmReasonToString(msg->latest_arming_reason),
                armDisarmReasonToString(msg->latest_disarming_reason)
            );

            is_first_msg_ = false;
        }
        else
        {
            logIfChanged("arming_state", last_status_.arming_state, msg->arming_state, armingStateToString);
            logIfChanged("nav_state", last_status_.nav_state, msg->nav_state, navStateToString);
            logIfChanged("failsafe", last_status_.failsafe, msg->failsafe, boolToString, LogLevel::Warn);
            logIfChanged("pre_flight_checks_pass", last_status_.pre_flight_checks_pass, msg->pre_flight_checks_pass, boolToString);
            logIfChanged("gcs_connection_lost", last_status_.gcs_connection_lost, msg->gcs_connection_lost, boolToString);
            logIfChanged("latest_arming_reason", last_status_.latest_arming_reason, msg->latest_arming_reason, armDisarmReasonToString);
            logIfChanged("latest_disarming_reason", last_status_.latest_disarming_reason, msg->latest_disarming_reason, armDisarmReasonToString);
        }

        last_status_ = *msg;       
        
    }

    void logIfChanged(const char * field_name, uint8_t old_value, uint8_t new_value,
                  ToStringFn to_string, LogLevel level = LogLevel::Info)
    {
        if (old_value == new_value) 
        {
            return;
        }

        switch (level) 
        {
            case LogLevel::Info:
                RCLCPP_INFO(get_logger(), "%s: %s -> %s", field_name, to_string(old_value), to_string(new_value));
                break;
            case LogLevel::Warn:
                RCLCPP_WARN(get_logger(), "%s: %s -> %s", field_name, to_string(old_value), to_string(new_value));
                break;
            case LogLevel::Error:
                RCLCPP_ERROR(get_logger(), "%s: %s -> %s", field_name, to_string(old_value), to_string(new_value));
                break;
        }
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr subscription_;

    px4_msgs::msg::VehicleStatus last_status_;
    bool is_first_msg_ = true;
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto vehicle_status_monitor_node = std::make_shared<VehicleStatusMonitor>(); 
    rclcpp::spin(vehicle_status_monitor_node);

    rclcpp::shutdown();
    return 0;
}