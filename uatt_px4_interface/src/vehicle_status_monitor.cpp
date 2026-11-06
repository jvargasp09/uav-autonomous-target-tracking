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
                "arming_state: %i | nav_state: %i | failsafe: %i | pre_flight_checks_pass: %i | gcs_connection_lost: %i | latest_arming_reason: %i | latest_disarming_reason: %i",
                msg->arming_state,
                msg->nav_state,
                msg->failsafe,
                msg->pre_flight_checks_pass,
                msg->gcs_connection_lost,
                msg->latest_arming_reason,
                msg->latest_disarming_reason
            );

            is_first_msg_ = false;
        }
        else
        {
            logIfChanged("arming_state", last_status_.arming_state, msg->arming_state);
            logIfChanged("nav_state", last_status_.nav_state, msg->nav_state);
            logIfChanged("failsafe", last_status_.failsafe, msg->failsafe, LogLevel::Warn);
            logIfChanged("pre_flight_checks_pass", last_status_.pre_flight_checks_pass, msg->pre_flight_checks_pass);
            logIfChanged("gcs_connection_lost", last_status_.gcs_connection_lost, msg->gcs_connection_lost);
            logIfChanged("latest_arming_reason", last_status_.latest_arming_reason, msg->latest_arming_reason);
            logIfChanged("latest_disarming_reason", last_status_.latest_disarming_reason, msg->latest_disarming_reason);
        }

        last_status_ = *msg;       
        
    }

    void logIfChanged(const char* field_name, int old_value, int new_value, LogLevel level = LogLevel::Info)
    {        
        if(old_value != new_value)
        {
            switch (level)
            {
                case LogLevel::Info:
                    RCLCPP_INFO(get_logger(), "%s: %i -> %i", field_name, old_value, new_value);
                    break;
                case LogLevel::Warn:
                    RCLCPP_WARN(get_logger(), "%s: %i -> %i", field_name, old_value, new_value);
                    break;
                case LogLevel::Error:
                    RCLCPP_ERROR(get_logger(), "%s: %i -> %i", field_name, old_value, new_value);
                    break;
            }   
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