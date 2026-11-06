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
        
        subscription_ = create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v1", qos,
            [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg)
            {
                RCLCPP_INFO(
                    this->get_logger(), 
                    "arming_state: %i nav_state: %i failsafe: %i pre_flight_checks_pass: %i gcs_connection_lost: %i latest_arming_reason: %i latest_disarming_reason: %i",
                    msg->arming_state,
                    msg->nav_state,
                    msg->failsafe,
                    msg->pre_flight_checks_pass,
                    msg->gcs_connection_lost,
                    msg->latest_arming_reason,
                    msg->latest_disarming_reason
                );
            }
        );
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr subscription_;
};



 int main(int argc, char * argv[])
 {
    rclcpp::init(argc, argv); // Initialize ROS 2

    auto vehicle_status_monitor_node = std::make_shared<VehicleStatusMonitor>(); // Create an instance of the VehicleStatusMonitor node
    rclcpp::spin(vehicle_status_monitor_node); // Keep the node running

    rclcpp::shutdown(); // Shutdown ROS 2
    return 0;
 }