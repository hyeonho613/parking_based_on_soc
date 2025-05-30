#ifndef PARKING_BASED_ON_SOC_HPP_
#define PARKING_BASED_ON_SOC_HPP_

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_adapi_v1_msgs/msg/route_state.hpp"
#include "autoware_adapi_v1_msgs/srv/change_operation_mode.hpp"

class ParkingBasedOnSoc : public rclcpp::Node
{
public:
    ParkingBasedOnSoc();

private:
    void SocStatusCallback(const std_msgs::msg::Float32 msg);
    void OperationModeCallback(const autoware_adapi_v1_msgs::msg::OperationModeState msg);
    void RoutingStateCallback(const autoware_adapi_v1_msgs::msg::RouteState msg);

    void ChangeOperationMode();

    void TimerCallback();


    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr parking_goal_pub_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr soc_status_sub_;
    rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr operation_mode_sub_;
    rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr routing_state_sub_;

    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr auto_mode_client_;

    rclcpp::TimerBase::SharedPtr timer_;

    float soc_status_;
    geometry_msgs::msg::PoseStamped goal_msg_;
    bool has_published_;
    uint8_t is_stopped_;
    uint16_t is_arrived_;

    int auto_mode_attempts_;
};

#endif