#include "parking_based_on_soc/parking_based_on_soc.hpp"

ParkingBasedOnSoc::ParkingBasedOnSoc() : Node("parking_based_on_soc")
{
    parking_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/planning/mission_planning/goal", rclcpp::QoS(1));

    soc_status_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/soc_can", 10, std::bind(&ParkingBasedOnSoc::SocStatusCallback, this, std::placeholders::_1));
    operation_mode_sub_ = this->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
        "/system/operation_mode/state", 10, std::bind(&ParkingBasedOnSoc::OperationModeCallback, this, std::placeholders::_1));
    routing_state_sub_ = this->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
        "/api/routing/state", 10, std::bind(&ParkingBasedOnSoc::RoutingStateCallback, this, std::placeholders::_1));
    position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/localization/kinematic_state", 10, std::bind(&ParkingBasedOnSoc::PositionCallback, this, std::placeholders::_1));

    auto_mode_client_ = this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
        "/api/operation_mode/change_to_autonomous");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ParkingBasedOnSoc::TimerCallback, this));

    soc_status_ = 0.0;
    is_stopped_ = 0;
    goal_msg_ = geometry_msgs::msg::PoseStamped();
    has_published_ = false;
    is_arrived_ = 0;
    auto_mode_attempts_ = 0;
    position_ = geometry_msgs::msg::Point();
}

void ParkingBasedOnSoc::SocStatusCallback(const std_msgs::msg::Float32 msg)
{
    soc_status_ = msg.data;
}

void ParkingBasedOnSoc::OperationModeCallback(const autoware_adapi_v1_msgs::msg::OperationModeState msg)
{
    is_stopped_ = msg.mode;
}

void ParkingBasedOnSoc::RoutingStateCallback(const autoware_adapi_v1_msgs::msg::RouteState msg)
{
    is_arrived_ = msg.state;
}

void ParkingBasedOnSoc::PositionCallback(const nav_msgs::msg::Odometry msg)
{
    position_ = msg.pose.pose.position; 
}

void ParkingBasedOnSoc::ChangeOperationMode()
{
    std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>request = 
      std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
    std::shared_future<std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response>>result = 
      auto_mode_client_->async_send_request(request);
    // auto future_and_request_id = auto_mode_client_->async_send_request(request);
    // auto future = future_and_request_id.future.share();
}

void ParkingBasedOnSoc::TimerCallback()
{
    if (soc_status_ > 0.2 && has_published_)
    {
        has_published_ = false;
        auto_mode_attempts_ = 0; // 선택적으로 시도 횟수도 리셋하여 새 모드 변경 허용
        RCLCPP_INFO(this->get_logger(), "SOC > 0.2, reset");
    }

    if(soc_status_ < 0.2 && is_stopped_ == 1 && is_arrived_ == 3 && std::abs(11354.0-position_.x)<10.0 && std::abs(90650.0-position_.y)<10.0)
    {
        if(!has_published_) // 주차장 목표가 아직 설정되지 않았다면
        {
            goal_msg_.header.stamp = this->now(); // 주차장을 목적지로 설정
            goal_msg_.header.frame_id = "map";

            goal_msg_.pose.position.x = 11310.415702509279;
            goal_msg_.pose.position.y = 90811.31663179968;
            goal_msg_.pose.position.z = 80.73453034766041;

            goal_msg_.pose.orientation.x = 0.011767542269303335;
            goal_msg_.pose.orientation.y = -0.01644872365889284;
            goal_msg_.pose.orientation.z = 0.5817232134648086;
            goal_msg_.pose.orientation.w = 0.8131353315132175;
            
            parking_goal_pub_->publish(goal_msg_);
            RCLCPP_INFO(this->get_logger(), "Goal_Pose is Parking_space");
            has_published_ = true;
        }
    }

    if(has_published_ && (is_arrived_ == 3 || is_arrived_ == 2) && auto_mode_attempts_ < 5) 
    {
        ChangeOperationMode();
        RCLCPP_INFO(this->get_logger(), "Change Operation Mode : Auto");
        auto_mode_attempts_++;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParkingBasedOnSoc>());
    rclcpp::shutdown();
    return 0; 
}
