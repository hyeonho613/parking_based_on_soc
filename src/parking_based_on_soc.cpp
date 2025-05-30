#include "parking_based_on_soc/parking_based_on_soc.hpp"

ParkingBasedOnSoc::ParkingBasedOnSoc() : Node("parking_based_on_soc")
{
    parking_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/planning/mission_planning/goal", rclcpp::QoS(1));

    soc_status_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/soc_status", 10, std::bind(&ParkingBasedOnSoc::SocStatusCallback, this, std::placeholders::_1));
    operation_mode_sub_ = this->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
        "/system/operation_mode/state", 10, std::bind(&ParkingBasedOnSoc::OperationModeCallback, this, std::placeholders::_1));
    routing_state_sub_ = this->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
        "/api/routing/state", 10, std::bind(&ParkingBasedOnSoc::RoutingStateCallback, this, std::placeholders::_1));

    auto_mode_client_ = this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
        "/api/operation_mode/change_to_autonomous");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ParkingBasedOnSoc::TimerCallback, this));

    soc_status_ = 0.0;
    is_stopped_ = 0;
    goal_msg_ = geometry_msgs::msg::PoseStamped();
    has_published_ = false;
    is_arrived_ = 0;
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
    if(soc_status_ < 0.2 && is_stopped_ == 1) // SOC가 20% 아래로 내려간다면
    {
        goal_msg_.header.stamp = this->now();
        goal_msg_.header.frame_id = "map";

        goal_msg_.pose.position.x = 3725.6572265625;
        goal_msg_.pose.position.y = 73751.59375;
        goal_msg_.pose.position.z = 0.0;

        goal_msg_.pose.orientation.x = 0.0;
        goal_msg_.pose.orientation.y = 0.0;
        goal_msg_.pose.orientation.z = 0.8691726320057526;
        goal_msg_.pose.orientation.w = 0.4945087823003679;

        parking_goal_pub_->publish(goal_msg_);
        RCLCPP_INFO(this->get_logger(), "Published parking goal due to low SOC");

        static int execution_count = 0;

        if (execution_count < 4)
        {
        ChangeOperationMode();
        RCLCPP_INFO(this->get_logger(), "Change Operation Mode : Auto");
        execution_count++;
        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParkingBasedOnSoc>());
    rclcpp::shutdown();
    return 0; 
}

// 문제점 1: auto가 안눌림 - 해결
// 문제점 2: goal 위치까지 가는데 주차를 안함. lanelet에서 parking_space까지 거리가 멀어서? 아니면 렉걸려서?
// auto 눌리는거 해결, but "stop" 해야할때 auto가 계속 눌림
// 