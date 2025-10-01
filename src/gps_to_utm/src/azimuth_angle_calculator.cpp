#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <cmath>

/**
 * @brief 두 개의 GPS 센서로부터 데이터를 받아 azimuth_angle (Heading)를 계산하는 노드.
 * 
 * - 두 GPS 토픽을 구독 (기본: /f9p/fix, /f9r/fix).
 * - /mission_state 토픽을 구독하여 후진 미션 상태 확인.
 * - 두 메시지의 타임스탬프를 비교하여 동기화.
 * - 위도/경도를 사용하여 두 지점 간의 방위각(azimuth_angle)을 계산.
 * - 후진 미션 시 (REVERSE_T, REVERSE_PARALLEL)에는 각도를 180도 반전.
 * - 계산된 azimuth_angle를 /azimuth_angle 토픽으로 발행.
 */
class AzimuthAngleCalculator : public rclcpp::Node
{
public:
    AzimuthAngleCalculator()
    : Node("azimuth_angle_calculate_node"), current_mission_state_("UNKNOWN")
    {
        // 파라미터 선언 (토픽 이름, 동기화 시간 임계값 등)
        this->declare_parameter<std::string>("gps1_topic", "/f9r/fix"); // 차량 뒤쪽
        this->declare_parameter<std::string>("gps2_topic", "/f9p/fix"); // 차량 앞쪽
        this->declare_parameter<std::string>("yaw_topic", "/azimuth_angle");
        this->declare_parameter<double>("max_time_diff_sec", 0.1);
        this->declare_parameter<std::string>("mission_state_topic", "/mission_state");

        // 파라미터 값 가져오기
        auto gps1_topic = this->get_parameter("gps1_topic").as_string();
        auto gps2_topic = this->get_parameter("gps2_topic").as_string();
        auto yaw_topic = this->get_parameter("yaw_topic").as_string();
        auto mission_state_topic = this->get_parameter("mission_state_topic").as_string();
        max_time_diff_ = this->get_parameter("max_time_diff_sec").as_double();

        // GPS 토픽 구독자 설정
        gps1_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gps1_topic, 10, std::bind(&AzimuthAngleCalculator::gps1_callback, this, std::placeholders::_1));
        
        gps2_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gps2_topic, 10, std::bind(&AzimuthAngleCalculator::gps2_callback, this, std::placeholders::_1));

        // Mission state 구독자 설정
        mission_state_subscription_ = this->create_subscription<std_msgs::msg::String>(
            mission_state_topic, 10, std::bind(&AzimuthAngleCalculator::mission_state_callback, this, std::placeholders::_1));

        // 계산된 Yaw를 발행할 Publisher 설정
        yaw_publisher_ = this->create_publisher<std_msgs::msg::Float64>(yaw_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Azimuth angle Calculator 노드가 시작되었습니다.");
        RCLCPP_INFO(this->get_logger(), "  - GPS 1 Topic: %s", gps1_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  - GPS 2 Topic: %s", gps2_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  - Yaw Topic: %s", yaw_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  - Mission State Topic: %s", mission_state_topic.c_str());
    }

private:
    /**
     * @brief /mission_state 토픽 콜백 함수
     */
    void mission_state_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        current_mission_state_ = msg->data;
    }

    /**
     * @brief GPS1 토픽의 콜백 함수
     */
    void gps1_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        gps1_fix_ = msg;
        try_calculate_yaw();
    }

    /**
     * @brief GPS2 토픽의 콜백 함수
     */
    void gps2_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        gps2_fix_ = msg;
        try_calculate_yaw();
    }

    /**
     * @brief 두 GPS 데이터가 모두 유효할 때 Yaw 계산을 시도
     */
    void try_calculate_yaw()
    {
        if (!gps1_fix_ || !gps2_fix_) {
            return;
        }

        // 시간 동기화 확인
        auto time1 = gps1_fix_->header.stamp.sec + gps1_fix_->header.stamp.nanosec / 1e9;
        auto time2 = gps2_fix_->header.stamp.sec + gps2_fix_->header.stamp.nanosec / 1e9;

        if (std::abs(time1 - time2) > max_time_diff_) {
            RCLCPP_WARN(this->get_logger(), 
                "GPS 타임스탬프 차이가 너무 큽니다: %.4fs. 계산을 건너뜁니다.", std::abs(time1 - time2));
            return;
        }

        // 위도와 경도를 라디안으로 변환
        double lat1 = gps1_fix_->latitude * M_PI / 180.0;
        double lon1 = gps1_fix_->longitude * M_PI / 180.0;
        double lat2 = gps2_fix_->latitude * M_PI / 180.0;
        double lon2 = gps2_fix_->longitude * M_PI / 180.0;

        // 방위각(Bearing) 계산
        // GPS1을 기준점으로 GPS2의 방향을 계산합니다. 
        // f9r이 뒤쪽, f9p가 앞쪽이므로 f9r -> f9p 방향이 차량의 진행 방향
        double dLon = lon2 - lon1;
        double y = std::sin(dLon) * std::cos(lat2);
        double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dLon);
        double bearing_rad = std::atan2(y, x);

        // 라디안을 도로 변환하고 0-360도 범위로 정규화
        double bearing_deg = fmod((bearing_rad * 180.0 / M_PI) + 360.0, 360.0);

        // 미션 상태에 따라 각도 조정
        double final_bearing_deg = bearing_deg;
        if (current_mission_state_ == "REVERSE_T" || current_mission_state_ == "REVERSE_PARALLEL") {
            final_bearing_deg = fmod(bearing_deg + 180.0, 360.0);
        }

        // 결과 발행
        auto yaw_msg = std_msgs::msg::Float64();
        yaw_msg.data = final_bearing_deg;
        yaw_publisher_->publish(yaw_msg);

        if (current_mission_state_ == "REVERSE_T" || current_mission_state_ == "REVERSE_PARALLEL") {
             RCLCPP_INFO(this->get_logger(), "후진 미션 감지. Azimuth Angle 조정됨: %.2f°", final_bearing_deg);
        } else {
             RCLCPP_INFO(this->get_logger(), "계산된 Azimuth_Angle: %.2f°", final_bearing_deg);
        }

        // 한 번 계산에 사용된 데이터는 초기화하여 중복 계산 방지
        gps1_fix_ = nullptr;
        gps2_fix_ = nullptr;
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps1_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps2_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_state_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher_;

    sensor_msgs::msg::NavSatFix::SharedPtr gps1_fix_;
    sensor_msgs::msg::NavSatFix::SharedPtr gps2_fix_;
    double max_time_diff_;
    std::string current_mission_state_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AzimuthAngleCalculator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
