#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <sstream>
#include <filesystem>

// 탐지 영역 정보를 담는 구조체
struct DetectionArea
{
    std::string name; // 영역 이름 (토픽 생성에 사용)
    double x;
    double y;
    double radius;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_publisher; // 시각화 마커 퍼블리셔
    bool last_state;
};

class CsvRadiusDetector : public rclcpp::Node
{
public:
    CsvRadiusDetector()
    : Node("csv_radius_detector_node")
    {
        // tf_gps_csv.cpp와 동일한 파라미터 이름을 사용하여 원점 CSV 파일 경로를 공유
        this->declare_parameter<std::string>("csv_file_path", "");

        // CSV 파일에서 원점 정보를 로드
        load_origin_from_csv();

        // UTM 좌표를 수신하는 구독자
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/f9r_utm", 10, std::bind(&CsvRadiusDetector::utm_callback, this, std::placeholders::_1));

        // 파라미터로부터 영역을 정의하고 해당 퍼블리셔들을 생성
        initialize_areas_from_params();

        // 1Hz 주기로 시각화 마커를 발행하는 타이머 생성
        viz_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CsvRadiusDetector::publish_markers, this));

        RCLCPP_INFO(this->get_logger(), "csv_radius_detector_node has been started.");
    }

private:
    void load_origin_from_csv()
    {
        std::string csv_file = this->get_parameter("csv_file_path").as_string();
        if (csv_file.empty() || !std::filesystem::exists(csv_file))
        {
            RCLCPP_ERROR(this->get_logger(), "Origin CSV file not found or path is empty: %s", csv_file.c_str());
            return;
        }

        std::ifstream file(csv_file);
        std::string line;
        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string segment;
            std::vector<std::string> seglist;
            while(std::getline(ss, segment, ','))
            {
                seglist.push_back(segment);
            }
            try
            {
                if (seglist.size() >= 2) {
                    origin_x_ = std::stod(seglist[0]);
                    origin_y_ = std::stod(seglist[1]);
                    RCLCPP_INFO(this->get_logger(), "CSV origin for visualization set to: x=%.2f, y=%.2f", origin_x_.value(), origin_y_.value());
                    return;
                } 
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(this->get_logger(), "Could not parse row for origin, trying next. Line: %s. Error: %s", line.c_str(), e.what());
            }
        }
        RCLCPP_ERROR(this->get_logger(), "No valid origin point found in CSV file: %s", csv_file.c_str());
    }

    void initialize_areas_from_params()
    {
        this->declare_parameter<std::vector<std::string>>("area_names", std::vector<std::string>());
        std::vector<std::string> area_names = this->get_parameter("area_names").as_string_array();

        RCLCPP_INFO(this->get_logger(), "Loading %zu detection areas from parameters...", area_names.size());

        for (const auto& name : area_names)
        {
            std::string state_topic = this->declare_parameter<std::string>(name + ".state_topic", "");
            std::string viz_topic = this->declare_parameter<std::string>(name + ".viz_topic", "");
            double x = this->declare_parameter<double>(name + ".x", 0.0);
            double y = this->declare_parameter<double>(name + ".y", 0.0);
            double radius = this->declare_parameter<double>(name + ".radius", 0.0);

            if (state_topic.empty() || viz_topic.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Skipping area '%s' due to empty topic names.", name.c_str());
                continue;
            }

            areas_.push_back({name, x, y, radius,
                              this->create_publisher<std_msgs::msg::Bool>(state_topic, 10),
                              this->create_publisher<visualization_msgs::msg::Marker>(viz_topic, 10),
                              false});
            RCLCPP_INFO(this->get_logger(), "  - Loaded area: %s (x: %.2f, y: %.2f, r: %.2f)", name.c_str(), x, y, radius);
        }
    }

    void utm_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        double current_x = msg->point.x;
        double current_y = msg->point.y;

        for (auto& area : areas_)
        {
            double dx = current_x - area.x;
            double dy = current_y - area.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            bool current_state = (distance <= area.radius);

            auto bool_msg = std::make_unique<std_msgs::msg::Bool>();
            bool_msg->data = current_state;
            area.state_publisher->publish(std::move(bool_msg));

            if (current_state != area.last_state)
            {
                RCLCPP_INFO(this->get_logger(), "Topic '%s' state changed to %s (distance: %.2f, radius: %.2f)",
                            area.state_publisher->get_topic_name(), current_state ? "true" : "false", distance, area.radius);
                area.last_state = current_state;
            }
        }
    }

    void publish_markers()
    {
        if (!origin_x_.has_value()) {
            return; 
        }

        const double origin_x = origin_x_.value();
        const double origin_y = origin_y_.value();

        int id_counter = 0;
        const std::vector<std::array<float, 4>> colors = {
            {{1.0f, 0.0f, 0.0f, 0.5f}}, // Red
            {{1.0f, 1.0f, 0.0f, 0.5f}}, // Yellow
            {{0.0f, 1.0f, 0.0f, 0.5f}}, // Green
            {{0.0f, 0.0f, 1.0f, 0.5f}}, // Blue
            {{1.0f, 0.0f, 1.0f, 0.5f}}  // Magenta
        };

        for (const auto& area : areas_)
        {
            visualization_msgs::msg::Marker cylinder_marker;
            cylinder_marker.header.frame_id = "csv";
            cylinder_marker.header.stamp = this->now();
            cylinder_marker.ns = area.name + "_area";
            cylinder_marker.id = id_counter;
            cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            cylinder_marker.action = visualization_msgs::msg::Marker::ADD;

            cylinder_marker.pose.position.x = area.x - origin_x;
            cylinder_marker.pose.position.y = area.y - origin_y;
            cylinder_marker.pose.position.z = -0.5;
            cylinder_marker.pose.orientation.w = 1.0;

            cylinder_marker.scale.x = area.radius * 2.0;
            cylinder_marker.scale.y = area.radius * 2.0;
            cylinder_marker.scale.z = 0.1;

            cylinder_marker.color.r = colors[id_counter % colors.size()][0];
            cylinder_marker.color.g = colors[id_counter % colors.size()][1];
            cylinder_marker.color.b = colors[id_counter % colors.size()][2];
            cylinder_marker.color.a = 0.20;

            cylinder_marker.lifetime = rclcpp::Duration::from_seconds(1.5);
            area.viz_publisher->publish(cylinder_marker);

            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = "csv";
            text_marker.header.stamp = this->now();
            text_marker.ns = area.name + "_label";
            text_marker.id = id_counter;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;

            text_marker.pose.position.x = area.x - origin_x;
            text_marker.pose.position.y = area.y - origin_y;
            text_marker.pose.position.z = 1.0;
            
            text_marker.scale.z = 0.8;

            text_marker.color.r = 1.0f;
            text_marker.color.g = 1.0f;
            text_marker.color.b = 1.0f;
            text_marker.color.a = 1.0f;

            text_marker.text = area.name;
            text_marker.lifetime = rclcpp::Duration::from_seconds(1.5);
            area.viz_publisher->publish(text_marker);

            id_counter++;
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    std::vector<DetectionArea> areas_;
    rclcpp::TimerBase::SharedPtr viz_timer_;
    std::optional<double> origin_x_;
    std::optional<double> origin_y_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CsvRadiusDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}