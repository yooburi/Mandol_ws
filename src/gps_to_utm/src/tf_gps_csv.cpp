#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h" // For setRPY
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // For toMsg() if needed, or direct assignment
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <cmath> // For M_PI, sin, cos, radians
#include <filesystem> // For std::filesystem::exists
#include <algorithm> // For std::min, std::max

/* 보간 없는 버전 ---------------------------- */

// Helper function to convert Euler angles to a tf2::Quaternion
tf2::Quaternion quaternion_from_euler(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}

class TfGpsCsvNode : public rclcpp::Node
{
public:
    TfGpsCsvNode() : Node("tf_gps_csv_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("csv_file_path", "");
        
        // Initialize TF Broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Initialize data storage
        origin_x_ = std::nullopt; // Use std::nullopt for optional doubles
        origin_y_ = std::nullopt;
        current_yaw_radians_ = 0.0;

        // Create Subscriptions
        f9r_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/f9r_utm", 10, std::bind(&TfGpsCsvNode::f9r_callback, this, std::placeholders::_1));
        f9p_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/f9p_utm", 10, std::bind(&TfGpsCsvNode::f9p_callback, this, std::placeholders::_1));
        yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/azimuth_angle", 10, std::bind(&TfGpsCsvNode::yaw_callback, this, std::placeholders::_1));

        // Create Publishers
        // For Transient Local QoS: ensures the last message is saved for late-joining subscribers
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.transient_local();
        qos_profile.reliable();
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/csv_path", qos_profile);
        yaw_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/azimuth_angle_text", 10);

        // Load CSV path once
        load_csv_path();

        // Timer to continuously publish CSV path
        // 변경 (50=0.05sec = 20 Hz)
        path_publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&TfGpsCsvNode::publish_path_callback, this));
    }

private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::optional<double> origin_x_; // Using std::optional for nullable doubles
    std::optional<double> origin_y_;
    double current_yaw_radians_;
    nav_msgs::msg::Path path_msg_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr f9r_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr f9p_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr yaw_marker_pub_;
    rclcpp::TimerBase::SharedPtr path_publish_timer_;

    void load_csv_path()
    {
        std::string csv_file = this->get_parameter("csv_file_path").as_string();
        if (!std::filesystem::exists(csv_file))
        {
            RCLCPP_ERROR(this->get_logger(), "CSV file not found at: %s", csv_file.c_str());
            return;
        }

        std::ifstream file(csv_file);
        std::string line;
        int i = 0;
        
        path_msg_.header.frame_id = "csv";
        path_msg_.poses.clear();

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
                if (seglist.size() < 2) {
                    RCLCPP_WARN(this->get_logger(), "Skipping row %d (not enough columns): %s", i, line.c_str());
                    i++;
                    continue;
                }
                double x = std::stod(seglist[0]);
                double y = std::stod(seglist[1]);

                if (!origin_x_.has_value()) {
                    origin_x_ = x;
                    origin_y_ = y;
                    RCLCPP_INFO(this->get_logger(), "CSV origin set to: x=%.2f, y=%.2f", origin_x_.value(), origin_y_.value());
                }

                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "csv";
                pose.pose.position.x = x - origin_x_.value();
                pose.pose.position.y = y - origin_y_.value();
                pose.pose.position.z = 0.0;
                path_msg_.poses.push_back(pose);

            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(this->get_logger(), "Could not parse row %d (likely header): %s. Error: %s", i, line.c_str(), e.what());
            }
            i++;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu points from CSV.", path_msg_.poses.size());

        if (path_msg_.poses.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No valid points loaded from CSV.");
            return;
        }
    }

    void publish_path_callback()
    {
        if (!path_msg_.poses.empty())
        {
            path_msg_.header.stamp = this->get_clock()->now();
            for (auto& pose : path_msg_.poses)
            {
                pose.header.stamp = path_msg_.header.stamp;
            }
            path_pub_->publish(path_msg_);
        }
    }

    void yaw_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Yaw from North (degrees) to ROS standard yaw (radians from East)
        // ROS yaw: 0 is +X (East), increases counter-clockwise.
        // North is +Y. So, 90 degrees from North (clockwise) is East.
        // 90 - msg.data converts North-based clockwise degrees to East-based counter-clockwise degrees.
        current_yaw_radians_ = (90.0 - msg->data) * M_PI / 180.0;

        // --- NEW FEATURE: Publish yaw as a text marker ---
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "f9r"; // Anchor the marker to the f9r frame
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "azimuth_angle_text";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position the marker to the right of the f9r frame origin
        // In ROS, Y is typically left, so a negative Y is to the right.
        marker.pose.position.x = -1.0;
        marker.pose.position.y = -0.5; // 50cm to the right
        marker.pose.position.z = 0.5;  // 50cm up

        // Text properties
        marker.scale.z = 0.4; // Text height
        marker.color.a = 1.0; // Alpha (must be non-zero)
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0; // White
        marker.text = "/azimuth_angle " + std::to_string(msg->data) + "°";

        yaw_marker_pub_->publish(marker);
    }

    void f9r_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (!origin_x_.has_value())
        {
            return;
        }
        broadcast_transform(msg, "f9r");
    }

    void f9p_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (!origin_x_.has_value())
        {
            return;
        }
        broadcast_transform(msg, "f9p");
    }

    void broadcast_transform(const geometry_msgs::msg::PointStamped::SharedPtr msg, const std::string& child_frame_id)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "csv"; // Parent frame for the GPS sensors
        t.child_frame_id = child_frame_id; // Child frame (f9r or f9p)

        // Translate the GPS point relative to the CSV origin
        t.transform.translation.x = msg->point.x - origin_x_.value();
        t.transform.translation.y = msg->point.y - origin_y_.value();
        t.transform.translation.z = 0.0; // Assuming 2D for now

        // Apply the current yaw to the transform
        tf2::Quaternion q = quaternion_from_euler(0, 0, current_yaw_radians_);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfGpsCsvNode>());
    rclcpp::shutdown();
    return 0;
}