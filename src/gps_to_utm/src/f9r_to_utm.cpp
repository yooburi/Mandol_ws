#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "gps_to_utm/utm_converter.hpp"

class F9rToUtm : public rclcpp::Node
{
public:
    F9rToUtm()
    : Node("f9r_to_utm_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/f9r/fix", 10, std::bind(&F9rToUtm::topic_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/f9r_utm", 10);

        RCLCPP_INFO(this->get_logger(), "f9r_to_utm node has been started.");
    }

private:
    void topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Convert lat/lon to UTM
        utm_converter::UTMCoords utm_coords = utm_converter::toUTM(msg->latitude, msg->longitude);

        if (utm_coords.zone == 0) {
            RCLCPP_WARN(this->get_logger(), "Latitude is out of UTM range (-80 to 84 degrees).");
            return;
        }

        // Create and populate the PointStamped message
        auto utm_msg = std::make_unique<geometry_msgs::msg::PointStamped>();

        utm_msg->header = msg->header; // Copy header from NavSatFix
        utm_msg->header.frame_id = "utm"; // Set a meaningful frame_id

        utm_msg->point.x = utm_coords.easting;  // X (E/m)
        utm_msg->point.y = utm_coords.northing; // Y (N/m)
        utm_msg->point.z = msg->altitude;       // Z (A/m)

        // Publish the message
        publisher_->publish(std::move(utm_msg));

        RCLCPP_DEBUG(this->get_logger(), "Published UTM: X=%.2f, Y=%.2f, Z=%.2f", 
            utm_coords.easting, utm_coords.northing, msg->altitude);
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<F9rToUtm>());
    rclcpp::shutdown();
    return 0;
}
