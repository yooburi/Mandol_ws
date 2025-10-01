#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <vector>
#include <limits>
#include <cmath>
#include <memory>
#include <string>
#include <chrono>
#include <algorithm>
#include <mutex>

using std::placeholders::_1;

struct Pt { double x; double y; };

class ROIPathPublisher : public rclcpp::Node
{
public:
  ROIPathPublisher()
  : Node("f9r_roi_path"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    last_start_idx_(0),
    last_end_idx_(0),
    published_once_(false),
    has_csv_path_(false)
  {
    // ====== Parameters ======
    target_frame_      = this->declare_parameter<std::string>("target_frame", "f9r");   // /f9r_roi_path 시작점 선정 기준 프레임(기준 원점)
    csv_frame_         = this->declare_parameter<std::string>("csv_frame", "csv");      // CSV(UTM m) 프레임
    timer_frequency_   = this->declare_parameter<double>("timer_frequency", 20.0);
    roi_length_m_      = this->declare_parameter<double>("roi_length_m", 4.0);          // ROI 길이 [m]
    use_points_length_ = this->declare_parameter<bool>("use_points_length", false);     // ROI 길이를 포인트 개수로 자를지 여부. 만약 false면 자동으로 roi_length_m 사용
    roi_length_pts_    = this->declare_parameter<int>("roi_length_points", 50);         // use_points_length=true일 때 ROI에 포함할 점의 개수. 
    search_span_pts_   = this->declare_parameter<int>("search_span_points", 2000);      // 초기 전방 인덱스 탐색 폭
    hysteresis_k_      = this->declare_parameter<int>("hysteresis_k", 0);               // 점프 완화 (/f9r_roi_path 시작점 선정 시 자잘한 떨림에 의한 변화 방지. 0~3 정도가 적정값. 클수록 방지 정도가 커짐)
    line_width_        = this->declare_parameter<double>("line_width", 0.3);
    end_marker_size_   = this->declare_parameter<double>("end_marker_size", 0.4);

    // ====== Subscriptions (transient_local) ======
    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
        "/csv_path",
        rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&ROIPathPublisher::pathCallback, this, _1));

    sub_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/f9r/fix",
        rclcpp::QoS(10).best_effort(),
        std::bind(&ROIPathPublisher::fixCallback, this, _1));

    // ====== Publishers (transient_local) ======
    auto qos_tl = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    roi_path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/f9r_roi_path", qos_tl);
    roi_end_pub_  = this->create_publisher<visualization_msgs::msg::Marker>("/f9r_roi_end",  qos_tl);

    // ====== Timer ======
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1e-3, timer_frequency_)),
      std::bind(&ROIPathPublisher::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
      "[f9r_roi_path] csv_frame=%s, target=%s, freq=%.1f Hz, ROI=%.2f %s, search_span_pts=%d",
      csv_frame_.c_str(), target_frame_.c_str(), timer_frequency_,
      use_points_length_ ? (double)roi_length_pts_ : roi_length_m_,
      use_points_length_ ? "pts" : "m", search_span_pts_);
  }

private:
  // ===========================
  // Callbacks
  // ===========================
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    // 새 경로를 로컬 버퍼에 파싱
    std::vector<Pt> new_pts;
    new_pts.reserve(msg->poses.size());
    for (const auto & ps : msg->poses) {
      new_pts.push_back(Pt{ps.pose.position.x, ps.pose.position.y});
    }

    std::lock_guard<std::mutex> lk(state_mtx_);

    const bool was_empty = csv_pts_.empty();
    const size_t oldN = csv_pts_.size();

    // 간단 변경 감지
    bool changed = was_empty || (oldN != new_pts.size());
    if (!changed && !csv_pts_.empty()) {
      auto diff = [](const Pt& a, const Pt& b){
        return std::fabs(a.x - b.x) > 1e-6 || std::fabs(a.y - b.y) > 1e-6;
      };
      const size_t probes[5] = {0, oldN/4, oldN/2, (3*oldN)/4, oldN ? oldN-1 : 0};
      for (size_t k = 0; k < 5 && k < oldN; ++k) {
        const size_t i = std::min(probes[k], oldN - 1);
        if (diff(csv_pts_[i], new_pts[i])) { changed = true; break; }
      }
    }

    csv_pts_.swap(new_pts);
    has_csv_path_ = !csv_pts_.empty();

    if (!has_csv_path_) {
      RCLCPP_WARN(this->get_logger(), "/csv_path is empty");
      return;
    }

    if (changed) {
      const size_t N = csv_pts_.size();
      if (last_start_idx_ >= N) last_start_idx_ = N - 1;
      if (last_end_idx_   >= N) last_end_idx_   = N - 1;
      if (last_end_idx_ < last_start_idx_) last_end_idx_ = last_start_idx_;
      RCLCPP_INFO(this->get_logger(),
                  "csv_path changed: size=%zu (clamped last_s=%zu, last_e=%zu)",
                  N, last_start_idx_, last_end_idx_);
    }

    RCLCPP_INFO(this->get_logger(), "Received /csv_path: %zu points, frame=%s (changed=%s)",
                csv_pts_.size(), msg->header.frame_id.c_str(), changed ? "true":"false");
  }

  void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr /*msg*/)
  {
    // 실제 좌표 변환은 timerCallback에서 TF로 수행
  }

  void timerCallback()
  {
    // 상태를 로컬 복사 (락 범위 최소화)
    std::vector<Pt> csv_local;
    size_t last_s_local, last_e_local;
    bool published_once_local;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      if (!has_csv_path_ || csv_pts_.size() < 2) return;
      csv_local = csv_pts_;
      last_s_local = last_start_idx_;
      last_e_local = last_end_idx_;
      published_once_local = published_once_;
    }

    // 로봇 현재 위치 (target_frame 원점) -> csv 프레임으로 변환
    geometry_msgs::msg::PointStamped robot_in_target;
    robot_in_target.header.frame_id = target_frame_;
    robot_in_target.header.stamp = this->get_clock()->now();
    robot_in_target.point.x = 0.0;
    robot_in_target.point.y = 0.0;
    robot_in_target.point.z = 0.0;

    geometry_msgs::msg::PointStamped robot_in_csv;
    try {
      tf_buffer_.transform(robot_in_target, robot_in_csv, csv_frame_, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF transform %s->%s failed: %s",
                           target_frame_.c_str(), csv_frame_.c_str(), ex.what());
      return;
    }

    const double xf = robot_in_csv.point.x;
    const double yf = robot_in_csv.point.y;

    // ====== 시작점 후보 탐색 범위 결정 ======
    // 1) 첫 퍼블리시 전: 기존 방식 (전방 탐색)
    // 2) 그 이후: "직전 ROI 위의 점들"로만 제한 → [last_s_local, last_e_local]
    const size_t N = csv_local.size();
    size_t cand_begin, cand_end;

    if (!published_once_local) {
      cand_begin = std::min(last_s_local, N - 1);
      cand_end   = std::min(cand_begin + static_cast<size_t>(std::max(0, search_span_pts_)), N - 1);
    } else {
      cand_begin = std::min(last_s_local, N - 1);
      cand_end   = std::min(last_e_local, N - 1);
      if (cand_end < cand_begin) cand_end = cand_begin; // 안정성
    }

    // ====== 최근접 인덱스 탐색 (f9r 실제 위치 기준) ======
    size_t nearest_idx = cand_begin;
    double best_d2 = std::numeric_limits<double>::infinity();

    for (size_t i = cand_begin; i <= cand_end; ++i) {
      const double dx = csv_local[i].x - xf;
      const double dy = csv_local[i].y - yf;
      const double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        nearest_idx = i;
      }
    }

    // ====== 단조성 + 히스테리시스 ======
    size_t proposed = nearest_idx;
    if (hysteresis_k_ > 0 && proposed < last_s_local + static_cast<size_t>(hysteresis_k_)) {
      proposed = last_s_local;
    }
    const size_t start_idx = std::max(proposed, last_s_local);
    if (start_idx >= N) return;

    // ====== ROI 끝 인덱스 계산 ======
    size_t end_idx = start_idx;
    if (use_points_length_) {
      const size_t w = static_cast<size_t>(std::max(1, roi_length_pts_));
      end_idx = std::min(start_idx + w, N - 1);
    } else {
      const double Wm = std::max(0.1, roi_length_m_);
      double acc = 0.0;
      size_t i = start_idx;
      while (i + 1 < N) {
        const double dx = csv_local[i+1].x - csv_local[i].x;
        const double dy = csv_local[i+1].y - csv_local[i].y;
        acc += std::hypot(dx, dy);
        ++i;
        if (acc >= Wm) break;
      }
      end_idx = i;
    }

    // ====== Publish ======
    publishPathMarker(start_idx, end_idx, csv_local);
    publishEndMarker(end_idx, csv_local);

    // ====== 상태 커밋 ======
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      const size_t N_cur = csv_pts_.size();
      const size_t s_clamped = std::min(start_idx, (N_cur ? N_cur - 1 : 0));
      const size_t e_clamped = std::min(end_idx,   (N_cur ? N_cur - 1 : 0));
      last_start_idx_ = s_clamped;
      last_end_idx_   = std::max(e_clamped, s_clamped); // 안전
      published_once_ = true;
    }
  }

  // ===========================
  // Marker helpers (로컬 벡터 사용)
  // ===========================
  void publishPathMarker(size_t s, size_t e, const std::vector<Pt>& csv_pts)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = csv_frame_;
    m.header.stamp = this->get_clock()->now();
    m.ns = "f9r_roi";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = line_width_;
    m.color.a = 0.4f;
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    m.pose.orientation.w = 1.0;

    if (e >= s) {
      m.points.reserve(e - s + 1);
      for (size_t i = s; i <= e; ++i) {
        geometry_msgs::msg::Point p;
        p.x = csv_pts[i].x;
        p.y = csv_pts[i].y;
        p.z = 0.0;
        m.points.push_back(p);
      }
    }

    roi_path_pub_->publish(m);
  }

  void publishEndMarker(size_t idx, const std::vector<Pt>& csv_pts)
  {
    if (idx >= csv_pts.size()) return;

    visualization_msgs::msg::Marker m;
    m.header.frame_id = csv_frame_;
    m.header.stamp = this->get_clock()->now();
    m.ns = "f9r_roi_end";
    m.id = 1;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = end_marker_size_;
    m.scale.y = end_marker_size_;
    m.scale.z = end_marker_size_;
    m.color.a = 1.0f;
    m.color.r = 1.0f;
    m.color.g = 0.1f;
    m.color.b = 0.1f;
    m.pose.orientation.w = 1.0;
    m.pose.position.x = csv_pts[idx].x;
    m.pose.position.y = csv_pts[idx].y;
    m.pose.position.z = 0.0;

    roi_end_pub_->publish(m);
  }

private:
  // ====== TF ======
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ====== Params ======
  std::string target_frame_;
  std::string csv_frame_;
  double timer_frequency_;
  double roi_length_m_;
  bool   use_points_length_;
  int    roi_length_pts_;
  int    search_span_pts_;
  int    hysteresis_k_;
  double line_width_;
  double end_marker_size_;

  // ====== State ======
  std::vector<Pt> csv_pts_;
  size_t last_start_idx_;
  size_t last_end_idx_;
  bool   published_once_;
  bool   has_csv_path_;
  std::mutex state_mtx_;

  // ====== ROS IO ======
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr         sub_path_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr roi_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr roi_end_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROIPathPublisher>());
  rclcpp::shutdown();
  return 0;
}
