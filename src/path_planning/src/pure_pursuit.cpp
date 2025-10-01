// pure_pursuit.cpp (곡률 적응형 Ld, 지수 형태, 모든 시각화는 CSV 프레임 기준, 단일 TF 조회)
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <vector>
#include <cmath>
#include <limits>
#include <string>
#include <optional>
#include <algorithm>
#include <memory>

class PurePursuitNode : public rclcpp::Node {
public:
  PurePursuitNode()
  : Node("pure_pursuit_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    kappa_filt_(0.0),
    ld_filt_(std::nullopt),
    throttle_filt_(std::nullopt) // NEW: 스로틀 필터 상태 초기화
  {
    // ------------------------------
    // 파라미터 선언 (Parameters)
    // ------------------------------
    input_marker_topic_   = this->declare_parameter<std::string>("input_marker_topic", "/f9r_roi_path"); // 입력 경로 토픽
    base_frame_           = this->declare_parameter<std::string>("base_frame", "f9r");   // 차량 기준 프레임
    fallback_path_frame_  = this->declare_parameter<std::string>("path_frame", "csv");  // 경로 마커의 frame_id가 비었을 경우 사용할 기본 프레임
    wheelbase_            = this->declare_parameter<double>("wheelbase", 0.724);          // 차량 축거 (Wheelbase) [m]

    // --- 전방 주시 거리 (Lookahead Distance) ---
    fixed_ld_             = this->declare_parameter<double>("fixed_lookahead", 3.0);    // 고정 Ld (디버깅/로깅용)
    min_ld_               = this->declare_parameter<double>("min_lookahead", 1.5);      // 최소 Ld
    max_ld_               = this->declare_parameter<double>("max_lookahead", 4.0);      // 최대 Ld
    back_ld_              = this->declare_parameter<double>("back_ld_", 2.0);           // 후진 시 고정 Ld

    // --- 곡률 적응형 Ld (지수 형태) ---
    beta_                 = this->declare_parameter<double>("beta", 4.0);               // ★ 지수 감쇠 계수 (값이 클수록 곡률 변화에 민감)
    curvature_window_     = this->declare_parameter<int>("curvature_window", 7);        // 곡률 계산 시 사용할 주변 점 개수 (홀수 권장)
    curvature_ema_alpha_  = this->declare_parameter<double>("curvature_ema_alpha", 0.7);// 곡률 값에 대한 지수이동평균(EMA) 필터 계수 (0~1)
    ld_ema_alpha_         = this->declare_parameter<double>("ld_ema_alpha", 0.5);       // Ld 값에 대한 EMA 필터 계수 (0~1)

    // --- 조향각 출력 (Steering Output) ---
    steer_cmd_topic_      = this->declare_parameter<std::string>("steer_cmd_topic", "/auto_steer_angle"); // 조향각 명령 토픽
    steer_is_degree_      = this->declare_parameter<bool>("steer_is_degree", true);     // 조향각 단위가 도(degree)인지 여부
    steer_limit_deg_      = this->declare_parameter<double>("steer_limit_deg", 18.0);   // 조향각 제한 (도 단위)

    // --- 스로틀 출력 (Throttle Output, NEW) ---
    throttle_from_planning_topic_ = this->declare_parameter<std::string>("throttle_from_planning_topic", "/throttle_from_planning"); // 스로틀 명령 토픽
    min_throttle_         = this->declare_parameter<double>("min_throttle", 0.4);       // 최소 스로틀
    max_throttle_         = this->declare_parameter<double>("max_throttle", 1.0);       // 최대 스로틀
    throttle_ema_alpha_   = this->declare_parameter<double>("throttle_ema_alpha", 0.5); // 스로틀 값에 대한 EMA 필터 계수

    // --- 타이머 설정 (Timing) ---
    control_rate_hz_      = this->declare_parameter<double>("control_rate_hz", 30.0);   // 제어 루프 실행 주기 [Hz]

    // --- 시각화 (Visualization, 모든 좌표는 CSV/경로 프레임 기준) ---
    viz_enable_           = this->declare_parameter<bool>("visualize", true);           // 시각화 활성화 여부
    viz_ns_               = this->declare_parameter<std::string>("viz_ns", "pure_pursuit"); // 시각화 마커 네임스페이스

    // ------------------------------
    // 구독 & 발행 설정 (Subscriptions & Publications)
    // ------------------------------
    sub_marker_ = this->create_subscription<visualization_msgs::msg::Marker>(
      input_marker_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PurePursuitNode::onMarker, this, std::placeholders::_1));

    sub_mission_state_ = this->create_subscription<std_msgs::msg::String>(
      "/mission_state", 10, std::bind(&PurePursuitNode::onMissionState, this, std::placeholders::_1));

    sub_throttle_cmd_ = this->create_subscription<std_msgs::msg::Float32>(
      "/throttle_cmd", 10, std::bind(&PurePursuitNode::onThrottleCmd, this, std::placeholders::_1));

    pub_steer_ = this->create_publisher<std_msgs::msg::Float32>(steer_cmd_topic_, 10);
    pub_throttle_ = this->create_publisher<std_msgs::msg::Float32>(throttle_from_planning_topic_, 10);

    if (viz_enable_) {
      pub_viz_target_  = this->create_publisher<visualization_msgs::msg::Marker>("/pp/lookahead_point", 1);      // 전방 주시점
      pub_viz_ldarc_   = this->create_publisher<visualization_msgs::msg::Marker>("/pp/lookahead_circle", 1);     // 현재 Ld 원
      pub_viz_ldmin_   = this->create_publisher<visualization_msgs::msg::Marker>("/pp/lookahead_min_circle", 1); // 최소 Ld 원
      pub_viz_ldmax_   = this->create_publisher<visualization_msgs::msg::Marker>("/pp/lookahead_max_circle", 1); // 최대 Ld 원
      pub_viz_steer_   = this->create_publisher<visualization_msgs::msg::Marker>("/pp/steer_arrow", 1);          // 계산된 조향각 화살표
      pub_viz_steer_tx_= this->create_publisher<visualization_msgs::msg::Marker>("/pp/steer_text", 1);           // 계산된 조향각 텍스트
      pub_viz_ld_text_  = this->create_publisher<visualization_msgs::msg::Marker>("/pp/ld_text", 1);              // Ld 값 텍스트
      pub_viz_throttle_text_ = this->create_publisher<visualization_msgs::msg::Marker>("/pp/throttle_text", 1);    // 스로틀 값 텍스트
      pub_viz_mission_text_ = this->create_publisher<visualization_msgs::msg::Marker>("/pp/mission_text", 1);      // 미션 상태 텍스트
      pub_viz_vehicle_boundary_ = this->create_publisher<visualization_msgs::msg::Marker>("/henes_broon_T870", 1); // 차량 경계
      pub_viz_circumcircle_ = this->create_publisher<visualization_msgs::msg::Marker>("/pure_pursuit/circumcircle", 1);
    }

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_),
      std::bind(&PurePursuitNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(),
      "PurePursuit (exp Ld) started: input=%s, base=%s, Ld_range=[%.2f~%.2f], beta=%.2f, win=%d",
      input_marker_topic_.c_str(), base_frame_.c_str(), min_ld_, max_ld_, beta_, curvature_window_);
    RCLCPP_INFO(this->get_logger(), "Throttle control enabled: topic=%s, range=[%.2f, %.2f], alpha=%.2f",
      throttle_from_planning_topic_.c_str(), min_throttle_, max_throttle_, throttle_ema_alpha_);
  }

private:
  // ------------------------------
  // 마커 콜백: 원본 경로(CSV 프레임) 저장
  // ------------------------------
  void onMarker(const visualization_msgs::msg::Marker::SharedPtr msg)
  {
    if (msg->type != visualization_msgs::msg::Marker::LINE_STRIP || msg->points.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Input marker is not a LINE_STRIP or empty. (입력 마커가 LINE_STRIP이 아니거나 비어있습니다)");
      path_points_.clear();
      path_frame_id_.clear();
      return;
    }
    path_points_    = msg->points;
    path_frame_id_  = msg->header.frame_id.empty() ? fallback_path_frame_ : msg->header.frame_id;
    last_path_stamp_= msg->header.stamp;
  }

  void onMissionState(const std_msgs::msg::String::SharedPtr msg)
  {
    current_mission_state_ = msg->data;
  }

  void onThrottleCmd(const std_msgs::msg::Float32::SharedPtr msg)
  {
    current_throttle_cmd_ = msg->data;
  }

  // ------------------------------
  // 타이머: 단일 TF (base->path), 모든 계산은 경로(CSV) 프레임에서 수행
  // ------------------------------
  void onTimer()
  {
    if (path_points_.size() < 2 || path_frame_id_.empty()) {
      publishSteer(0.0);
      publishThrottle(min_throttle_); // 경로 없으면 최소 스로틀 (사실상 정지)
      throttle_filt_.reset();         // NEW: 필터 상태 리셋
      if (viz_enable_) publishSteerMarkersCsv(0.0, geometry_msgs::msg::Point());
      return;
    }

    // 1) TF 변환: base_frame -> path_frame (차량->경로)
    geometry_msgs::msg::TransformStamped base_to_path_tf;
    try {
      base_to_path_tf = tf_buffer_.lookupTransform(
          path_frame_id_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF lookup failed (%s <- %s): %s",
                           path_frame_id_.c_str(), base_frame_.c_str(), ex.what());
      publishSteer(0.0);
      publishThrottle(min_throttle_); // TF 에러 시 최소 스로틀
      throttle_filt_.reset();         // NEW: 필터 상태 리셋
      if (viz_enable_) publishSteerMarkersCsv(0.0, geometry_msgs::msg::Point());
      return;
    }

    // 경로 프레임 기준 차량의 위치
    geometry_msgs::msg::PointStamped base_origin, robot_in_path;
    base_origin.header.frame_id = base_frame_;
    base_origin.point.x = 0.0; base_origin.point.y = 0.0; base_origin.point.z = 0.0;
    tf2::doTransform(base_origin, robot_in_path, base_to_path_tf);
    const geometry_msgs::msg::Point rob_p = robot_in_path.point;

    // 2) 가장 가까운 경로 점 탐색 (경로 프레임 기준)
    int nearest_idx = -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(path_points_.size()); ++i) {
      const auto& p = path_points_[i];
      const double dx = p.x - rob_p.x;
      const double dy = p.y - rob_p.y;
      const double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) { best_d2 = d2; nearest_idx = i; }
    }
    if (nearest_idx < 0) {
      publishSteer(0.0);
      publishThrottle(min_throttle_); // 최근접점 못찾으면 최소 스로틀
      throttle_filt_.reset();         // NEW: 필터 상태 리셋
      if (viz_enable_) {
        publishLookaheadCirclesCsv(fixed_ld_, rob_p);
        publishSteerMarkersCsv(0.0, rob_p);
      }
      return;
    }

    // 3) 최근접점 주변의 곡률 추정 (경로 프레임 기준)
    const double kappa_abs_avg = localCurvatureAbsAvg(nearest_idx);
    // 안정성을 위한 EMA 필터 적용
    kappa_filt_ = curvature_ema_alpha_ * kappa_abs_avg + (1.0 - curvature_ema_alpha_) * kappa_filt_;

    // 4) 지수 함수 형태의 곡률 적응형 전방 주시 거리(Ld) 계산
    double Ld;
    if (current_mission_state_ == "REVERSE_T" || current_mission_state_ == "REVERSE_PARALLEL") {
      Ld = back_ld_;
    } else {
      const double ld_span = std::max(0.0, max_ld_ - min_ld_);
      double ld_eff = min_ld_ + ld_span * std::exp(-beta_ * kappa_filt_);   // ★ 핵심: Ld = Ld_min + (Ld_max-Ld_min)*exp(-beta*kappa)
      ld_eff = clamp(ld_eff, min_ld_, max_ld_);

      // Ld 값의 급격한 변화를 막기 위한 EMA 필터 (선택 사항)
      if (!ld_filt_.has_value()) ld_filt_ = ld_eff;
      ld_filt_ = ld_ema_alpha_ * ld_eff + (1.0 - ld_ema_alpha_) * ld_filt_.value();
      Ld = clamp(ld_filt_.value(), min_ld_, max_ld_);
    }

    // 5) 최근접점으로부터 Ld만큼 떨어진 목표점 보간 (경로 프레임 기준)
    geometry_msgs::msg::Point target_in_path = interpolateLookaheadFrom(nearest_idx, Ld);

    // 6) 조향각 계산: v_path(경로 기준 목표 벡터)를 v_base(차량 기준 목표 벡터)로 변환
    const double vx_p = target_in_path.x - rob_p.x;
    const double vy_p = target_in_path.y - rob_p.y;

    const auto& q_bp = base_to_path_tf.transform.rotation; // base->path 회전
    tf2::Quaternion q(q_bp.x, q_bp.y, q_bp.z, q_bp.w);
    tf2::Matrix3x3 R_bp(q), R_pb = R_bp.transpose(); // R_pb는 path->base 회전 행렬

    // v_base = R_pb * v_path (z=0으로 가정)
    const double vy_b = R_pb[1][0]*vx_p + R_pb[1][1]*vy_p; // 차량 기준 y축 방향 속도 성분

    const double L = wheelbase_;
    const double ld_sq = std::max(Ld*Ld, 1e-6);
    const double kappa_pp = 2.0 * vy_b / ld_sq; // Pure pursuit 곡률 공식
    double steer_rad = std::atan(L * kappa_pp); // 조향각(라디안) = atan(Wheelbase * kappa)

    double steer_out = steer_is_degree_ ? (steer_rad * 180.0 / M_PI) : steer_rad;
    const double limit = steer_is_degree_ ? steer_limit_deg_ : (steer_limit_deg_ * M_PI / 180.0);
    if (steer_out >  limit) steer_out =  limit;
    if (steer_out < -limit) steer_out = -limit;

    publishSteer(-steer_out); // <<< 조향각에 -1 곱하여 발행

    // 7) 스로틀 계산 
    const double abs_steer_normalized = std::abs(steer_out) / limit; // 정규화된 조향각 크기 (0~1)
    // 조향각이 클수록 스로틀을 줄임
    double throttle_value = max_throttle_ - abs_steer_normalized * (max_throttle_ - min_throttle_);
    throttle_value = clamp(throttle_value, min_throttle_, max_throttle_);
    
    // 스로틀 값 EMA 필터 적용
    if (!throttle_filt_.has_value()) {
      throttle_filt_ = throttle_value;
    } else {
      throttle_filt_ = throttle_ema_alpha_ * throttle_value + (1.0 - throttle_ema_alpha_) * throttle_filt_.value();
    }
    publishThrottle(throttle_filt_.value());

    // 8) 시각화 (모든 좌표는 CSV 프레임 기준)
    if (viz_enable_) {
      publishLookaheadPointCsv(target_in_path);
      publishLookaheadCirclesCsv(Ld, rob_p);
      publishSteerMarkersCsv(steer_out, rob_p); // 시각화도 반전된 값 사용
      publishLdTextCsv(Ld, rob_p); // Ld 값 텍스트 시각화 추가
      publishThrottleTextCsv(rob_p); // 스로틀 값 텍스트 시각화 추가
      publishMissionTextCsv(rob_p); // 미션 상태 텍스트 시각화 추가
      publishVehicleBoundaryCsv(rob_p, q);

      // NEW: 곡률 시각화를 위한 외접원 발행
      if (path_points_.size() > 2 && nearest_idx > 0 && nearest_idx < (path_points_.size() - 1)) {
        const auto& p1 = path_points_[nearest_idx - 1];
        const auto& p2 = path_points_[nearest_idx];
        const auto& p3 = path_points_[nearest_idx + 1];
        auto circle_opt = calculateCircumcircle(p1, p2, p3);
        if (circle_opt) {
          publishCircumcircleCsv(circle_opt->first, circle_opt->second);
        }
      }
    }
  }

  // --- 곡률 계산 관련 함수 ---
  // 세 점의 외접원 계산
  std::optional<std::pair<geometry_msgs::msg::Point, double>>
  calculateCircumcircle(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2, const geometry_msgs::msg::Point& p3) const
  {
      double D = 2.0 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
      if (std::abs(D) < 1e-9) { // 점들이 거의 일직선 상에 있을 경우
          return std::nullopt;
      }

      geometry_msgs::msg::Point center;
      center.x = ((p1.x * p1.x + p1.y * p1.y) * (p2.y - p3.y) + (p2.x * p2.x + p2.y * p2.y) * (p3.y - p1.y) + (p3.x * p3.x + p3.y * p3.y) * (p1.y - p2.y)) / D;
      center.y = ((p1.x * p1.x + p1.y * p1.y) * (p3.x - p2.x) + (p2.x * p2.x + p2.y * p2.y) * (p1.x - p3.x) + (p3.x * p3.x + p3.y * p3.y) * (p2.x - p1.x)) / D;
      center.z = 0.0;

      double radius = std::hypot(p1.x - center.x, p1.y - center.y);

      return std::make_pair(center, radius);
  }

  // 지정된 인덱스 주변의 지역 곡률 절대값 평균 계산
  double localCurvatureAbsAvg(int idx) const
  {
    const int N = static_cast<int>(path_points_.size());
    if (N < 3) return 0.0;

    const int half = std::max(1, curvature_window_ / 2);
    int i0 = std::max(1, idx - half);
    int i1 = std::min(N - 2, idx + half);
    if (i1 <= i0) { // 윈도우가 유효하지 않을 경우 최소한의 점으로 재설정
      i0 = std::max(1, std::min(idx, N-2) - 1);
      i1 = std::min(N - 2, i0 + 1);
    }

    double sum_abs_kappa = 0.0;
    int cnt = 0;
    for (int i = i0; i <= i1; ++i) { // i를 중심으로 세 점(i-1, i, i+1)을 사용
      const auto& p1 = path_points_[i-1];
      const auto& p2 = path_points_[i];
      const auto& p3 = path_points_[i+1];

      // Menger 곡률 공식: kappa = 4 * Area(p1,p2,p3) / (|p1-p2|*|p2-p3|*|p3-p1|)
      const double a = std::hypot(p2.x - p1.x, p2.y - p1.y);
      const double b = std::hypot(p3.x - p2.x, p3.y - p2.y);
      const double c = std::hypot(p3.x - p1.x, p3.y - p1.y);
      if (a < 1e-6 || b < 1e-6 || c < 1e-6) continue;

      // 삼각형 넓이의 2배 = |(x2-x1)(y3-y1) - (y2-y1)(x3-x1)|
      const double cross = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
      const double area2 = std::abs(cross);
      const double kappa = 2.0 * area2 / (a * b * c); // Menger 곡률 공식은 4*Area지만, area2가 2*Area이므로 분자가 2*area2
      sum_abs_kappa += kappa;
      ++cnt;
    }
    if (cnt == 0) return 0.0;
    return sum_abs_kappa / static_cast<double>(cnt);
  }

  // 최근접점으로부터 Ld 거리만큼 떨어진 경로 상의 점을 보간하여 찾음
  geometry_msgs::msg::Point interpolateLookaheadFrom(int nearest_idx, double Ld) const
  {
    geometry_msgs::msg::Point target;
    target.x = path_points_.back().x;
    target.y = path_points_.back().y;
    target.z = 0.0;

    double acc = 0.0; // 누적 거리
    geometry_msgs::msg::Point prev = path_points_[nearest_idx];
    for (int i = nearest_idx + 1; i < static_cast<int>(path_points_.size()); ++i) {
      const geometry_msgs::msg::Point cur = path_points_[i];
      const double seg = std::hypot(cur.x - prev.x, cur.y - prev.y); // 현재 점과 이전 점 사이의 거리
      if (acc + seg >= Ld) {
        const double remain = Ld - acc; // 찾아야 할 남은 거리
        const double t = (seg > 1e-6) ? (remain / seg) : 0.0; // 현재 세그먼트에서 보간 비율
        target.x = prev.x + t * (cur.x - prev.x);
        target.y = prev.y + t * (cur.y - prev.y);
        target.z = 0.0;
        return target;
      }
      acc += seg;
      prev = cur;
    }
    return target; // 경로가 Ld보다 짧으면 마지막 점을 목표점으로 반환
  }

  // --- 유틸리티 함수 ---
  static double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
  }

  void publishSteer(double value)
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(value);
    pub_steer_->publish(msg);
  }

  void publishThrottle(double value)
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(value);
    pub_throttle_->publish(msg);
  }


  // --- 시각화 함수 (모든 좌표는 CSV/경로 프레임 기준) ---
  void publishCircumcircleCsv(const geometry_msgs::msg::Point& center, double radius)
  {
      auto circle_marker = makeCircleCsv(path_frame_id_, "circumcircle", 0, center, radius);
      circle_marker.scale.x = 0.03; // 선 두께
      circle_marker.color.a = 0.4;
      circle_marker.color.r = 1.0;
      circle_marker.color.g = 0.5;
      circle_marker.color.b = 0.0; // 주황색
      pub_viz_circumcircle_->publish(circle_marker);
  }

  void publishVehicleBoundaryCsv(const geometry_msgs::msg::Point& robot_center_csv, const tf2::Quaternion& robot_orientation_csv)
  {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = path_frame_id_;
    marker.header.stamp = this->now();
    marker.ns = "vehicle_boundary";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0; // 점들을 직접 변환하므로 포즈는 단위행렬

    marker.scale.x = 0.05; // 선 두께
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0; // 흰색

    // 차량 로컬 프레임(f9r) 기준 모서리 정의 (x: 전방, y: 좌측)
    const double x_front = 1.034;
    const double x_rear = -0.261;
    const double y_left = 0.3875;
    const double y_right = -0.3875;

    std::vector<tf2::Vector3> local_corners;
    local_corners.push_back(tf2::Vector3(x_front, y_right, 0.0)); // 앞-오른쪽
    local_corners.push_back(tf2::Vector3(x_front, y_left, 0.0));  // 앞-왼쪽
    local_corners.push_back(tf2::Vector3(x_rear, y_left, 0.0));   // 뒤-왼쪽
    local_corners.push_back(tf2::Vector3(x_rear, y_right, 0.0));  // 뒤-오른쪽
    local_corners.push_back(tf2::Vector3(x_front, y_right, 0.0)); // 폐곡선 만들기

    tf2::Matrix3x3 rot_matrix(robot_orientation_csv);
    tf2::Vector3 center_vec(robot_center_csv.x, robot_center_csv.y, robot_center_csv.z);

    for (const auto& local_corner : local_corners) {
      tf2::Vector3 transformed_corner = rot_matrix * local_corner + center_vec;
      geometry_msgs::msg::Point p;
      p.x = transformed_corner.x();
      p.y = transformed_corner.y();
      p.z = transformed_corner.z();
      marker.points.push_back(p);
    }

    pub_viz_vehicle_boundary_->publish(marker);
  }

  void publishLookaheadPointCsv(const geometry_msgs::msg::Point& target_in_path)
  {
    visualization_msgs::msg::Marker tp;
    tp.header.frame_id = path_frame_id_;
    tp.header.stamp = this->now();
    tp.ns = viz_ns_;
    tp.id = 1;
    tp.type = visualization_msgs::msg::Marker::SPHERE;
    tp.action = visualization_msgs::msg::Marker::ADD;
    tp.scale.x = tp.scale.y = tp.scale.z = 0.4;
    tp.color.a = 1.0; tp.color.r = 0.0; tp.color.g = 0.0; tp.color.b = 1.0; // 파란색
    tp.pose.orientation.w = 1.0;
    tp.pose.position = target_in_path;
    pub_viz_target_->publish(tp);
  }

  static visualization_msgs::msg::Marker makeCircleCsv(const std::string& frame,
                                                       const std::string& ns, int id,
                                                       const geometry_msgs::msg::Point& center,
                                                       double radius)
  {
    visualization_msgs::msg::Marker circle;
    circle.header.frame_id = frame;
    circle.header.stamp = rclcpp::Clock().now();
    circle.ns = ns;
    circle.id = id;
    circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
    circle.action = visualization_msgs::msg::Marker::ADD;
    circle.pose.orientation.w = 1.0;
    circle.pose.position = center;

    const int N = 64; // 원을 그릴 점의 개수
    circle.points.resize(N+1);
    for (int i=0;i<=N;++i){
      const double th = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(N);
      geometry_msgs::msg::Point p;
      p.x = radius * std::cos(th);
      p.y = radius * std::sin(th);
      p.z = 0.0;
      circle.points[i] = p;
    }
    return circle;
  }

  void publishLookaheadCirclesCsv(double Ld, const geometry_msgs::msg::Point& robot_center_csv)
  {
    auto ld = makeCircleCsv(path_frame_id_, viz_ns_, 2, robot_center_csv, Ld);
    ld.scale.x = 0.1; ld.color.a = 1.0; ld.color.r = 0.0; ld.color.g = 0.0; ld.color.b = 1.0; // 파란색
    pub_viz_ldarc_->publish(ld);

    auto ldmin = makeCircleCsv(path_frame_id_, viz_ns_, 3, robot_center_csv, min_ld_);
    ldmin.scale.x = 0.03; ldmin.color.a = 0.1; ldmin.color.r = 1.0; ldmin.color.g = 1.0; ldmin.color.b = 1.0; // 반투명 흰색
    pub_viz_ldmin_->publish(ldmin);

    auto ldmax = makeCircleCsv(path_frame_id_, viz_ns_, 4, robot_center_csv, max_ld_);
    ldmax.scale.x = 0.03; ldmax.color.a = 0.1; ldmax.color.r = 1.0; ldmax.color.g = 1.0; ldmax.color.b = 1.0; // 반투명 흰색
    pub_viz_ldmax_->publish(ldmax);
  }

  void publishSteerMarkersCsv(double steer_value, const geometry_msgs::msg::Point& robot_center_csv)
  {
    // 조향각을 차량 로컬 프레임의 벡터로 변환
    double angle_rad = steer_is_degree_ ? steer_value * M_PI / 180.0 : steer_value;
    const double bx = std::cos(angle_rad);
    const double by = std::sin(angle_rad);

    // TF를 이용해 경로 프레임으로 변환
    geometry_msgs::msg::TransformStamped base_to_path_tf;
    try {
      base_to_path_tf = tf_buffer_.lookupTransform(
          path_frame_id_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.02));
    } catch (...) {
      // TF 실패 시 기본 방향(x축)으로 화살표 표시
      geometry_msgs::msg::Point p1;
      p1.x = robot_center_csv.x + 1.0;
      p1.y = robot_center_csv.y;
      p1.z = 0.0;
      publishSimpleArrowCsv(robot_center_csv, p1);
      publishSteerTextCsv(steer_value, robot_center_csv);
      return;
    }

    const auto& q_bp = base_to_path_tf.transform.rotation;
    tf2::Quaternion q(q_bp.x, q_bp.y, q_bp.z, q_bp.w);
    tf2::Matrix3x3 R_bp(q);

    // 로컬 벡터를 경로 프레임 벡터로 회전
    const double vx_p = R_bp[0][0]*bx + R_bp[0][1]*by;
    const double vy_p = R_bp[1][0]*bx + R_bp[1][1]*by;

    const double len = 1.0 + 0.02 * std::abs(steer_value); // 조향각이 클수록 화살표 길게

    geometry_msgs::msg::Point p0 = robot_center_csv;
    geometry_msgs::msg::Point p1;
    p1.x = robot_center_csv.x + len * vx_p;
    p1.y = robot_center_csv.y + len * vy_p;
    p1.z = robot_center_csv.z;

    publishSimpleArrowCsv(p0, p1);
    publishSteerTextCsv(steer_value, robot_center_csv);
  }

  void publishSimpleArrowCsv(const geometry_msgs::msg::Point& p0, const geometry_msgs::msg::Point& p1)
  {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = path_frame_id_;
    arrow.header.stamp = this->now();
    arrow.ns = viz_ns_;
    arrow.id = 5;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose.orientation.w = 1.0;
    arrow.scale.x = 0.1;  // 화살대 두께
    arrow.scale.y = 0.4;  // 화살촉 밑면 너비
    arrow.scale.z = 0.5;  // 화살촉 높이
    arrow.color.a = 1.0; arrow.color.r = 1.0; arrow.color.g = 0.8; arrow.color.b = 0.1; // 노란색
    arrow.points = {p0, p1};
    pub_viz_steer_->publish(arrow);
  }

  void publishSteerTextCsv(double steer_value, const geometry_msgs::msg::Point& robot_center_csv)
  {
    visualization_msgs::msg::Marker txt;
    txt.header.frame_id = path_frame_id_;
    txt.header.stamp = this->now();
    txt.ns = viz_ns_;
    txt.id = 6;
    txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::msg::Marker::ADD;
    txt.scale.z = 1.8; // 텍스트 크기
    txt.color.a = 1.0; txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 1.0; // 흰색
    txt.pose.position = robot_center_csv;
    txt.pose.position.y += -1.8; // 텍스트 위치 조정

    char buf[64];
    if (steer_is_degree_) std::snprintf(buf, sizeof(buf), "/auto_steer_angle %.2f deg", steer_value);
    else                  std::snprintf(buf, sizeof(buf), "/auto_steer_angle %.3f rad", steer_value);
    txt.text = std::string(buf);
    pub_viz_steer_tx_->publish(txt);
  }

  void publishLdTextCsv(double Ld, const geometry_msgs::msg::Point& robot_center_csv)
  {
    visualization_msgs::msg::Marker txt;
    txt.header.frame_id = path_frame_id_;
    txt.header.stamp = this->now();
    txt.ns = viz_ns_;
    txt.id = 7; // ID for Ld text
    txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::msg::Marker::ADD;
    txt.scale.z = 1.8; // 텍스트 크기
    txt.color.a = 1.0;

    // 미션 상태에 따라 색상 변경
    if (current_mission_state_ == "REVERSE_T" || current_mission_state_ == "REVERSE_PARALLEL") {
      txt.color.r = 1.0; txt.color.g = 0.1; txt.color.b = 0.1; // 빨간색
    } else {
      txt.color.r = 0.2; txt.color.g = 1.0; txt.color.b = 0.2; // 연두색
    }

    txt.pose.position = robot_center_csv;
    txt.pose.position.y += -3.8; // 조향각 텍스트 아래에 표시
    txt.pose.position.z += 0.5;

    char buf[64];
    std::snprintf(buf, sizeof(buf), "Ld %.2f m", Ld);
    txt.text = std::string(buf);
    pub_viz_ld_text_->publish(txt);
  }

  void publishThrottleTextCsv(const geometry_msgs::msg::Point& robot_center_csv)
  {
    visualization_msgs::msg::Marker txt;
    txt.header.frame_id = path_frame_id_;
    txt.header.stamp = this->now();
    txt.ns = viz_ns_;
    txt.id = 8; // ID for throttle text
    txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::msg::Marker::ADD;
    txt.scale.z = 1.8; // 텍스트 크기
    txt.color.a = 1.0; txt.color.r = 0.2; txt.color.g = 0.8; txt.color.b = 1.0; // 하늘색
    txt.pose.position = robot_center_csv;
    txt.pose.position.y += -5.8; // Ld 텍스트 아래에 표시
    txt.pose.position.z += 0.5;

    char buf[64];
    std::snprintf(buf, sizeof(buf), "/throttle_cmd %.2f", current_throttle_cmd_);
    txt.text = std::string(buf);
    pub_viz_throttle_text_->publish(txt);
  }

  void publishMissionTextCsv(const geometry_msgs::msg::Point& robot_center_csv)
  {
    visualization_msgs::msg::Marker txt;
    txt.header.frame_id = path_frame_id_;
    txt.header.stamp = this->now();
    txt.ns = viz_ns_;
    txt.id = 9; // ID for mission state text
    txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::msg::Marker::ADD;
    txt.scale.z = 1.8; // 텍스트 크기
    txt.color.a = 1.0; txt.color.r = 1.0; txt.color.g = 0.2; txt.color.b = 1.0; // 자홍색
    txt.pose.position = robot_center_csv;
    txt.pose.position.y += -7.8; // 스로틀 텍스트 아래에 표시
    txt.pose.position.z += 0.5;

    char buf[64];
    std::snprintf(buf, sizeof(buf), "/mission_state %s", current_mission_state_.c_str());
    txt.text = std::string(buf);
    pub_viz_mission_text_->publish(txt);
  }

  // ------------------------------
  // 멤버 변수 (Members)
  // ------------------------------
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // 원본 경로 (CSV 프레임)
  std::vector<geometry_msgs::msg::Point> path_points_;
  std::string path_frame_id_;
  rclcpp::Time last_path_stamp_;

  // 파라미터
  std::string input_marker_topic_;
  std::string base_frame_;
  std::string fallback_path_frame_;
  double wheelbase_;

  // Ld & 곡률 (지수 형태)
  double fixed_ld_, min_ld_, max_ld_;
  double beta_;
  int    curvature_window_;
  double curvature_ema_alpha_;
  double ld_ema_alpha_;

  // 조향각 출력
  std::string steer_cmd_topic_;
  bool   steer_is_degree_;
  double steer_limit_deg_;

  // 스로틀 출력
  std::string throttle_from_planning_topic_;
  double min_throttle_;
  double max_throttle_;
  double throttle_ema_alpha_;

  double control_rate_hz_;
  bool   viz_enable_;
  std::string viz_ns_;

  // 입출력 인터페이스
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr sub_marker_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             pub_steer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             pub_throttle_; 
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    pub_viz_target_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    pub_viz_ldarc_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    pub_viz_ldmin_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    pub_viz_ldmax_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    pub_viz_steer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    pub_viz_steer_tx_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    pub_viz_ld_text_; // Ld 값 시각화용
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    pub_viz_throttle_text_; // 스로틀 값 시각화용
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    pub_viz_mission_text_; // 미션 상태 시각화용
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    pub_viz_vehicle_boundary_; // NEW
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    pub_viz_circumcircle_; // NEW
  rclcpp::TimerBase::SharedPtr                                     timer_;

  // 상태 변수 (필터)
  double kappa_filt_; // 곡률 필터 상태
  std::optional<double> ld_filt_; // Ld 필터 상태
  std::optional<double> throttle_filt_; // 스로틀 필터 상태

  // 미션 및 스로틀 상태
  double back_ld_;
  std::string current_mission_state_ = "GPS_FWD";
  float current_throttle_cmd_ = 0.0;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mission_state_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_throttle_cmd_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}
