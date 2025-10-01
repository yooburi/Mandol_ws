# ==============================================================================
#
#         ## 미션 감독 노드 상태 기계 ##
#
#
#     [세이프티 레이어] (선점형)
#            |
#     (안전한가?) --(아니오)--> [세이프티 홀드] --(복귀)--> [이전 미션]
#            |
#          (예)
#            |
#     [미션 레이어]
#            |
#     +----------------------------------------------------------------------+
#     |                                                                      |
#     |  +-----------+ --(T 완료)--> +---------+ --(T 트리거)--> +-----------+ |
#     |  |   T자 후진  |             | GPS 주행  |               |   T자 후진  | |
#     |  +-----------+ <--(/T 완료) +---------+  (/T 트리거)--> +-----------+ |
#     |                                  | ^                                 |
#     |                                  | |                                 |
#     |  +-----------+ <---(/P 완료) | (/P 트리거)--> +-----------+ |
#     |  |  평행 주차  |             |                 |  평행 주차  | |
#     |  +-----------+ --(P 완료)--> +-----------------+ --(P 트리거)--> |
#     |                                                                      |
#     +----------------------------------------------------------------------+
#
#
# ==============================================================================
# ## 상태 표 ##
# +--------------------+--------------------------+---------------------------------+
# |      미션 상태     |      활성 알고리즘       |           상태 전이 조건        |
# +--------------------+--------------------------+---------------------------------+
# | INIT (초기화)      | -                        | 시스템 시작                     |
# | GPS_FWD (GPS 주행) | FWD_CONTROLLER           | 기본 상태, 또는 서브 미션 완료   |
# | REVERSE_T (T자 후진)| REVERSE_T_CONTROLLER     | /reverse_T/trigger 가 true      |
# | REVERSE_PARALLEL   | REVERSE_PARALLEL_CONTROLLER| /reverse_parallel/trigger 가 true|
# +--------------------+--------------------------+---------------------------------+
#
# ## 세이프티 표 ##
# +----------------+------------+------------------------------------------------+
# |    안전 상태   |   우선순위 |                  트리거 조건                   |
# +----------------+------------+------------------------------------------------+
# | SAFE_OK (안전) |    N/A     | 활성화된 정지 신호 없음                        |
# | STOP/SLOPE     |      1     | /slope_stop 이 true                            |
# | STOP/OBSTACLE  |      2     | /obstacle_existance 가 true                    |
# | STOP/TRAFFIC   |      3     | /traffic_stop && /intersection 이 true          |
# +----------------+------------+------------------------------------------------+
#
# ## 테스트 시나리오 ##
# A. 세이프티 선점: /traffic_stop=true && /intersection=true → throttle 0, SAFETY_HOLD.
#    두 신호 중 하나라도 false → 히스테리시스 후 이전 상태로 복귀.
# B. 경사로 정지: /slope_stop=true → 5초 정지 유지 후 복귀.
# C. 장애물 정지: /obstacle_existance=true → 5초 정지 후 GPS_FWD로 복귀.
# D. 미션 전환: /reverse_T/trigger → REVERSE_T 상태.
#    /reverse_T/done → GPS_FWD 상태.
# E. 토픽 일관성: mission_state, active_algorithm,
#    safety_status, safety_active 가 모두 올바른지 확인.
# ==============================================================================

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String, Bool, Float32
from enum import Enum, auto


class MissionState(Enum):
    INIT = auto()
    GPS_FWD = auto()
    REVERSE_T = auto()
    REVERSE_PARALLEL = auto()


class SafetyStatus(Enum):
    SAFE_OK = auto()
    STOP_SLOPE = auto()
    STOP_OBSTACLE = auto()
    STOP_TRAFFIC = auto()


class ActiveAlgorithm(Enum):
    FWD_CONTROLLER = auto()
    REVERSE_T_CONTROLLER = auto()
    REVERSE_PARALLEL_CONTROLLER = auto()
    SAFETY_HOLD = auto()


class MissionSupervisorNode(Node):

    def __init__(self):
        super().__init__('mission_supervisor_node')

        # 파라미터 선언
        self.declare_parameter('loop_rate_hz', 50.0)
        self.declare_parameter('hysteresis_sec', 0.5) # 신호 비활성화 후 히스테리시스 시간
        self.declare_parameter('slope_hold_sec', 5.0) # 경사로 정지 유지 시간
        self.declare_parameter('obstacle_hold_sec', 5.0) # 장애물 정지 유지 시간
        self.declare_parameter('throttle_topic', '/throttle_cmd') # 스로틀 명령 토픽명
        self.declare_parameter('direction_switch_hold_sec', 2.0) # 전/후진 전환 시 정지 유지 시간

        # 파라미터 가져오기
        self.loop_rate = self.get_parameter('loop_rate_hz').value
        self.hysteresis_sec = self.get_parameter('hysteresis_sec').value
        self.slope_hold_sec = self.get_parameter('slope_hold_sec').value
        self.obstacle_hold_sec = self.get_parameter('obstacle_hold_sec').value
        self.throttle_topic = self.get_parameter('throttle_topic').value
        self.direction_switch_hold_sec = self.get_parameter('direction_switch_hold_sec').value

        # 내부 상태 변수
        self.mission_state = MissionState.INIT # 초기 상태
        self.prev_mission_state = MissionState.GPS_FWD # 이전 미션 상태 저장용 & GPS_FWD로 초기화
        self.safety_status = SafetyStatus.SAFE_OK # 초기 안전 상태
        self.is_safety_active = False # 세이프티 선점 활성화 여부 -> true면 스로틀 0

        # 안전 신호 상태(SAFE_OK가 아닌 신호가 들어오면 세이프티 선점)
        self.slope_stop_active = False # 경사로 정지 신호 상태 -> ture면 경사로 정지
        self.obstacle_stop_active = False # 장애물 정지 신호 상태 -> ture면 장애물 정지
        self.traffic_stop_active = False # 신호등 정지 신호 상태 -> true면 신호등 정지 후보
        self.intersection_active = False # 교차로 영역 상태 -> true면 교차로 내부

        # 경사로 정지 세션 관리 (A안: 5초 유지 후 출발 허용)
        self.prev_slope_stop_active = False
        self.slope_session_start = None  # rclpy.time.Time
        self.slope_session_released = False  # 동일 세션에서 5초 경과 후 재선점 방지

        # 안전 복귀 타이머 
        self.safety_triggered_time = None # 세이프티 선점이 처음 활성화된 시간
        self.stop_signal_false_time = None # 모든 정지 신호가 비활성화된 시간

        # 전/후진 전환 시 정지 유지 제어용 상태
        self.prev_motion_dir = 0  # -1: 후진, 0: 미정/초기, +1: 전진
        self.direction_switch_start_sec = None  # 전환 시작 시간 (초)

        # 트리거/완료 신호의 상승 에지 검출용 변수 (이전 상태 저장)
        self.prev_reverse_t_trigger = False # 이전 /reverse_T/trigger 상태
        self.prev_reverse_t_done = False # 이전 /reverse_T/done 상태
        self.prev_reverse_parallel_trigger = False # 이전 /reverse_parallel/trigger 상태
        self.prev_reverse_parallel_done = False # 이전 /reverse_parallel/done 상태

        # 미션 트리거/완료의 원샷 래치(한 번만 유효)
        self.latched_reverse_t_trigger = False
        self.latched_reverse_t_done = False
        self.latched_reverse_parallel_trigger = False
        self.latched_reverse_parallel_done = False

        # 수신된 스로틀 값 저장용 변수
        self.throttle_from_planning = 0.0

        # 퍼블리셔
        self.throttle_pub = self.create_publisher(Float32, self.throttle_topic, 10)
        self.mission_state_pub = self.create_publisher(String, '/mission_state', 10) # 미션 상태 토픽
        self.active_algorithm_pub = self.create_publisher(String, '/active_algorithm', 10) # 활성 알고리즘 토픽
        self.safety_status_pub = self.create_publisher(String, '/safety_status', 10) # 안전 상태 토픽
        self.safety_active_pub = self.create_publisher(Bool, '/safety_active', 10) # 세이프티 활성화 여부 토픽

        # 서브스크라이버(중요)
        self.create_subscription(Bool, '/slope_stop', self.slope_stop_cb, 10) # 경사로 정지 신호.. Boolean
        self.create_subscription(Bool, '/traffic_stop', self.traffic_stop_cb, 10) # 신호등 정지 신호.. Boolean
        self.create_subscription(Bool, '/intersection', self.intersection_cb, 10) # 교차로 영역.. Boolean
        self.create_subscription(Bool, '/obstacle_existance', self.obstacle_stop_cb, 10) # 장애물 정지 신호.. Boolean
        self.create_subscription(Bool, '/reverse_T/trigger', self.reverse_t_trigger_cb, 10) # T자 후진 트리거.. Boolean
        self.create_subscription(Bool, '/reverse_T/done', self.reverse_t_done_cb, 10) # T자 후진 완료.. Boolean
        self.create_subscription(Bool, '/reverse_parallel/trigger', self.reverse_parallel_trigger_cb, 10) # 평행 주차 트리거.. Boolean
        self.create_subscription(Bool, '/reverse_parallel/done', self.reverse_parallel_done_cb, 10) # 평행 주차 완료.. Boolean
        self.create_subscription(Float32, '/throttle_from_planning', self.throttle_planning_cb, 10) # 스로틀 계획 값.. Float32

        # 메인 타이머 루프
        self.timer = self.create_timer(1.0 / self.loop_rate, self.timer_callback)

        self.get_logger().info("미션 감독 노드 시작.")

    def throttle_planning_cb(self, msg):
        self.throttle_from_planning = msg.data

    # --- 서브스크라이버 콜백 ---
    # SAFE 신호 상태. / Subscribe 받아서 상태 트래킹
    def slope_stop_cb(self, msg):
        now = self.get_clock().now()
        self.slope_stop_active = msg.data
        # 상승 에지: 세션 시작 및 해제 플래그 초기화
        if self.slope_stop_active and not self.prev_slope_stop_active:
            self.slope_session_start = now
            self.slope_session_released = False
        # 하강 에지: 세션 리셋
        if (not self.slope_stop_active) and self.prev_slope_stop_active:
            self.slope_session_start = None
            self.slope_session_released = False
        self.prev_slope_stop_active = self.slope_stop_active

    def traffic_stop_cb(self, msg):
        self.traffic_stop_active = msg.data

    def obstacle_stop_cb(self, msg):
        self.obstacle_stop_active = msg.data

    def intersection_cb(self, msg):
        self.intersection_active = msg.data

    # 상승에지 검출 이유-> 명령이 지속적으로 true일 수 있기 때문. 한 번만 처리하기 위해.
    # 신호가 true이고 이전 상태가 false일 때만 처리. 
    def reverse_t_trigger_cb(self, msg):
        if msg.data and not self.prev_reverse_t_trigger and not self.latched_reverse_t_trigger: # 상승 에지 + 원샷 미사용
            changed = self.handle_mission_transition(MissionState.REVERSE_T)
            if changed:
                self.latched_reverse_t_trigger = True
        self.prev_reverse_t_trigger = msg.data

    def reverse_t_done_cb(self, msg):
        if msg.data and not self.prev_reverse_t_done and not self.latched_reverse_t_done:
            changed = self.handle_mission_transition(MissionState.GPS_FWD)
            if changed:
                self.latched_reverse_t_done = True
        self.prev_reverse_t_done = msg.data

    def reverse_parallel_trigger_cb(self, msg):
        if msg.data and not self.prev_reverse_parallel_trigger and not self.latched_reverse_parallel_trigger:
            changed = self.handle_mission_transition(MissionState.REVERSE_PARALLEL)
            if changed:
                self.latched_reverse_parallel_trigger = True
        self.prev_reverse_parallel_trigger = msg.data

    def reverse_parallel_done_cb(self, msg):
        if msg.data and not self.prev_reverse_parallel_done and not self.latched_reverse_parallel_done:
            changed = self.handle_mission_transition(MissionState.GPS_FWD)
            if changed:
                self.latched_reverse_parallel_done = True
        self.prev_reverse_parallel_done = msg.data

    # 핵심 미션 상태 전이 처리
    # 안전 선점이 활성화된 경우 미션 상태 변경 불가 -> 미션 상태를 안전하게 변경하는 역할.
    def handle_mission_transition(self, new_state):
        # 세이프티 선점이 비활성화된 경우에만 미션 상태 변경 허용
        if not self.is_safety_active:
            if self.mission_state != new_state:
                self.mission_state = new_state
                self.get_logger().info(f"미션 상태 변경: {new_state.name}")
                return True
            else:
                # 동일 상태로의 전이는 의미가 없으므로 미변경 처리
                return False
        # 세이프티 활성 시 전이 무시
        return False

    # --- 메인 루프 ---
    def timer_callback(self):
        self.process_safety_layer()
        self.process_mission_layer()
        self.publish_status()

    # --- 세이프티 레이어 안전 ---
    def process_safety_layer(self):
        now = self.get_clock().now()
        
        # 현재 안전 상태 평가
        current_safety_status = self.evaluate_safety_conditions(now)
        
        # 정지 신호 활성화 여부 판단 (안전 상태가 SAFE_OK가 아니면 활성화)
        is_stop_signal_active = current_safety_status != SafetyStatus.SAFE_OK
        
        # 정지 신호 활성화 처리
        if is_stop_signal_active:
            if not self.is_safety_active:
                # 안전 상태가 방금 활성화됨
                self.is_safety_active = True
                self.prev_mission_state = self.mission_state  # 현재 미션 상태 저장
                self.safety_status = current_safety_status
                self.safety_triggered_time = now
                self.get_logger().warn(f"세이프티 선점 활성화: {self.safety_status.name}")
            
            # 더 높은 우선순위의 안전 상태가 들어오면 업데이트
            if self.safety_status != current_safety_status:
                 self.safety_status = current_safety_status
                 self.get_logger().warn(f"세이프티 상태 변경: {self.safety_status.name}")

            self.stop_signal_false_time = None # 히스테리시스 타이머 리셋

        else: # 정지 신호가 비활성화 상태
            if self.is_safety_active:
                # 복귀 조건 확인
                if self.stop_signal_false_time is None:
                    self.stop_signal_false_time = now

                time_since_false = (now - self.stop_signal_false_time).nanoseconds / 1e9
                time_since_triggered = (now - self.safety_triggered_time).nanoseconds / 1e9

                can_recover = False
                if self.safety_status == SafetyStatus.STOP_SLOPE:
                    # A안: 5초 유지 후, /slope_stop 이 계속 true여도 즉시 복귀 허용 (히스테리시스 미적용)
                    if time_since_triggered > self.slope_hold_sec:
                        self.mission_state = MissionState.GPS_FWD
                        self.get_logger().info(f"경사로 정지 5초 경과. 이전 미션으로 복귀 준비.")
                        can_recover = True
                elif self.safety_status == SafetyStatus.STOP_OBSTACLE:
                    if time_since_triggered > self.obstacle_hold_sec:
                        self.mission_state = MissionState.GPS_FWD
                        self.get_logger().info(f"장애물 정지 유지 시간 종료. GPS_FWD로 복귀합니다.")
                        if time_since_false > self.hysteresis_sec:
                            can_recover = True
                elif self.safety_status == SafetyStatus.STOP_TRAFFIC:
                    if time_since_false > self.hysteresis_sec:
                        can_recover = True

                if can_recover:
                    self.is_safety_active = False
                    self.safety_status = SafetyStatus.SAFE_OK
                    self.mission_state = self.prev_mission_state # 이전 미션 상태로 복귀
                    self.get_logger().info(f"세이프티 선점 해제. 이전 상태로 복귀: {self.mission_state.name}")
                    self.safety_triggered_time = None
                    self.stop_signal_false_time = None

    # 현재 안전 조건 평가
    def evaluate_safety_conditions(self, now: Time):
        # 우선순위: 경사로 > 장애물 > 신호등
        # 경사로: 세션 시작 후 5초 미만이면 STOP_SLOPE, 5초 경과하면 같은 세션에서는 해제 처리
        if self.slope_stop_active:
            if self.slope_session_start is None:
                # 노드 시작 직후 등 에지 미검출 대비
                self.slope_session_start = now
                self.slope_session_released = False
            elapsed = (now - self.slope_session_start).nanoseconds / 1e9
            if (not self.slope_session_released) and (elapsed < self.slope_hold_sec):
                return SafetyStatus.STOP_SLOPE
            else:
                # 동일 세션에서 5초 경과 → 재선점 방지
                self.slope_session_released = True
                # 경사 조건은 해제하고 다른 세이프티 신호 평가로 진행
        if self.obstacle_stop_active:
            return SafetyStatus.STOP_OBSTACLE
        # 신호등 정지는 교차로 내에서만 유효
        if self.traffic_stop_active and self.intersection_active:
            return SafetyStatus.STOP_TRAFFIC
        return SafetyStatus.SAFE_OK

    # --- 미션 레이어 ---
    def process_mission_layer(self):
        if self.mission_state == MissionState.INIT:
            self.mission_state = MissionState.GPS_FWD
            self.get_logger().info("초기 상태를 GPS_FWD로 설정합니다.")

        # 미션 로직은 트리거/완료 콜백에서 상태를 변경하여 처리됨
        # 이 함수는 주로 상태가 유효한지 확인.

    # --- 상태 퍼블리싱 ---
    def publish_status(self):
        # 스로틀 명령 퍼블리시
        now_sec = self.get_clock().now().nanoseconds / 1e9

        # 현재 미션 상태 기반 진행 방향 평가 (-1: 후진, +1: 전진)
        if self.mission_state == MissionState.GPS_FWD:
            current_dir = 1
        elif self.mission_state in (MissionState.REVERSE_T, MissionState.REVERSE_PARALLEL):
            current_dir = -1
        else:
            current_dir = 0

        # 전/후진 방향 전환 감지 시 정지 유지 타이머 시작 (세이프티 선점 중이 아닐 때만 의미 있음)
        if self.prev_motion_dir != 0 and current_dir != 0 and current_dir != self.prev_motion_dir:
            # 전/후진 전환 발생
            self.direction_switch_start_sec = now_sec
            self.get_logger().info(f"전/후진 전환 감지: {self.direction_switch_hold_sec:.1f}초 정지 후 전환합니다.")

        # 다음 반복을 위해 방향 저장
        self.prev_motion_dir = current_dir

        # 전환 정지 유지 상태 계산
        hold_for_switch = False
        if self.direction_switch_start_sec is not None:
            if (now_sec - self.direction_switch_start_sec) < self.direction_switch_hold_sec:
                hold_for_switch = True
            else:
                # 유지 시간 종료
                self.direction_switch_start_sec = None

        # 최종적으로 적용할 스로틀 값 계산
        throttle_msg = Float32()
        output_throttle = 0.0  # 기본값은 0

        # 전/후진 전환 유지 우선: 무조건 0으로 정지
        if hold_for_switch:
            output_throttle = 0.0
        # 세이프티 활성화 시: 경사로에서는 0.15로 유지, 그 외에는 0
        elif self.is_safety_active:
            if self.safety_status == SafetyStatus.STOP_SLOPE:
                output_throttle = 0.15
            else:
                output_throttle = 0.0
        else:
            if self.mission_state == MissionState.GPS_FWD: # GPS 주행 시 계획된 스로틀 사용
                output_throttle = self.throttle_from_planning
            elif self.mission_state == MissionState.REVERSE_T or self.mission_state == MissionState.REVERSE_PARALLEL: # 후진 미션 시 후진 스로틀 사용
                output_throttle = -0.2 # 후진 스로틀 값 고정 (필요시 조정 가능)
        
        throttle_msg.data = output_throttle
        self.throttle_pub.publish(throttle_msg)

        # 미션 상태 퍼블리시
        mission_state_msg = String()
        mission_state_msg.data = self.mission_state.name
        self.mission_state_pub.publish(mission_state_msg) # 미션 상태 퍼블리시

        # 활성 알고리즘 퍼블리시
        algo_msg = String()
        if self.is_safety_active:
            algo_msg.data = ActiveAlgorithm.SAFETY_HOLD.name
        else:
            if self.mission_state == MissionState.GPS_FWD:
                algo_msg.data = ActiveAlgorithm.FWD_CONTROLLER.name
            elif self.mission_state == MissionState.REVERSE_T:
                algo_msg.data = ActiveAlgorithm.REVERSE_T_CONTROLLER.name
            elif self.mission_state == MissionState.REVERSE_PARALLEL:
                algo_msg.data = ActiveAlgorithm.REVERSE_PARALLEL_CONTROLLER.name
            else:
                algo_msg.data = "UNKNOWN"
        self.active_algorithm_pub.publish(algo_msg)

        # 안전 상태 퍼블리시
        safety_status_msg = String()
        safety_status_msg.data = self.safety_status.name
        self.safety_status_pub.publish(safety_status_msg)

        # 안전 활성화 여부 퍼블리시
        safety_active_msg = Bool()
        safety_active_msg.data = self.is_safety_active
        self.safety_active_pub.publish(safety_active_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("미션 감독 노드를 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
