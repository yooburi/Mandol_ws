# Mandol_ws — ROS 2 자율주행 포트폴리오

Mandol_ws는 ROS 2 Humble 기반의 자율주행 실차/시뮬레이션을 위한 작업공간으로, GPS/RTK 기반 로컬라이제이션, 경로계획, 비전 인지, 미션 판단, 차량 구동까지 전체 파이프라인을 모듈화했습니다. 이 리포지토리의 핵심은 미션 레벨의 판단을 수행하는 `mission_supervisor` 노드입니다.

**핵심 가치**
- 안전 선점(Safety preemption)과 미션 전이를 분리한 2계층 상태기계 설계
- 인지/계획/제어 패키지와의 느슨한 결합을 통한 손쉬운 통합
- 실제 대회 트랙과 시나리오에 맞춘 실전형 토픽/파라미터 설계

**구성 개요**
- 센서: u-blox F9P/F9R, USB 카메라, RealSense 등
- 로컬라이제이션/맵: `gps_to_utm` (UTM 변환, TF, CSV 맵 퍼블리시)
- 경로계획/조향: `path_planning` (ROI 경로, Pure Pursuit, 미션 트리거)
- 인지: `mando_vision` (신호등/장애물 인지 → 정지 신호)
- 미션 판단: `mission_supervisor` (미션/안전 상태 결정, 스로틀 게이팅)
- 액추에이션: `serial_bridge` (스로틀/조향 시리얼 전송)

---

## mission_supervisor (핵심)

미션 레이어와 세이프티 레이어를 분리한 선점형 상태기계 노드입니다. 안전 신호가 활성화되면 어떤 미션 상태에서도 즉시 선점하고, 해제 조건을 만족하면 이전 미션으로 자연스럽게 복귀합니다.

- 노드: `mission_supervisor_node` (`mission_supervisor.mission_supervisor:main`)
- 언어/런타임: Python, rclpy

**상태 구조**
- 미션 상태(`MissionState`): `INIT` → `GPS_FWD` ↔ `REVERSE_T` ↔ `REVERSE_PARALLEL`
- 안전 상태(`SafetyStatus`): `SAFE_OK`, `STOP_SLOPE`, `STOP_OBSTACLE`, `STOP_TRAFFIC`
- 활성 알고리즘(`ActiveAlgorithm`): `FWD_CONTROLLER`, `REVERSE_T_CONTROLLER`, `REVERSE_PARALLEL_CONTROLLER`, `SAFETY_HOLD`

**구독 토픽 (입력)**
- `/slope_stop` (`Bool`): 경사로 정지 신호
- `/traffic_stop` (`Bool`): 신호등 정지 신호
- `/intersection` (`Bool`): 교차로 진입 여부 (신호등 정지 유효화)
- `/obstacle_existance` (`Bool`): 장애물 존재 신호
- `/reverse_T/trigger`, `/reverse_T/done` (`Bool`): T자 후진 시작/완료
- `/reverse_parallel/trigger`, `/reverse_parallel/done` (`Bool`): 평행 주차 시작/완료
- `/throttle_from_planning` (`Float32`): 경로계획(예: Pure Pursuit)에서 제안한 스로틀

**퍼블리시 토픽 (출력)**
- `/throttle_cmd` (`Float32`, 기본): 미션/안전 로직이 게이팅한 최종 스로틀 명령
- `/mission_state` (`String`): 현재 미션 상태
- `/active_algorithm` (`String`): 활성화된 하위 알고리즘
- `/safety_status` (`String`): 현재 안전 상태
- `/safety_active` (`Bool`): 안전 선점 활성화 여부

**주요 파라미터**
- `loop_rate_hz` (기본 50.0): 메인 루프 주기
- `hysteresis_sec` (기본 0.5): 정지 신호 해제 후 히스테리시스
- `slope_hold_sec` (기본 5.0): 경사로 정지 유지 시간
- `obstacle_hold_sec` (기본 5.0): 장애물 정지 유지 시간
- `direction_switch_hold_sec` (기본 2.0): 전/후진 전환 시 정지 유지 시간
- `throttle_topic` (기본 `/throttle_cmd`): 스로틀 명령 출력 토픽명

**동작 요약**
- 세이프티 레이어(선점):
  - 경사로(`STOP_SLOPE`)는 세션 단위로 5초 유지, 유지 중에는 스로틀 0.15, 이후 같은 세션에서 재선점 방지
  - 장애물(`STOP_OBSTACLE`)은 5초 유지 후 해제 히스테리시스 적용
  - 신호등(`STOP_TRAFFIC`)은 교차로 내부에서만 유효, 해제 히스테리시스 적용
  - 해제 시 이전 미션 상태로 복귀, `safety_active` 비활성화
- 미션 레이어: 트리거/완료 신호 상승에지 기반 전이, 안전 선점 중에는 전이 금지
- 스로틀 출력 정책:
  - 전/후진 방향 전환 감지 시 `direction_switch_hold_sec` 동안 강제 정지(0)
  - 안전 선점 시: 경사로만 0.15, 그 외 0.0
  - 정상 시: `GPS_FWD`는 계획 스로틀 사용, `REVERSE_*`는 고정 후진 스로틀(-0.2)

**시나리오 예시**
- 교차로 내 신호등 정지 활성 → 즉시 정지, 해제 히스테리시스 후 복귀
- 경사로 정지 활성 → 5초 유지 후 복귀(같은 세션 재선점 방지)
- T자/평행 주차 트리거 → 후진 미션 진입, 완료 신호로 `GPS_FWD` 복귀

---

## 주요 패키지 요약

- `gps_to_utm` (C++/Python)
  - 기능: GPS Fix → UTM 변환(`f9r_to_utm`, `f9p_to_utm`), 방위각 계산, CSV 경로 TF/시각화(`tf_gps_csv_node`), bag→CSV 유틸리티
  - 메시지/의존: `rclcpp`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, `rosbag2_*`

- `path_planning` (C++)
  - 기능: ROI 경로 생성(`f9r_roi_path`), Pure Pursuit(`pure_pursuit_node`)로 `/auto_steer_angle`, `/throttle_from_planning` 산출, CSV 영역 기반 미션 트리거 발행
  - 인터페이스: `/slope_stop`, `/intersection`, `/reverse_*` 트리거/완료, `/throttle_from_planning`

- `mando_vision` (Python)
  - 기능: 신호등/장애물 인지(ONNX 추론) → `/traffic_stop`, `/obstacle_existance` 생성, 시각화 이미지 퍼블리시

- `serial_bridge` (Python)
  - 기능: `/throttle_cmd`, `/auto_steer_angle`를 아두이노로 시리얼 전송 (포트/보드레이트 파라미터화)

- `RTK_GPS_NTRIP`, `usb_cam` (서드파티)
  - 기능: RTK 보정수신/유블럭스 드라이버, USB 카메라 드라이버

---

## 빌드/실행 (요약)

- 요구사항: ROS 2 Humble, colcon, 필요한 ROS 패키지들(각 패키지 `package.xml` 참조)
- 빌드: 작업공간 루트에서 `colcon build` → `source install/setup.bash`
- 예시 실행(핵심): `ros2 run mission_supervisor mission_supervisor_node`

실제 센서/지도/계획 노드의 상세 실행 방법은 각 패키지의 Readme 및 설정 파일(예: `src/path_planning/config/*`, `src/gps_to_utm/config/*`)을 참고하세요. 본 문서는 포트폴리오 목적의 개요를 제공합니다.

---

## 아키텍처 한눈에 보기

- 인지(`mando_vision`) → `/traffic_stop`, `/obstacle_existance`
- 로컬라이제이션/맵(`gps_to_utm`) → UTM/TF/CSV 경로
- 경로계획(`path_planning`) → `/auto_steer_angle`, `/throttle_from_planning`, 미션 트리거
- 미션 판단(`mission_supervisor`) → `/throttle_cmd` 게이팅·상태 퍼블리시
- 액추에이션(`serial_bridge`) → 시리얼로 차량 구동

---

## 라이선스/저작권

이 워크스페이스는 자체 코드와 서드파티 패키지를 함께 포함합니다. 패키지별 라이선스는 각 폴더의 `package.xml` 혹은 라이선스 파일을 참고하세요.

## Maintainer

- yoo <smzzang21@konkuk.ac.kr>

