# 🧭 ROS2 `mission_supervisor_node` 작성 프롬프트 (한국어)

## 개요
ROS2(rclpy, Python) 기반의 **두 계층 상태 기계(state machine)** 노드 **`mission_supervisor_node`** 를 작성하기 위한 요구사항 정리.

---

## 1. 선점 세이프티 레이어 (항상 실행)

- 외부 정지 신호를 **상시 모니터링**  
- 조건이 **true**면 즉시 **선점(preempt)**  
  - `/throttle_cmd` = 0 (`Float32`) publish  
  - 미션 상태는 변경하지 않음  
- 조건이 **false 유지 + hysteresis (0.5s 기본)** 후 **이전 미션 상태** 복귀  
- 세부 규칙  
  - **STOP/SLOPE**: 즉시 정지 → 5초 유지 후 SAFE 상태 → hysteresis 통과 시 복귀  
  - **STOP/OBSTACLE**: 즉시 정지 → 5초 유지 후 hysteresis 통과 시 GPS_FWD 복귀  
  - **STOP/TRAFFIC**: 즉시 정지 → 신호 해제 + hysteresis 후 복귀  
- 상태 publish  
  - `safety_status` (string: SAFE_OK, STOP/SLOPE, STOP/TRAFFIC, STOP/OBSTACLE)  
  - `safety_active` (bool)

---

## 2. 미션 레이어 (진행)

- 기본 상태: `GPS_FWD`  
- 동작 순서  
  1. 세이프티 확인 → active면 선점 후 미션 전환 skip  
  2. 세이프티 inactive면 미션 전환 조건 평가  
  3. 현재 미션 상태에 맞는 컨트롤러 활성화 신호 publish  

### 상태 전이 그래프

- INIT --(초기화/시작)--> GPS_FWD

- GPS_FWD --(/reverse_T/trigger==true)--> REVERSE_T
- REVERSE_T --(/reverse_T/done==true)--> GPS_FWD

- GPS_FWD --(/reverse_parallel/trigger==true)--> REVERSE_PARALLEL
- REVERSE_PARALLEL --(/reverse_parallel/done==true)--> GPS_FWD


---

## 3. 세이프티 조건

- `STOP/SLOPE` → `/slope_stop : std_msgs/Bool`  
- `STOP/TRAFFIC` → `/traffic_stop : std_msgs/Bool`  
- `STOP/OBSTACLE` → `/obstacle_existance : std_msgs/Bool`  

**해제 규칙:** false가 `hysteresis_sec` 동안 유지되어야 함  
**우선순위:** `STOP/SLOPE` > `STOP/OBSTACLE` > `STOP/TRAFFIC`

---

## 4. ROS2 인터페이스

### Subscribers
- `/slope_stop`, `/traffic_stop`, `/obstacle_existance`  
- `/reverse_T/trigger`, `/reverse_T/done`  
- `/reverse_parallel/trigger`, `/reverse_parallel/done`

### Publishers
- `/throttle_cmd : Float32` → 세이프티 active 시 0.0  
- `/mission_state : String`  
- `/active_algorithm : String`  
  - 값: `FWD_CONTROLLER`, `REVERSE_T_CONTROLLER`, `REVERSE_PARALLEL_CONTROLLER`, `SAFETY_HOLD`  
- `/safety_status : String`  
- `/safety_active : Bool`

### Parameters
- `loop_rate_hz` (기본 50.0)  
- `hysteresis_sec` (기본 0.5)  
- `slope_hold_sec` (기본 5.0)  
- `obstacle_hold_sec` (기본 5.0)  
- `throttle_topic` (`/throttle_cmd`)

---

## 5. 구현 세부

- 타이머 루프에서 세이프티 → 미션 순서로 판단  
- trigger/done은 **상승 에지** 검출  
- 로그 출력 (`get_logger().info`)  
- 미션 상태는 `Enum` 사용  
- 세이프티 해제 시 `previous_mission_state` 복귀  

---

## 6. 요구사항

- 단일 파일 `decision_node.py`  
- 클래스 구조: `DecisionNode(rclpy.node.Node)`  
- `if __name__ == "__main__":` 포함  
- 코드 상단에 ASCII 다이어그램과 상태표 요약 주석 포함  
- ROS2 표준만 사용, 외부 라이브러리 의존 없음  

---

## 7. 테스트 시나리오 (주석 포함)

- **A. 세이프티 선점**  
  - `/traffic_stop=true` → throttle 0, SAFETY_HOLD  
  - `/traffic_stop=false` → hysteresis 후 GPS_FWD 복귀  

- **B. Slope Hold**  
  - `/slope_stop=true` → 5초 정지 유지 후 복귀  

- **C. Obstacle Hold**  
  - `/obstacle_existance=true` → 5초 정지 후 복귀  

- **D. 미션 전환**  
  - `/reverse_T/trigger` → REVERSE_T  
  - `/reverse_T/done` → GPS_FWD  

- **E. 상태 토픽 일관성**  
  - mission_state, active_algorithm, safety_status, safety_active 가 항상 올바르게 반영  

---

**➡️ 이제 전체 Python 코드를 작성하세요.**
