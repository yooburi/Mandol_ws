# f9r\_roi\_path 노드 설명서

이 문서는 `/f9r_roi_path.cpp`의 동작을 **한눈에** 이해할 수 있도록 정리한 마크다운 가이드입니다. ROS 2 Humble + C++ 기준입니다.

---

## 개요

* 입력: `/csv_path` (`nav_msgs/Path`) – **CSV 프레임**에서 저장된 전체 기준 경로
* (옵션) 입력: `/f9r/fix` (`sensor_msgs/NavSatFix`) – 타이밍/상태용
* TF: `target_frame`(차량 원점) → `csv_frame`(경로 프레임) 변환으로 **차량의 현재 위치** 산출
* 출력:

  * `/f9r_roi_path` (`visualization_msgs/Marker`, LINE\_STRIP) – ROI 구간
  * `/f9r_roi_end`  (`visualization_msgs/Marker`, SPHERE) – ROI 끝점

**핵심 목표**

1. **인덱스 단조 비감소**: 시작 인덱스는 절대 뒤로 가지 않음
2. **전방 탐색만**: 역방향/역주행 선택 금지
3. **점프 방지 제약**: **직전 프레임의 ROI 위에 있었던 점들** 중에서만 다음 시작점 선택

---

## 프레임 & 토픽

* `target_frame` (기본 `f9r`): 차량의 현재 위치 프레임 (보통 `base_link`/`base_footprint`)
* `csv_frame` (기본 `csv`): `/csv_path`가 정의된 고정 지도 프레임(대개 UTM\[m])

노드는 `TF`를 이용해 `target_frame` 원점(0,0,0)을 `csv_frame`으로 변환하여 \*\*차량의 현재 (x,y)\*\*를 사용합니다.

---

## 파라미터

| 이름                   |     기본값 | 단위  | 의미 / 사용처              | 변경 시 영향                          |
| -------------------- | ------: | --- | --------------------- | -------------------------------- |
| `target_frame`       | `"f9r"` | -   | 차량 원점 프레임명            | TF 변환 실패 시 ROI 갱신 불가             |
| `csv_frame`          | `"csv"` | -   | 경로가 정의된 프레임명          | RViz 표시/거리 계산 기준                 |
| `timer_frequency`    |  `20.0` | Hz  | ROI 갱신 주기             | 높을수록 반응성↑/부하↑                    |
| `roi_length_m`       |   `7.0` | m   | 거리 기반 ROI 길이          | `use_points_length=false`일 때만 유효 |
| `use_points_length`  | `false` | -   | ROI 길이 단위를 포인트 개수로 전환 | true면 `roi_length_points` 사용     |
| `roi_length_points`  |    `50` | pts | 포인트 개수 기반 ROI 길이      | 점 간격이 일정할 때 편리                   |
| `search_span_points` |  `2000` | pts | **초기 1회** 전방 탐색 폭     | 초기 락온 실패/속도에 영향                  |
| `hysteresis_k`       |     `0` | idx | 시작점 갱신 히스테리시스         | 미세 흔들림 억제, 반응성 저하 가능             |
| `line_width`         |   `0.2` | m   | ROI 선 두께 (RViz)       | 시각화만 영향                          |
| `end_marker_size`    |   `0.7` | m   | 끝점 구체 크기 (RViz)       | 시각화만 영향                          |

추천값 예시는 마지막 섹션 참고.

---

## 알고리즘 흐름

1. **경로 수신** (`/csv_path`)

   * `nav_msgs/Path`를 내부 벡터 `csv_pts_`(x,y)로 저장
   * 경로가 실제로 바뀌었을 때만 인덱스 상한 클램프(리셋 금지)

2. **현재 위치 계산**

   * `target_frame` 원점 → `csv_frame`으로 TF 변환 → (xf, yf)

3. **시작점 후보 구간 설정**

   * 첫 퍼블리시 전: `last_start_idx_`부터 `search_span_points`만큼 **전방**
   * 이후 모든 프레임: **직전 ROI** `[last_start_idx_, last_end_idx_]` **내부로 제한**

     * 👉 다른 경로/인접 루프 등으로 “점프” 불가

4. **후보 구간에서 최근접점 탐색**

   * (xf, yf)와 각 후보점 간 **거리 제곱** 최솟값 인덱스 `nearest_idx`

5. **단조성 + 히스테리시스**

   * `start_idx = max(nearest_idx, last_start_idx_)`
   * `hysteresis_k > 0`이면 `nearest_idx < last_start_idx_ + k`일 때 **유지**

6. **ROI 길이 확정**

   * 거리 기반: 인접 점 간 거리 누적 ≥ `roi_length_m`가 되는 인덱스까지
   * 개수 기반: `start_idx + roi_length_points`

7. **출력 퍼블리시**

   * `/f9r_roi_path`: LINE\_STRIP (녹색)
   * `/f9r_roi_end`: SPHERE (빨강)
   * 이후 `last_start_idx_ = start_idx`, `last_end_idx_ = end_idx`, `published_once_ = true`

---

## “직전 ROI 위에서만 시작점 선정” 제약

* **초기 1회**만 넓게 전방 탐색해 올바른 트랙에 **락온**
* 이후에는 시작점 후보를 **직전 프레임에서 실제로 그렸던 ROI 구간**으로 제한
* 결과: 분기/교차로에서 다른 라인으로 **점프**하는 현상 방지

---

## 인덱스 단조성 보장

* `start_idx = max(nearest_idx, last_start_idx_)`
* 어떤 상황에서도 시작 인덱스는 **감소하지 않음**
* 경로 재수신 시에도 **리셋 금지**(경로 변경 시 상한 보정만)

---

## QoS (Reliability / Durability)

* 구독 `/csv_path`: `QoS(1).reliable().transient_local()`
* 발행 `/f9r_roi_path`, `/f9r_roi_end`: 동일하게 `reliable().transient_local()`
* 장점: RViz/노드 재연결 시 **마지막 메시지**를 곧바로 재전달
* 주의: Transient Local은 상태 관리(인덱스 리셋 금지) 문제를 해결해주지 않음

---

## 스레드 안전

* `pathCallback`(구독)과 `timerCallback`(타이머)이 공유 상태를 만지므로 `std::mutex` 사용
* 콜백 내부에서는 **락 최소화**를 위해 **로컬 복사본**으로 계산 → 결과만 반영

---

## 실패/경계 케이스

* **TF 변환 실패**: 로그 경고 + 프레임 스킵

  * `target_frame`와 `csv_frame` 이름/TF 트리 확인
* **빈 경로**: `/csv_path` 비었을 때 ROI 퍼블리시 중단
* **경로 변경**: 크기/표본점 비교로 **변경 감지**, 인덱스 상한만 클램프

---

## RViz 시각화

* `/f9r_roi_path`: LINE\_STRIP, 굵기 `line_width` (m), 녹색
* `/f9r_roi_end`: SPHERE, 크기 `end_marker_size` (m), 붉은 점
* 표시 프레임: `csv_frame` (좌표 맞춰보기 쉬움)

---

## 설정 예시 (YAML)

```yaml
f9r_roi_path:
  ros__parameters:
    target_frame: "base_link"
    csv_frame: "csv"
    timer_frequency: 20.0
    use_points_length: false
    roi_length_m: 20.0       # 거리 기반
    # use_points_length: true
    # roi_length_points: 50   # 점 간격≈1m라면 ≈50m
    search_span_points: 3000
    hysteresis_k: 2
    line_width: 0.2
    end_marker_size: 0.7
```

---

## 튜닝 가이드

* **초기 락온이 늦다** → `search_span_points` ↑
* **시작점이 자잘하게 흔들린다** → `hysteresis_k` 1\~3
* **ROI 너무 짧/길다** → `roi_length_m` 또는 `roi_length_points` 조정
* **고속에서 반응이 둔하다** → `timer_frequency` ↑ (CPU 여유 고려)

---

✦ pure_pursuit.cpp 파일에서 /throttle_from_planning 토픽의 값은 현재 계산된 조향각(steering angle)에 기반하여 결정됩니다. 핵심 로직은 "조향각이 클수록 
  속도를 줄이는 것"입니다.

  계산 과정은 onTimer 함수 내의 7) 스로틀 계산 섹션에 있으며, 다음과 같은 단계로 이루어집니다.

   1. 조향각 정규화 (Normalization)
       * 먼저, 계산된 최종 조향각(steer_out)의 절댓값을 조향각 제한(limit)으로 나누어 0과 1 사이의 값으로 정규화합니다.
       * 이 값(abs_steer_normalized)은 0일 때 직진, 1일 때 최대 조향 상태를 의미합니다.

   1     // 7) 스로틀 계산 
   2     const double abs_steer_normalized = std::abs(steer_out) / limit; // 정규화된 조향각 크기 (0~1)

   2. 기본 스로틀 값 계산
       * 정규화된 조향각을 이용해 max_throttle_과 min_throttle_ 사이에서 스로틀 값을 선형적으로 계산합니다.
       * 조향각이 0(직진)이면 throttle_value는 max_throttle_이 됩니다.
       * 조향각이 최대치(limit)이면 throttle_value는 min_throttle_이 됩니다.

   1     // 조향각이 클수록 스로틀을 줄임
   2     double throttle_value = max_throttle_ - abs_steer_normalized * (max_throttle_ - min_throttle_);
   3     throttle_value = clamp(throttle_value, min_throttle_, max_throttle_);

   3. EMA 필터 적용 (Smoothing)
       * 계산된 스로틀 값이 급격하게 변하는 것을 방지하기 위해 지수이동평균(Exponential Moving Average, EMA) 필터를 적용합니다.
       * throttle_ema_alpha_ 파라미터가 이 필터의 강도를 조절합니다. (1에 가까울수록 현재 값에 가중, 0에 가까울수록 이전 값에 가중)

   1     // 스로틀 값 EMA 필터 적용
   2     if (!throttle_filt_.has_value()) {
   3       throttle_filt_ = throttle_value;
   4     } else {
   5       throttle_filt_ = throttle_ema_alpha_ * throttle_value + (1.0 - throttle_ema_alpha_) * throttle_filt_.value();
   6     }

   4. 최종 값 발행 (Publish)
       * 필터링된 최종 스로틀 값을 /throttle_from_planning 토픽으로 발행합니다.
   1     publishThrottle(throttle_filt_.value());

  요약:
  /throttle_from_planning 값은 코너를 돌 때(조향각이 커질 때) 감속하고, 직진할 때(조향각이 작아질 때) 가속하도록 조향각에 반비례하여 계산된 후, 부드러운
  주행을 위해 EMA 필터를 거쳐 최종적으로 발행됩니다. 관련 파라미터는 min_throttle, max_throttle, throttle_ema_alpha 입니다.
