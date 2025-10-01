#pragma once
#include <vector>
#include <cstddef>

namespace path_planning {

/**
 * GPU 헬퍼:
 * - 경로 갱신 시: transform_and_scan(...) 호출 (이미 사용 중)
 * - 타이머 주기: argmin_window_xpos_async(...)로 윈도우 내 x>0 최근접 인덱스 비동기 계산
 */
class GPURoiHelper {
public:
  GPURoiHelper();
  ~GPURoiHelper();

  // 경로 업데이트(원본 → 타겟프레임 변환 + 누적호길이)
  bool transform_and_scan(const std::vector<float>& x_in,
                          const std::vector<float>& y_in,
                          float cos_yaw, float sin_yaw,
                          float tx, float ty,
                          std::vector<float>& x_out,
                          std::vector<float>& y_out,
                          std::vector<double>& arc_out);

  // 변환된 좌표(x_out, y_out)를 디바이스에 보관(타이머용). transform_and_scan 뒤에 호출.
  void upload_transformed_xy(const std::vector<float>& x_out,
                             const std::vector<float>& y_out);

  // 타이머용: last_idx를 중심으로 윈도우 [i0, i1)에서 x>0만 대상으로 argmin (비동기).
  // - launch만 하고 즉시 복귀 (동기화 안 함)
  // - 결과는 try_fetch_argmin_result(...)로 가져감
  // - 범위를 벗어나면 자동으로 [0,N)로 클램핑
  void argmin_window_xpos_async(int last_idx, int window_radius);

  // 직전 async 호출 결과를 가져온다. (준비 안 되었으면 false)
  bool try_fetch_argmin_result(int& out_index);

  // 현재 디바이스에 올라간 포인트 개수
  int size() const;

  void reset();

private:
  struct Impl;
  Impl* pimpl_;
};

} // namespace path_planning
