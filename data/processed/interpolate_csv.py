# -*- coding: utf-8 -*-
"""
경로를 코드 상단에서 직접 지정해 사용할 수 있는 버전.

사용 예시 1) 단일 파일만 처리:
    INPUT_PATH = r"/absolute/or/relative/path/to/your.csv"
    OUTPUT_PATH = None  # None이면 입력 파일과 같은 경로에 *_interpolate 저장
    BATCH_MODE = False

사용 예시 2) 폴더 일괄 처리:
    INPUT_PATH = r"/path/to/folder"   # 폴더로 지정
    GLOB_PATTERN = "*.csv"            # 선택 패턴
    OUTPUT_DIR = None                 # None이면 각 파일과 동일 폴더
    BATCH_MODE = True

실행:
    python interpolate_neighbors_pathcfg.py
"""
from pathlib import Path
import pandas as pd
import numpy as np

# =========[ 설정 영역: 필요에 맞게 바꾸세요 ]=========
# 단일 파일 또는 폴더 경로를 넣으세요.
INPUT_PATH = r"/home/hannibal/Mandol_ws/data/processed/yellowbox_right.csv"

# 단일 파일 모드에서 출력 파일 전체 경로를 직접 지정하려면 값 지정.
# None이면 INPUT_PATH와 같은 폴더에 "<원본파일명>_interpolate.csv"로 저장합니다.
OUTPUT_PATH = None

# True로 두면 폴더 일괄 처리 모드가 됩니다.
BATCH_MODE = False

# BATCH_MODE=True일 때만 사용: 폴더 내에서 어떤 파일을 고를지 패턴 지정
GLOB_PATTERN = "*.csv"

# BATCH_MODE=True일 때만 사용: 출력 폴더 지정 (None이면 각 입력 파일과 동일 폴더)
OUTPUT_DIR = None
# ================================================


def insert_midpoints_between_neighbors(df: pd.DataFrame,
                                       x_col: str = "X(E/m)",
                                       y_col: str = "Y(N/m)",
                                       dir_col: str = "direction") -> pd.DataFrame:
    for c in (x_col, y_col, dir_col):
        if c not in df.columns:
            raise KeyError(f"필수 컬럼이 존재하지 않습니다: {c}")

    rows = []
    cols = list(df.columns)
    n = len(df)
    for i in range(n):
        rows.append(df.iloc[i].to_dict())
        if i < n - 1:
            left = df.iloc[i]
            right = df.iloc[i + 1]
            mid = {c: np.nan for c in cols}
            mid[x_col] = (left[x_col] + right[x_col]) / 2.0
            mid[y_col] = (left[y_col] + right[y_col]) / 2.0
            mid[dir_col] = left[dir_col]
            rows.append(mid)
    return pd.DataFrame(rows, columns=cols)


def process_single_file(input_csv: Path, output_csv: Path | None) -> Path:
    df = pd.read_csv(input_csv)
    out = insert_midpoints_between_neighbors(df)
    if output_csv is None:
        output_csv = input_csv.with_name(input_csv.stem + "_interpolate" + input_csv.suffix)
    out.to_csv(output_csv, index=False)
    print(f"[완료] {input_csv.name} -> {output_csv}")
    return output_csv


def process_batch(input_dir: Path, pattern: str, output_dir: Path | None) -> None:
    if not input_dir.is_dir():
        raise ValueError(f"BATCH_MODE이지만 폴더가 아닙니다: {input_dir}")
    files = sorted(input_dir.glob(pattern))
    if not files:
        print(f"[안내] 패턴 '{pattern}'에 해당하는 파일이 없습니다: {input_dir}")
        return
    for f in files:
        dst = None
        if output_dir is not None:
            output_dir.mkdir(parents=True, exist_ok=True)
            dst = output_dir / (f.stem + "_interpolate" + f.suffix)
        process_single_file(f, dst)


def main():
    ip = Path(INPUT_PATH)
    if BATCH_MODE:
        od = Path(OUTPUT_DIR) if OUTPUT_DIR else None
        process_batch(ip, GLOB_PATTERN, od)
    else:
        if not ip.exists():
            raise FileNotFoundError(f"입력 경로를 찾을 수 없습니다: {ip}")
        if ip.is_dir():
            # 안전장치: 단일 모드인데 폴더가 들어왔다면 첫 번째 CSV만 처리
            csvs = sorted(ip.glob("*.csv"))
            if not csvs:
                raise FileNotFoundError(f"폴더에 CSV가 없습니다: {ip}")
            target = csvs[0]
            print(f"[경고] 단일 파일 모드인데 폴더가 지정되어 첫 번째 CSV만 처리합니다: {target.name}")
            process_single_file(target, Path(OUTPUT_PATH) if OUTPUT_PATH else None)
        else:
            process_single_file(ip, Path(OUTPUT_PATH) if OUTPUT_PATH else None)

if __name__ == "__main__":
    main()