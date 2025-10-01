from pathlib import Path

in_path = Path("/home/hannibal/Mandol_ws/data/processed/1_T_right_P_right.csv")

# 출력 파일명: 기존 이름 뒤에 _reverse 붙이기
if in_path.suffix.lower() == ".csv":
    out_path = in_path.with_name(in_path.stem + "_reverse" + in_path.suffix)
else:
    out_path = in_path.with_name(in_path.name + "_reverse")

# 빈 행 보존을 위해 텍스트 라인 단위로 처리
with in_path.open("r", encoding="utf-8", errors="replace", newline=None) as f:
    lines = f.read().splitlines()

if not lines:
    raise ValueError("입력 CSV가 비어 있습니다.")

header = lines[0]
rest = lines[1:]
rest_reversed = list(reversed(rest))

# 헤더 유지 + 나머지 역순으로 기록
with out_path.open("w", encoding="utf-8", newline="\n") as f:
    f.write(header + "\n")
    if rest_reversed:
        f.write("\n".join(rest_reversed) + "\n")

print(f"Saved: {out_path}")
