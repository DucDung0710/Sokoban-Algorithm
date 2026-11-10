"""Chạy solver trên từng map thực hành (1-20) trong sokoban.PROBLEMS.

Cách dùng:
    python run_map.py              # hỏi số map rồi chạy
    python run_map.py 1            # chạy map 1
    python run_map.py 1 5 7-10     # chạy nhiều map / khoảng map
    python run_map.py all          # chạy cả 20 map
    python run_map.py 3 -t 30      # timebound 30 giây
    python run_map.py 3 -p         # in từng bước của lời giải
    python run_map.py 3 -q         # không in bàn chơi ban đầu / cuối

Mỗi map chạy trong một tiến trình riêng có giới hạn cứng (timebound + 10s),
giống cách autograder chạy, nên solver bị treo cũng không làm kẹt cả script.
Đặt file này cùng thư mục với solution.py, sokoban.py, validation.py.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
RESULT_TAG = "@@RESULT@@"


def force_utf8_stdio():
    """Ép stdout/stderr dùng UTF-8 để in tiếng Việt không bị UnicodeEncodeError.

    Trên Windows, khi stdout là pipe (tiến trình con) hoặc bị chuyển hướng ra file,
    Python dùng bảng mã ANSI (cp1252/cp1258) nên in chữ như 'đ', 'ạ', 'ả' sẽ crash.
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass


# ---------------------------------------------------------------- worker ---
def run_worker(index, timebound, show_board, print_path):
    """Chạy solve() cho 1 map trong tiến trình hiện tại, in kết quả."""
    sys.path.insert(0, str(ROOT))
    from sokoban import PROBLEMS, SokobanState
    from validation import signature, validate_solution

    def fresh(state):
        return SokobanState("START", 0, None, state.width, state.height,
                            state.robots, state.boxes, state.storage, state.obstacles)

    original = PROBLEMS[index - 1]
    initial = fresh(original)
    before = (signature(initial), initial.action, initial.gval, initial.parent)

    print("Map {}: {}x{}, {} robot, {} thùng, {} ô đích".format(
        index, initial.width, initial.height, len(initial.robots),
        len(initial.boxes), len(initial.storage)))
    if show_board:
        print(initial.state_string())

    import solution
    start = time.perf_counter()
    try:
        final = solution.solve(initial, timebound=timebound)
        error = None
    except Exception as exc:  # crash của solver
        import traceback
        traceback.print_exc()
        final, error = False, "{}: {}".format(type(exc).__name__, exc)
    elapsed = time.perf_counter() - start

    result = dict(map=index, time=round(elapsed, 3), steps=None)
    if error:
        result.update(status="CRASH", detail=error)
    elif (signature(initial), initial.action, initial.gval, initial.parent) != before:
        result.update(status="INVALID", detail="solve() đã sửa trạng thái đầu vào")
    elif final is False:
        result.update(status="NO_SOLUTION", detail="solve() trả về False")
    else:
        valid, detail = validate_solution(fresh(original), final)
        result.update(status="PASS" if valid else "INVALID", detail=detail)
        if valid:
            result["steps"] = final.gval
            if print_path:
                final.print_path()
            elif show_board:
                print("Trạng thái cuối:")
                print(final.state_string())

    print(RESULT_TAG + json.dumps(result, ensure_ascii=False))


# ---------------------------------------------------------------- driver ---
def parse_maps(tokens, total):
    maps = []
    for token in tokens:
        for part in token.replace(",", " ").split():
            if part.lower() == "all":
                maps.extend(range(1, total + 1))
            elif "-" in part:
                lo, hi = part.split("-", 1)
                maps.extend(range(int(lo), int(hi) + 1))
            else:
                maps.append(int(part))
    bad = [m for m in maps if not 1 <= m <= total]
    if bad:
        raise ValueError("Map không hợp lệ: {} (chỉ có 1-{})".format(bad, total))
    return list(dict.fromkeys(maps))  # bỏ trùng, giữ thứ tự


def run_one(index, args):
    cmd = [sys.executable, str(Path(__file__).resolve()), "--worker", str(index),
           "-t", str(args.timebound)]
    if args.quiet:
        cmd.append("-q")
    if args.path:
        cmd.append("-p")
    hard_limit = args.timebound + 10
    # Ép tiến trình con ghi UTF-8 (khớp với encoding="utf-8" khi đọc bên dưới).
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1")
    try:
        run = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True,
                             encoding="utf-8", errors="replace", timeout=hard_limit,
                             env=env)
    except subprocess.TimeoutExpired as exc:
        out = exc.stdout or ""
        if isinstance(out, bytes):
            out = out.decode("utf-8", "replace")
        print(out, end="")
        return dict(map=index, status="TIMEOUT", time=hard_limit, steps=None,
                    detail="Quá giới hạn cứng {}s".format(hard_limit))

    result = None
    for line in run.stdout.splitlines():
        if line.startswith(RESULT_TAG):
            result = json.loads(line[len(RESULT_TAG):])
        else:
            print(line)
    if run.stderr.strip():
        print(run.stderr.rstrip())
    if result is None:
        result = dict(map=index, status="CRASH", time=None, steps=None,
                      detail="Tiến trình kết thúc bất thường (mã {})".format(run.returncode))
    return result


def main():
    force_utf8_stdio()
    parser = argparse.ArgumentParser(
        description="Chạy solver Sokoban trên từng map.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    parser.add_argument("maps", nargs="*", help="Số map, ví dụ: 1  hoặc  1 3 5-8  hoặc  all")
    parser.add_argument("-t", "--timebound", type=float, default=120,
                        help="timebound truyền vào solve() (giây, mặc định 120)")
    parser.add_argument("-p", "--path", action="store_true", help="In từng bước lời giải")
    parser.add_argument("-q", "--quiet", action="store_true", help="Không in bàn chơi")
    parser.add_argument("--worker", type=int, help=argparse.SUPPRESS)
    args = parser.parse_args()

    if args.worker is not None:
        run_worker(args.worker, args.timebound, not args.quiet, args.path)
        return 0

    sys.path.insert(0, str(ROOT))
    from sokoban import PROBLEMS
    total = len(PROBLEMS)

    tokens = args.maps
    if not tokens:
        try:
            tokens = [input("Nhập số map (1-{}, vd: 1 | 2 4 6 | 5-10 | all): ".format(total))]
        except EOFError:
            return 1
    try:
        maps = parse_maps(tokens, total)
    except ValueError as exc:
        parser.error(str(exc))
    if not maps:
        parser.error("Chưa nhập map nào")

    results = []
    for index in maps:
        print("=" * 60)
        res = run_one(index, args)
        results.append(res)
        steps = "{} bước".format(res["steps"]) if res["steps"] is not None else "-"
        t = "{:.3f}s".format(res["time"]) if res["time"] is not None else "-"
        print(">> Map {}: {} | {} | {}".format(index, res["status"], steps, t))
        if res["status"] != "PASS":
            print("   " + str(res.get("detail", "")))

    if len(results) > 1:
        print("=" * 60)
        print("{:>4} | {:<11} | {:>6} | {:>9}".format("Map", "Kết quả", "Bước", "Thời gian"))
        print("-" * 40)
        for r in results:
            print("{:>4} | {:<11} | {:>6} | {:>9}".format(
                r["map"], r["status"], r["steps"] if r["steps"] is not None else "-",
                "{:.3f}s".format(r["time"]) if r["time"] is not None else "-"))
        passed = [r for r in results if r["status"] == "PASS"]
        print("-" * 40)
        print("PASS {}/{} | Tổng bước: {} | Tổng thời gian: {:.3f}s".format(
            len(passed), len(results), sum(r["steps"] for r in passed),
            sum(r["time"] or 0 for r in results)))
    return 0 if all(r["status"] == "PASS" for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())