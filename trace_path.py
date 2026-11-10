"""Truy ngược và in lời giải Sokoban từ state đích mà solution.solve() trả về.

File này KHÔNG thuộc bài nộp và solution.py không import nó, nên không ảnh
hưởng tới RAM hay thời gian giải map. Đường đi được lưu ngầm trong chuỗi
parent của state đích; các hàm dưới đây chỉ lần ngược chuỗi đó.

Dùng trong code:
    import solution, trace_path
    goal = solution.solve(PROBLEMS[4], 10)
    trace_path.get_actions(goal)      # ['0 right', '0 right', ...]
    trace_path.print_solution(goal)   # in từng bước dạng bản đồ

Chạy từ dòng lệnh:
    python trace_path.py <số map> [timebound] [--cols N] [--no-color]
"""
import solution


def get_path(goal):
    """Lần ngược chuỗi parent, trả list state từ trạng thái đầu tới đích; O(L).

    Mỗi state chỉ nhớ parent (state ngay trước) và action (bước vừa đi), nên
    đường đi được lưu ngầm. List chỉ chứa tham chiếu tới các state có sẵn,
    không sao chép state nào. goal là False hoặc None thì trả [].
    """
    path = []
    state = goal if goal is not False else None
    while state is not None:
        path.append(state)
        state = state.parent
    path.reverse()
    return path


def get_actions(goal):
    """Danh sách action từ đầu tới đích, ví dụ ["0 left", "1 up"]; O(L)."""
    return [state.action for state in get_path(goal)[1:]]


def _use_color(stream):
    """Chỉ tô màu khi in ra terminal có hỗ trợ ANSI; ghi ra file thì không."""
    import os
    if os.environ.get("NO_COLOR") or not hasattr(stream, "isatty") or not stream.isatty():
        return False
    if os.name != "nt":
        return True
    # Windows: chỉ các terminal hiện đại (Windows Terminal, VS Code, ...) hiểu mã ANSI.
    return any(k in os.environ for k in ("WT_SESSION", "TERM_PROGRAM", "TERM", "ANSICON"))


def _frame(state, title, note, mover, pushed, color):
    """Dựng một khung bản đồ (list dòng) với trục tọa độ; ô rộng 3 ký tự."""
    def paint(text, code):
        return "\033[%sm%s\033[0m" % (code, text) if color else text

    robot_at = {pos: i for i, pos in enumerate(state.robots)}
    lines = [title, note,
             "    " + "".join((" %d " if x < 10 else "%2d ") % x for x in range(state.width)),
             "   +" + "---" * state.width + "+"]
    for y in range(state.height):
        cells = []
        for x in range(state.width):
            pos = (x, y)
            on_goal = pos in state.storage
            if pos in state.obstacles:
                cells.append(paint("###", "90"))
            elif pos in robot_at:
                i = robot_at[pos]
                text = "(%d)" % i if on_goal else " %d " % i
                cells.append(paint(text, "1;33" if i == mover else "36"))
            elif pos in state.boxes:
                text = "[*]" if on_goal else " $ "
                code = "1;32" if on_goal else "33"
                if pos == pushed:
                    code = "1;30;42" if on_goal else "1;30;43"
                cells.append(paint(text, code))
            elif on_goal:
                cells.append(paint(" . ", "31"))
            else:
                cells.append("   ")
        lines.append("%2d |" % y + "".join(cells) + "|")
    lines.append("   +" + "---" * state.width + "+")
    return lines


def print_solution(goal, proved=None, columns=None, color=None):
    """In lời giải: các bước xếp cạnh nhau, có trục tọa độ, tô màu nếu được.

    proved: mặc định lấy từ solution.LAST_INFO của lần solve() gần nhất.
    columns: số khung mỗi hàng; None = tự tính theo bề ngang terminal.
    color: None = tự bật khi in ra terminal hỗ trợ màu; ghi ra file thì tắt.
    Chỉ in ký tự ASCII để không lỗi mã hóa trên console Windows. O(L·W·H).
    """
    import re
    import shutil
    import sys
    import textwrap

    out = sys.stdout
    path = get_path(goal)
    if not path:
        print("No solution found.")
        return
    if proved is None:
        proved = solution.LAST_INFO.get("proved", False)
    if color is None:
        color = _use_color(out)
    total = len(path) - 1
    pushes = sum(1 for a, b in zip(path, path[1:]) if a.boxes != b.boxes)
    n_boxes = len(path[0].boxes)

    frames = []
    for k, state in enumerate(path):
        on_goal = len(state.boxes & state.storage)
        mover = pushed = None
        if k == 0:
            title, note = "Step 0/%d  start" % total, "%d/%d on goal" % (on_goal, n_boxes)
        else:
            robot, direction = state.action.split()
            mover = int(robot)
            title = "Step %d/%d  robot %s %s" % (k, total, robot, direction)
            prev = state.parent
            if prev.boxes != state.boxes:
                pushed = next(iter(state.boxes - prev.boxes))
                src = next(iter(prev.boxes - state.boxes))
                note = "PUSH (%d,%d)->(%d,%d)  %d/%d on goal" % (
                    src + pushed + (on_goal, n_boxes))
            else:
                note = "%d/%d on goal" % (on_goal, n_boxes)
        frames.append(_frame(state, title, note, mover, pushed, color))

    visible = lambda s: len(re.sub(r"\033\[[0-9;]*m", "", s))
    width = max(visible(line) for f in frames for line in f)
    term = shutil.get_terminal_size((120, 24)).columns
    gap = 4
    if columns is None:
        columns = max(1, (term + gap) // (width + gap))
    bar = "=" * min(term, max(60, columns * (width + gap) - gap))

    label = "PROVEN OPTIMAL" if proved else "best found within the time limit (not proven optimal)"
    if color:
        label = "\033[1;32m%s\033[0m" % label if proved else "\033[33m%s\033[0m" % label
    print(bar)
    print(" Solution: %d steps, %d pushes, %d robot(s), %d box(es)  |  %s"
          % (total, pushes, len(path[0].robots), n_boxes, label))
    print(" Legend: 0 1 2 robot   (0) robot on goal   $ box   [*] box on goal   . goal   ### wall")
    print(bar)
    for start in range(0, len(frames), columns):
        group = frames[start:start + columns]
        for row in range(max(len(f) for f in group)):
            parts = []
            for f in group:
                line = f[row] if row < len(f) else ""
                parts.append(line + " " * (width - visible(line)))
            print((" " * gap).join(parts).rstrip())
        print()

    moves = []
    for state in path[1:]:
        robot, direction = state.action.split()
        token = robot + direction[0].upper()
        moves.append(token + "*" if state.boxes != state.parent.boxes else token)
    print(bar)
    print(" Moves (U/D/L/R = up/down/left/right, * = push):")
    for line in textwrap.wrap(" ".join(moves), max(40, len(bar) - 2)):
        print(" " + line)
    print(bar)


if __name__ == "__main__":
    import sys
    from sokoban import PROBLEMS
    args = sys.argv[1:]
    cols = None
    use_color = None
    if "--no-color" in args:
        args.remove("--no-color")
        use_color = False
    if "--cols" in args:
        i = args.index("--cols")
        cols = int(args[i + 1])
        del args[i:i + 2]
    map_index = int(args[0]) if args else 0
    seconds = float(args[1]) if len(args) > 1 else 120
    print_solution(solution.solve(PROBLEMS[map_index], seconds),
                   columns=cols, color=use_color)
