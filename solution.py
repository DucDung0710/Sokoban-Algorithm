# TODO (KHU 0 - chu A): Khai bao import, hang so va co bat/tat.
# TODO (KHU 1 - chu B): Xay board, state noi bo, successors_fast va rebuild.
# TODO (KHU 2 - chu C): Cai dat heuristic admissible.
# TODO (KHU 3 - chu D): Cai dat phat hien deadlock sound.
# TODO (KHU 4 - chu A): Cai dat tim kiem va quan ly ngan sach thoi gian.
# TODO (KHU 5 - chu A): Cai dat solve(initial_state, timebound=120).


from collections import deque
import gc
import heapq
import itertools
import time
from sokoban import sokoban_goal_state

#Tỉ lệ thời gian đc dùng
TIME_FRACTION = 0.85
#Cứ mỗi 500 node được mở ra thì kiểm tra đồng hồ
CHECK_EVERY = 500
#Số state tối đa được lưu trong visited, tránh tràn bộ nhớ
MAX_STATES = 2000000 
INF = float("inf")
HEURISTIC = "h1"
USE_GC_DISABLE = True

# ==== Khu 1 === Xay board, state noi bo, successors_fast va rebuild.
DIRS = ((0, -1), (1, 0), (0, 1), (-1, 0))     # up, right, down, left
NAMES = ("up", "right", "down", "left")

class Board:
    """Bảng tiền xử lý kích thước, láng giềng, đích và action."""

    __slots__=('W', 'H', 'N', 'nbr', 'goals', 'goal_mask', 'act',
                'dead', 'pd_min', 'pdc', 'h_cache')


def build_board(state):
    """Tiền xử lý ô kề và push-distance; O(WH+T+G log G+GWH+R)."""
    W, H = state.width, state.height
    #Đánh số ô thành số nguyên: p = y * W + x, với (x, y) là tọa độ ô
    walls = {y * W + x for (x, y) in state.obstacles}
    b = Board()
    b.W, b.H, b.N = W, H, W * H
    #nbr[p][d] là ô kề của p theo hướng d, hoặc -1 nếu là tường
    nbr = []
    for y in range(H):
        for x in range(W):
            row = []
            for dx, dy in DIRS:
                nx, ny = x + dx, y + dy
                p = ny * W + nx
                #Hợp lệ nếu ô kề nằm trong board và không phải tường
                ok = 0 <= nx < W and 0 <= ny < H and p not in walls
                row.append(p if ok else -1)
            nbr.append(tuple(row))
    b.nbr = tuple(nbr)
    #Kiểm tra các ô đích và lưu vào goals, goal_mask
    b.goals = tuple(sorted(y * W + x for (x, y) in state.storage))
    b.goal_mask = sum(1 << g for g in b.goals)
    #Bảng act[i] = (0 up, 1 right, 2 down, 3 left) cho robot i
    b.act = tuple(tuple('%d %s' % (i, n) for n in NAMES)
                  for i in range(len(state.robots)))
    b.h_cache = {}
    precompute_push_dist(b)
    return b


def precompute_push_dist(board):
    """BFS ngược từ từng đích để tính số lần đẩy tối thiểu đến ô đó.

    Khi xét ô hiện tại c và hướng đẩy d, ô đứng trước đó là p = c - d.
    Push ngược chỉ hợp lệ nếu cả p và ô robot cần đứng p - d đều là sàn.
    Robots/boxes ban đầu không được xem là tường cố định; robot có thể đi bộ
    tới ô hỗ trợ trong quá trình giải.

    Lưu pd[j][c] trong board.pdc, min_j pd[j][c] trong board.pd_min và
    các ô không thể đẩy tới bất kỳ đích nào trong board.dead.
    Độ phức tạp O(WH + GWH) thời gian, O(WH + GWH) bộ nhớ.
    """
    width, height = board.W, board.H
    board.pdc = []

    for goal in board.goals:
        distances = [INF] * board.N
        # Một đích có thể ở góc/cụt nên không cần có ô kề để là sàn hợp lệ.
        if not 0 <= goal < board.N:
            board.pdc.append(tuple(distances))
            continue

        distances[goal] = 0
        queue = deque([goal])
        while queue:
            current = queue.popleft()
            for direction, (dx, dy) in enumerate(DIRS):
                # Đi ngược hướng đẩy: tìm vị trí cũ của thùng và chỗ robot đứng.
                previous_x = current % width - dx
                previous_y = current // width - dy
                if not (0 <= previous_x < width and 0 <= previous_y < height):
                    continue
                previous = previous_y * width + previous_x
                support_x = previous_x - dx
                support_y = previous_y - dy
                if not (0 <= support_x < width and 0 <= support_y < height):
                    continue
                support = support_y * width + support_x

                # Hai ô đều phải thông sàn: ô thùng trước đó và ô kê robot.
                if (board.nbr[current][(direction + 2) % 4] != previous
                        or board.nbr[previous][(direction + 2) % 4] != support):
                    continue
                if distances[previous] != INF:
                    continue
                distances[previous] = distances[current] + 1
                queue.append(previous)
        board.pdc.append(tuple(distances))

    if board.pdc:
        board.pd_min = tuple(
            min(distances[cell] for distances in board.pdc)
            for cell in range(board.N)
        )
    else:
        board.pd_min = tuple(INF for _ in range(board.N))

    board.dead = frozenset(
        cell for cell, distance in enumerate(board.pd_min) if distance == INF
    )
    return board.pdc, board.pd_min, board.dead


def h_zero(state):
    """Heuristic 0 của UCS/A*; O(1)."""
    return 0


def h_box_manhattan(state):
    """Tổng Manhattan tới đích gần nhất cho mỗi hộp; O(BG)."""
    goals = set(state.storage)
    if not goals:
        return 0 if not state.boxes else INF
    return sum(
        min(abs(bx - gx) + abs(by - gy) for gx, gy in goals)
        for bx, by in state.boxes
    )


def h1(state, board=None):
    """Tổng khoảng cách đẩy riêng từng hộp đến đích gần nhất; O(B).

    Trong mọi lời giải, mỗi hộp kết thúc ở một đích và phải được đẩy từ vị trí
    đầu tới đó. Số đẩy thực tế của hộp không nhỏ hơn khoảng cách đẩy tĩnh tới
    đích gần nhất; cộng theo hộp vẫn là cận dưới vì mỗi bước đẩy chỉ di chuyển
    đúng một hộp. Số bước đi bộ chỉ làm tổng chi phí thật lớn hơn. Nếu một hộp
    ở ô dead, không tồn tại lời giải nên trả INF. Khi tìm kiếm, truyền board
    đã tiền xử lý để tránh tính lại BFS.
    """
    if board is None:
        board = build_board(state)
    total = 0
    for box_x, box_y in state.boxes:
        if not (0 <= box_x < board.W and 0 <= box_y < board.H):
            return INF
        cell = box_y * board.W + box_x
        distance = board.pd_min[cell]
        if distance == INF:
            return INF
        total += distance
    return total


HEURISTICS = {
    "h0": h_zero,
    "manhattan": h_box_manhattan,
    "h1": h1,
}

def is_dead_none(state):
    """Không cắt state nào khi chưa có deadlock detector; O(1)."""
    return False


def astar(initial_state, deadline, heuristic, is_dead=is_dead_none,
          max_states=MAX_STATES):
    """Tìm đường bằng A*; O(E log E) thời gian, O(E) bộ nhớ trong trường hợp xấu."""
    stats = {"expanded": 0, "generated": 0, "stale": 0,
             "pruned_h": 0, "pruned_dead": 0, "reason": "exhausted"}
    h0 = heuristic(initial_state)
    if h0 >= INF:
        stats["reason"] = "h_inf_at_start"
        return None, stats

    tie = itertools.count()
    start_key = initial_state.hashable_state()
    best_g = {start_key: 0}
    heap = [(h0, 0, next(tie), initial_state)]
    pops = 0

    while heap:
        pops += 1
        if pops % CHECK_EVERY == 0 and time.monotonic() >= deadline:
            stats["reason"] = "timeout"
            return None, stats

        _, neg_g, _, state = heapq.heappop(heap)
        g = -neg_g
        key = state.hashable_state()
        if g > best_g[key]:
            stats["stale"] += 1
            continue
        if sokoban_goal_state(state):
            stats["reason"] = "goal"
            return state, stats

        stats["expanded"] += 1
        for child in state.successors():
            child_key = child.hashable_state()
            child_g = g + 1
            old_g = best_g.get(child_key)
            if old_g is not None and child_g >= old_g:
                continue
            if is_dead(child):
                stats["pruned_dead"] += 1
                continue
            child_h = heuristic(child)
            if child_h >= INF:
                stats["pruned_h"] += 1
                continue
            best_g[child_key] = child_g
            heapq.heappush(
                heap, (child_g + child_h, -child_g, next(tie), child))
            stats["generated"] += 1

        if len(best_g) > max_states:
            stats["reason"] = "memory_cap"
            return None, stats

    return None, stats


def bfs_v0(initial_state, deadline, max_states=MAX_STATES):
    """BFS dự phòng để so sánh; O(V·R(R+B)) thời gian, O(V(R+B)) bộ nhớ."""
    if sokoban_goal_state(initial_state):
        return initial_state
    queue = deque([initial_state])
    visited = {initial_state.hashable_state()}
    expanded_nodes = 0
    while queue:
        if (expanded_nodes % CHECK_EVERY) == 0 and time.monotonic() >= deadline:
            return None
        if len(visited) > max_states:
            return None
        expanded_nodes += 1
        current = queue.popleft()
        for child in current.successors():
            key = child.hashable_state()
            if key in visited:
                continue
            if sokoban_goal_state(child):
                return child
            visited.add(key)
            queue.append(child)
    return None


def solve(initial_state, timebound=120):
    """Giải bằng A*; thời gian O(E log E), bộ nhớ O(E) trong trường hợp xấu."""
    #Trả về False ngay nếu hết thời gian mà chưa tìm đc lời giải
    if initial_state is None or timebound is None or timebound <= 0:
        return False

    #Thiết lập thời gian kết thúc (đang để thời gian chạy là 85% của timebound)
    deadline = time.monotonic() + timebound * TIME_FRACTION
    if HEURISTIC == "h1":
        board = build_board(initial_state)
        heuristic = lambda state: h1(state, board)
    else:
        heuristic = HEURISTICS.get(HEURISTIC, h_zero)
    gc_was_enabled = gc.isenabled()
    if USE_GC_DISABLE:
        gc.disable()
    try:
        goal, _ = astar(initial_state, deadline, heuristic, is_dead_none)
    except MemoryError:
        goal = None
    finally:
        if gc_was_enabled:
            gc.enable()

    return goal if goal is not None else False
