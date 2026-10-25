# TODO (KHU 0 - chu A): Khai bao import, hang so va co bat/tat.
# TODO (KHU 1 - chu B): Xay board, state noi bo, successors_fast va rebuild.
# TODO (KHU 2 - chu C): Cai dat heuristic admissible.
# TODO (KHU 3 - chu D): Cai dat phat hien deadlock sound.
# TODO (KHU 4 - chu A): Cai dat tim kiem va quan ly ngan sach thoi gian.
# TODO (KHU 5 - chu A): Cai dat solve(initial_state, timebound=120).



# ==== Khu 0 === (A) Import, hằng số, cờ bật/tắt.
from collections import deque
import gc
import heapq
import itertools
import time
from sokoban import sokoban_goal_state

TIME_FRACTION = 0.85     # chỉ dùng 85% timebound: bộ chấm còn tốn thời gian khởi động + kiểm lời giải
CHECK_EVERY = 500        # xem đồng hồ sau mỗi CHECK_EVERY lần lấy node ra
MAX_STATES = 2000000     # trần số state lưu trong best_g; chạm trần thì dừng thay vì bị OS giết
INF = float('inf')       # h trả INF nghĩa là state chắc chắn không giải được -> cắt
HEURISTIC = 'manhattan'  # 'h0' (kiểm phần tìm kiếm) | 'manhattan' (tạm, chờ h1 của C)
USE_GC_DISABLE = True    # tắt gc trong lúc tìm kiếm, luôn bật lại trong finally

# ==== Khu 1 === Xay board, state noi bo, successors_fast va rebuild.
DIRS = ((0, -1), (1, 0), (0, 1), (-1, 0))     # up, right, down, left
NAMES = ("up", "right", "down", "left")

class Board:
    __slots__=('W', 'H', 'N', 'nbr', 'goals', 'goal_mask', 'act',
                'dead', 'pd_min', 'pdc', 'h_cache')


def build_board(state):
    W, H = state.width, state.height
    walls = {y * W + x for (x, y) in state.obstacles}
    b = Board()
    b.W, b.H, b.N = W, H, W * H
    nbr = []
    for y in range(H):
        for x in range(W):
            row = []
            for dx, dy in DIRS:
                nx, ny = x + dx, y + dy
                p = ny * W + nx
                ok = 0 <= nx < W and 0 <= ny < H and p not in walls
                row.append(p if ok else -1)
            nbr.append(tuple(row))
    b.nbr = tuple(nbr)
    b.goals = tuple(sorted(y * W + x for (x, y) in state.storage))
    b.goal_mask = sum(1 << g for g in b.goals)
    b.act = tuple(tuple('%d %s' % (i, n) for n in NAMES)
                  for i in range(len(state.robots)))
    b.h_cache = {}
    #precompute_push_dist(b)                 #Khu 2 điền pd_min, pdc, dead
    return b


# ==== Khu 2 === (C) Heuristic admissible.
# Chưa có. Khi C xong h1, đăng ký vào bảng HEURISTICS ở Khu 4 và đổi cờ HEURISTIC ở Khu 0.


# ==== Khu 3 === (D) Phát hiện deadlock sound.
# Chưa có. Khi D xong, thay is_dead_none bằng hàm của D trong solve() (Khu 5).


# ==== Khu 4 === (A) Tìm kiếm và quản lý ngân sách thời gian.
#
# Hợp đồng tạm thời (tới khi B có successors_fast + rebuild):
#   - Tìm kiếm chạy trên SokobanState của framework, dùng state.successors().
#   - h(state)       -> số nguyên >= 0, hoặc INF nếu state chắc chắn không giải được.
#   - is_dead(state) -> True chỉ khi CHẮC CHẮN không giải được (phải sound).
# Khi B giao biểu diễn nhanh, chỉ đổi phần sinh node trong astar(); vòng lặp giữ nguyên.

def h_zero(state):
    """h0 = 0. A* với h0 chính là tìm kiếm chi phí đều: dùng để kiểm phần tìm kiếm."""
    return 0


def h_box_manhattan(state):
    """Tổng, với mỗi thùng, khoảng cách Manhattan tới ô đích gần nhất.

    Tạm thời cho A tự kiểm, sẽ thay bằng h1 của C.
    Admissible: mỗi lần đẩy dời đúng một thùng đi một ô và tốn một bước, nên thùng
    cách đích gần nhất d ô cần ít nhất d bước đẩy riêng cho nó.
    Consistent: một bước làm h đổi nhiều nhất 1 (chỉ một thùng dời một ô).
    """
    storage = state.storage
    total = 0
    for bx, by in state.boxes:
        if (bx, by) in storage:
            continue
        total += min(abs(bx - sx) + abs(by - sy) for sx, sy in storage)
    return total


HEURISTICS = {
    'h0': h_zero,
    'manhattan': h_box_manhattan,
}


def is_dead_none(state):
    """Chưa có bộ phát hiện deadlock: không bao giờ cắt."""
    return False


def astar(initial_state, deadline, h, is_dead=is_dead_none, max_states=MAX_STATES):
    """A* graph search có mở lại node (best_g + lazy deletion).

    Trả về (goal_state hoặc None, stats).
      - Kiểm đích lúc LẤY RA khỏi heap, không phải lúc sinh: đây là điều kiện
        để lời giải tối ưu khi h admissible.
      - Heap không có decrease-key, nên khi tìm được đường rẻ hơn tới một state
        ta đẩy bản mới vào và bỏ qua bản cũ lúc lấy ra (g > best_g[key]).
      - Entry heap là (f, -g, tie, state): tie tăng dần để Python không bao giờ
        phải so sánh hai SokobanState; -g ưu tiên node sâu hơn khi f bằng nhau.
    """
    stats = {'expanded': 0, 'generated': 0, 'stale': 0, 'pruned_h': 0,
             'pruned_dead': 0, 'reason': 'exhausted'}

    h0 = h(initial_state)
    if h0 >= INF:
        stats['reason'] = 'h_inf_at_start'
        return None, stats

    tie = itertools.count()
    start_key = initial_state.hashable_state()
    best_g = {start_key: 0}
    heap = [(h0, 0, next(tie), initial_state)]
    push, pop = heapq.heappush, heapq.heappop
    pops = 0

    while heap:
        pops += 1
        if pops % CHECK_EVERY == 0 and time.monotonic() >= deadline:
            stats['reason'] = 'timeout'
            return None, stats

        f, neg_g, _, state = pop(heap)
        g = -neg_g
        key = state.hashable_state()
        if g > best_g[key]:          # bản cũ, đã có đường rẻ hơn
            stats['stale'] += 1
            continue

        if sokoban_goal_state(state):
            stats['reason'] = 'goal'
            return state, stats

        stats['expanded'] += 1
        g2 = g + 1
        for child in state.successors():
            k2 = child.hashable_state()
            old = best_g.get(k2)
            if old is not None and g2 >= old:
                continue
            if is_dead(child):
                stats['pruned_dead'] += 1
                continue
            h2 = h(child)
            if h2 >= INF:
                stats['pruned_h'] += 1
                continue
            best_g[k2] = g2
            push(heap, (g2 + h2, -g2, next(tie), child))
            stats['generated'] += 1

        if len(best_g) > max_states:
            stats['reason'] = 'memory_cap'
            return None, stats

    return None, stats


def bfs_v0(initial_state, deadline, max_states=MAX_STATES):
    """BFS cơ sở (v0 của nhóm). Giữ lại để đối chiếu số bước với A*.

    BFS được kiểm đích lúc sinh vì mọi cạnh tốn 1 và node được mở theo đúng
    thứ tự độ sâu; A* thì KHÔNG được làm vậy.
    """
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


# ==== Khu 5 === (A) solve().
def solve(initial_state, timebound=120):
    """
    Giải Sokoban bằng A*.

    Args:
        initial_state: Trạng thái Sokoban ban đầu (không bị sửa).
        timebound: Giới hạn thời gian chạy, tính bằng giây.

    Returns:
        Trạng thái đích của framework nếu tìm được (lần ngược parent/action/gval
        cho đường đi hợp lệ), hoặc False nếu hết thời gian/không có lời giải.
    """
    if timebound is None or timebound <= 0:
        return False

    deadline = time.monotonic() + timebound * TIME_FRACTION
    h = HEURISTICS.get(HEURISTIC, h_zero)

    gc_was_enabled = gc.isenabled()
    if USE_GC_DISABLE:
        gc.disable()
    try:
        goal, _ = astar(initial_state, deadline, h, is_dead_none)
    except MemoryError:
        goal = None
    finally:
        if gc_was_enabled:
            gc.enable()

    return goal if goal is not None else False
