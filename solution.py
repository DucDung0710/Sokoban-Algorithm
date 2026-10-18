# TODO (KHU 0 - chu A): Khai bao import, hang so va co bat/tat.
# TODO (KHU 1 - chu B): Xay board, state noi bo, successors_fast va rebuild.
# PHAN C (vai tro C): Heuristic admissible va phat hien deadlock tinh.
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
HEURISTIC = "h2"
USE_GC_DISABLE = True
#Dãy trọng số cho anytime: w lớn tìm lời giải nhanh, w = 1 chứng minh tối ưu
WEIGHTS = (5.0, 3.0, 2.0, 1.5, 1.0)

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


def start_key(state):
    """Chuyển SokobanState sang state nội bộ (robots, boxes); O(R + B)."""
    W = state.width
    robots = tuple(y * W + x for (x, y) in state.robots)
    boxes = 0
    for (x, y) in state.boxes:
        boxes |= 1 << (y * W + x)
    return (robots, boxes)
 
 
def is_goal(b, s):
    """True khi không có thùng nào nằm ngoài ô đích; O(1).
 
    boxes & ~goal_mask giữ lại các thùng không ở đích: bằng 0 nghĩa là thắng.
    Số ô đích có thể nhiều hơn số thùng nên không so sánh bằng.
    """
    return (s[1] & ~b.goal_mask) == 0
 
 
def cells(mask):
    """Duyệt chỉ số các bit bằng 1 của mask, từ thấp lên cao; O(số bit)."""
    while mask:
        low = mask & -mask              # bit 1 thấp nhất
        yield low.bit_length() - 1      # chỉ số của bit đó
        mask ^= low                     # bỏ bit đó
 
 
def successors_fast(b, s):
    """Sinh các nước đi từ state nội bộ s, không tạo SokobanState.
 
    Trả list (action, s2, pushed): action là tên đúng như framework (vd "0 up"),
    s2 là state nội bộ mới, pushed là ô thùng vừa được đẩy tới (hoặc None).
    Tập (action, s2) phải khớp tuyệt đối với successors() của framework.
    """
    robots, boxes = s
    nbr = b.nbr
    act = b.act
    out = []
    for i, r in enumerate(robots):
        row = nbr[r]
        for d in range(4):
            t = row[d]                          # ô robot sắp bước tới
            if t < 0 or t in robots:            # tường, ngoài bàn, hoặc robot khác
                continue
            if (boxes >> t) & 1:                # phía trước có thùng: thử đẩy
                u = nbr[t][d]                   # ô phía sau thùng
                if u < 0 or (boxes >> u) & 1 or u in robots:
                    continue                    # không đẩy được
                nb = boxes ^ (1 << t) ^ (1 << u)    # thùng rời t, đến u (hai phép XOR)
                pushed = u
            else:                               # đi bộ
                nb = boxes
                pushed = None
            out.append((act[i][d], (robots[:i] + (t,) + robots[i + 1:], nb), pushed))
    return out
 
 
def rebuild(initial_state, actions):
    """Dựng lại chuỗi SokobanState thật từ danh sách action, bằng successors() của framework.
 
    Mỗi state con do framework tạo nên có đúng parent, action, gval và validation.py
    chấp nhận. initial_state chỉ được đọc, không bị sửa. Chi phí O(L * 4k).
    """
    cur = initial_state
    for a in actions:
        for c in cur.successors():
            if c.action == a:
                cur = c
                break
        else:
            raise RuntimeError('action khong hop le: ' + a)
    return cur


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


def _minimum_assignment_cost(costs):
    """Chi phí ghép nhỏ nhất cho mọi hàng vào các cột khác nhau; O(R²G)."""
    row_count = len(costs)
    if row_count == 0:
        return 0
    goal_count = len(costs[0])
    if row_count > goal_count:
        return INF

    # Thuật toán Hungarian hình chữ nhật; INF biểu diễn cặp không thể ghép.
    row_potential = [0] * (row_count + 1)
    goal_potential = [0] * (goal_count + 1)
    matched_row = [0] * (goal_count + 1)
    previous_goal = [0] * (goal_count + 1)

    for row in range(1, row_count + 1):
        matched_row[0] = row
        current_goal = 0
        min_reduced_cost = [INF] * (goal_count + 1)
        used_goal = [False] * (goal_count + 1)

        while True:
            used_goal[current_goal] = True
            current_row = matched_row[current_goal]
            delta = INF
            next_goal = 0
            for goal in range(1, goal_count + 1):
                if used_goal[goal]:
                    continue
                reduced_cost = (
                    costs[current_row - 1][goal - 1]
                    - row_potential[current_row]
                    - goal_potential[goal]
                )
                if reduced_cost < min_reduced_cost[goal]:
                    min_reduced_cost[goal] = reduced_cost
                    previous_goal[goal] = current_goal
                if min_reduced_cost[goal] < delta:
                    delta = min_reduced_cost[goal]
                    next_goal = goal

            if delta == INF:
                return INF
            for goal in range(goal_count + 1):
                if used_goal[goal]:
                    row_potential[matched_row[goal]] += delta
                    goal_potential[goal] -= delta
                else:
                    min_reduced_cost[goal] -= delta
            current_goal = next_goal
            if matched_row[current_goal] == 0:
                break

        while True:
            next_goal = previous_goal[current_goal]
            matched_row[current_goal] = matched_row[next_goal]
            current_goal = next_goal
            if current_goal == 0:
                break

    return -goal_potential[0]


def h2(state, board=None):
    """Ghép mỗi hộp với một đích khác nhau theo push-distance; O(B²G)."""
    if board is None:
        board = build_board(state)
    boxes = tuple(state.boxes)
    if not boxes:
        return 0
    if len(boxes) > len(board.goals):
        return INF

    costs = []
    for box_x, box_y in boxes:
        if not (0 <= box_x < board.W and 0 <= box_y < board.H):
            return INF
        cell = box_y * board.W + box_x
        costs.append(tuple(distances[cell] for distances in board.pdc))
    return _minimum_assignment_cost(costs)


def h3(state, board=None):
    """h2 cộng cận dưới bước đi trước lần đẩy đầu; O(N+B²G)."""
    if board is None:
        board = build_board(state)
    push_bound = h2(state, board)
    if push_bound == INF or push_bound == 0:
        return push_bound

    # Khoảng cách đi bộ chỉ xét tường; bỏ hộp để vẫn là cận dưới.
    walk_distance = [INF] * board.N
    queue = deque()
    for robot_x, robot_y in state.robots:
        if 0 <= robot_x < board.W and 0 <= robot_y < board.H:
            cell = robot_y * board.W + robot_x
            if walk_distance[cell] == INF:
                walk_distance[cell] = 0
                queue.append(cell)
    while queue:
        current = queue.popleft()
        for neighbor in board.nbr[current]:
            if neighbor >= 0 and walk_distance[neighbor] == INF:
                walk_distance[neighbor] = walk_distance[current] + 1
                queue.append(neighbor)

    # Bất kỳ lời giải chưa xong nào cũng cần một lần đẩy đầu tiên.
    # Mọi lời giải phải đưa robot tới một ô hỗ trợ trước lần đẩy đó.
    first_push_walk = INF
    for box_x, box_y in state.boxes:
        if not (0 <= box_x < board.W and 0 <= box_y < board.H):
            return INF
        cell = box_y * board.W + box_x
        for direction in range(4):
            destination = board.nbr[cell][direction]
            support = board.nbr[cell][(direction + 2) % 4]
            if destination >= 0 and support >= 0:
                first_push_walk = min(
                    first_push_walk, walk_distance[support])
    if first_push_walk == INF:
        return INF
    return push_bound + first_push_walk


HEURISTICS = {
    "h0": h_zero,
    "manhattan": h_box_manhattan,
    "h1": h1,
    "h2": h2,
    "h3": h3,
}


def is_dead_static(state, board):
    """True nếu hộp nằm ngoài bàn hoặc trên ô không thể đẩy tới đích; O(B)."""
    for box_x, box_y in state.boxes:
        if not (0 <= box_x < board.W and 0 <= box_y < board.H):
            return True
        if box_y * board.W + box_x in board.dead:
            return True
    return False


def has_frozen_box_deadlock(state, board):
    """True nếu có hộp bị khóa bởi tường/hộp đã khóa mà không ở đích; O(B²)."""
    boxes = set()
    for box_x, box_y in state.boxes:
        if not (0 <= box_x < board.W and 0 <= box_y < board.H):
            return True
        boxes.add(box_y * board.W + box_x)

    frozen = set()
    while True:
        newly_frozen = set()
        for cell in boxes - frozen:
            horizontal_blocked = (
                board.nbr[cell][1] < 0 or board.nbr[cell][3] < 0
                or board.nbr[cell][1] in frozen
                or board.nbr[cell][3] in frozen
            )
            vertical_blocked = (
                board.nbr[cell][0] < 0 or board.nbr[cell][2] < 0
                or board.nbr[cell][0] in frozen
                or board.nbr[cell][2] in frozen
            )
            if horizontal_blocked and vertical_blocked:
                newly_frozen.add(cell)
        if not newly_frozen:
            break
        frozen.update(newly_frozen)

    goals = set(board.goals)
    return any(cell not in goals for cell in frozen)


def is_dead(state, board):
    """Kết hợp ô dead tĩnh và freeze deadlock đã chứng minh được; O(B²)."""
    return (is_dead_static(state, board)
            or has_frozen_box_deadlock(state, board))


def is_dead_none(state):
    """Deadlock checker mặc định không cắt state nào; O(1)."""
    return False


def astar(initial_state, deadline, heuristic, is_dead=is_dead_none,
          max_states=MAX_STATES, weight=1.0, bound=INF):
    """A* có trọng số và cận cắt; O(E log E) thời gian, O(E) bộ nhớ.

    - Heap xếp theo f = g + weight * h. weight = 1 là A* thường (tối ưu khi h
      admissible); weight > 1 tìm nhanh hơn, chi phí <= weight * tối ưu.
    - bound: chi phí lời giải tốt nhất đã có. Node có g + h >= bound bị cắt vì
      với h admissible, mọi lời giải đi qua nó đều dài >= bound.
    - Kiểm đích lúc lấy ra; best_g + bỏ entry cũ (lazy deletion), cho phép mở lại.
    - stats["reason"] == "exhausted" khi có bound nghĩa là: không tồn tại lời giải
      ngắn hơn bound (nếu h admissible và is_dead sound).
    """
    stats = {"expanded": 0, "generated": 0, "stale": 0,
             "pruned_h": 0, "pruned_dead": 0, "pruned_bound": 0,
             "reason": "exhausted"}
    if is_dead(initial_state):
        stats["reason"] = "deadlock_at_start"
        return None, stats
    h0 = heuristic(initial_state)
    if h0 >= INF:
        stats["reason"] = "h_inf_at_start"
        return None, stats
    if h0 >= bound:
        stats["pruned_bound"] += 1
        return None, stats

    tie = itertools.count()
    start_key = initial_state.hashable_state()
    best_g = {start_key: 0}
    heap = [(weight * h0, 0, next(tie), initial_state)]
    push, pop = heapq.heappush, heapq.heappop
    pops = 0

    while heap:
        pops += 1
        if pops % CHECK_EVERY == 0 and time.monotonic() >= deadline:
            stats["reason"] = "timeout"
            return None, stats

        _, neg_g, _, state = pop(heap)
        g = -neg_g
        key = state.hashable_state()
        if g > best_g[key]:
            stats["stale"] += 1
            continue
        if sokoban_goal_state(state):
            stats["reason"] = "goal"
            return state, stats

        stats["expanded"] += 1
        child_g = g + 1
        for child in state.successors():
            child_key = child.hashable_state()
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
            if child_g + child_h >= bound:
                stats["pruned_bound"] += 1
                continue
            best_g[child_key] = child_g
            push(heap, (child_g + weight * child_h, -child_g, next(tie), child))
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


def anytime_search(initial_state, deadline, heuristic, deadlock,
                   weights=WEIGHTS, max_states=MAX_STATES):
    """Chạy weighted A* với w giảm dần, giữ lời giải tốt nhất; O(k·E log E).

    Mỗi pha dùng chi phí tốt nhất hiện có làm bound. Dừng sớm khi:
      - một pha chạy cạn heap (reason "exhausted"): không còn lời giải ngắn hơn,
        lời giải đang giữ (nếu có) là tối ưu;
      - pha w = 1 tìm được đích: đó là lời giải tối ưu;
      - hết giờ hoặc chạm trần bộ nhớ.
    Trả về (state đích tốt nhất hoặc None, danh sách thống kê từng pha, đã chứng
    minh tối ưu hay chưa).
    """
    best = None
    bound = INF
    phases = []
    proved = False
    for weight in weights:
        if time.monotonic() >= deadline:
            break
        try:
            goal, stats = astar(initial_state, deadline, heuristic, deadlock,
                                max_states=max_states, weight=weight, bound=bound)
        except MemoryError:
            # Hết RAM giữa chừng: giữ lời giải của các pha trước.
            break
        stats["weight"] = weight
        stats["cost"] = goal.gval if goal is not None else None
        phases.append(stats)
        if goal is not None:
            best = goal
            bound = goal.gval
            if weight <= 1.0:
                proved = True
                break
        elif stats["reason"] == "exhausted":
            # Không còn lời giải ngắn hơn bound: best (nếu có) là tối ưu,
            # không có best nghĩa là map không có lời giải.
            proved = True
            break
        elif stats["reason"] in ("timeout", "memory_cap"):
            break
        elif stats["reason"] in ("deadlock_at_start", "h_inf_at_start"):
            proved = True
            break
    return best, phases, proved

def solve(initial_state, timebound=120):
    """Giải bằng A*, h1 và deadlock ô tĩnh; O(E log E) thời gian, O(E) bộ nhớ."""
    #Trả về False ngay nếu hết thời gian mà chưa tìm đc lời giải
    if initial_state is None or timebound is None or timebound <= 0:
        return False

    #Thiết lập thời gian kết thúc (đang để thời gian chạy là 85% của timebound)
    deadline = time.monotonic() + timebound * TIME_FRACTION
    board = build_board(initial_state)
    if HEURISTIC in ("h1", "h2", "h3"):
        heuristic = lambda state: HEURISTICS[HEURISTIC](state, board)
    else:
        heuristic = HEURISTICS.get(HEURISTIC, h_zero)
    deadlock = lambda state: is_dead(state, board)
    gc_was_enabled = gc.isenabled()
    if USE_GC_DISABLE:
        gc.disable()
    try:
        goal, _ = astar(initial_state, deadline, heuristic, deadlock)
    except MemoryError:
        goal = None
    finally:
        if gc_was_enabled:
            gc.enable()

    return goal if goal is not None else False
