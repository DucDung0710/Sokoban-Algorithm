"""Solver Sokoban nhiều robot cho môn Trí tuệ nhân tạo (UET).

Tổ chức file theo 6 Khu, mỗi Khu có một người phụ trách:
  Khu 0 (A): import, hằng số, cờ cấu hình CONFIG.
  Khu 1 (B): Board, state nội bộ (robots, bitmask thùng), successors_fast, rebuild.
  Khu 2 (C): heuristic admissible h0-h3 dựa trên khoảng cách đẩy.
  Khu 3 (D, C làm): phát hiện deadlock sound (ô chết, freeze).
  Khu 4 (A): tìm kiếm A* có trọng số, anytime, quản lý thời gian và bộ nhớ.
  Khu 5 (A): solve(initial_state, timebound=120), giao diện framework gọi.
"""

# ==== Khu 0 === (A) Import, hằng số, cờ cấu hình.
from collections import deque
import gc
import heapq
import itertools
import time
from sokoban import sokoban_goal_state

# Tỉ lệ thời gian được dùng trên timebound (phần còn lại để dọn bộ nhớ, trả kết quả)
TIME_FRACTION = 0.85
# Cứ mỗi CHECK_EVERY lần lấy node khỏi heap thì kiểm tra đồng hồ một lần
CHECK_EVERY = 500
# Số state tối đa được lưu trong best_g, tránh tràn bộ nhớ
MAX_STATES = 1000000
INF = float("inf")
USE_GC_DISABLE = True
CONFIG = {
    "h": "h2",               # h0 | h1 | h2 | h3
    "dead": "freeze",        # none | cells | freeze
    "h_cache_max": 100000,
    "sym": True,             # coi các robot như nhau khi ghép khóa (astar và engine fast)
}
# Thông tin lần solve() gần nhất, bench.py đọc: {"proved": bool, "phases": [...]}
LAST_INFO = {}
#Dãy trọng số cho anytime: w lớn tìm lời giải nhanh, w = 1 chứng minh tối ưu
WEIGHTS = (5.0, 3.0, 2.0, 1.5, 1.0)
# Pha đầu chưa ra lời giải sau tỉ lệ này của ngân sách thì chuyển sang pha cứu hộ
FIRST_PHASE_FRACTION = 0.5
# Trọng số pha cứu hộ, chỉ chạy khi pha đầu thất bại mà vẫn còn thời gian
RESCUE_WEIGHT = 25.0
# Khi một trọng số thất bại, chia đôi dần về phía trọng số đã thành công tới khi hẹp hơn mức này
MIN_WEIGHT_GAP = 0.1
# Số thùng tối đa trên một chuỗi đệ quy kiểm freeze; vượt quá thì coi là chưa đóng băng (vẫn sound)
FREEZE_DEPTH = 12

# ==== Khu 1 === (B) Xây board, state nội bộ, successors_fast và rebuild.
DIRS = ((0, -1), (1, 0), (0, 1), (-1, 0))     # up, right, down, left
NAMES = ("up", "right", "down", "left")

class Board:
    """Bảng tiền xử lý kích thước, láng giềng, đích và action."""

    __slots__=('W', 'H', 'N', 'nbr', 'goals', 'goal_mask', 'goal_set', 'act',
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
    b.goal_set = frozenset(b.goals)     # tạo một lần, kiểm freeze dùng lại
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


# ---- Các hàm dưới đây (start_key .. rebuild) thuộc engine "fast": solve() hiện
# ---- KHÔNG gọi tới, chỉ dùng trong test và thử nghiệm. Xem thêm Khu 4a.

def start_key(state):
    """Chuyển SokobanState sang state nội bộ (robots, boxes)"""
    W = state.width                                                 #Lấy chiều rộng của board
    robots = tuple(y * W + x for (x, y) in state.robots)            #Lấy vị trí ban đầu của robot
    boxes = 0
    for (x, y) in state.boxes:
        boxes |= 1 << (y * W + x)                                   #Tìm vị trí thùng và lưu vào boxes bằng cách sử dụng bitmask
        #Kiếm tra ô p có thùng hay k bằng cách (boxes >> p) & 1 (1 nếu có thùng, 0 nếu không)
    return (robots, boxes)                                          #Trả về state nội bộ gồm vị trí robot và bitmask thùng


def cells(mask):
    """Yield vị trí các bit được bật trong bitmask theo thứ tự tăng dần."""
    if mask < 0:
        raise ValueError("mask must be non-negative")
    while mask:
        lowest_bit = mask & -mask
        yield lowest_bit.bit_length() - 1
        mask ^= lowest_bit


def is_goal(b, s):
    """True khi không có thùng nào nằm ngoài ô đích
    Số ô đích có thể nhiều hơn số thùng nên không so sánh bằng.
    """
    return (s[1] & ~b.goal_mask) == 0                               #s[1] là boxes, b.goal_mask là bitmask của các ô đích
                                                                    #Nếu tất cả các thùng đều nằm trong các ô đích
                                                                    #thì boxes & ~goal_mask sẽ bằng 0
                                                                    #nghĩa là không có thùng nào nằm ngoài ô đích.


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
    for i, r in enumerate(robots):              #Duyệt qua từng robot, r là vị trí hiện tại của robot i
        row = nbr[r]                            #Lấy danh sách các ô kề của robot i
        for d in range(4):                      #Thử đi cả 4 hướng
            t = row[d]
            if t < 0 or t in robots:            #Nếu là tường, ngoài bàn, hoặc robot khác thì bỏ qua hướng này
                continue
            if (boxes >> t) & 1:                #Nếu phía trước có thùng thì thử đẩy
                u = nbr[t][d]                   #Ô phía sau thùng
                if u < 0 or (boxes >> u) & 1 or u in robots:            #Kiểm tra ô phía sau thùng có trống không
                    continue                    #Không đẩy được thì bỏ qua hướng này
                nb = boxes ^ (1 << t) ^ (1 << u)    #Cập nhật vị trí thùng: rời t, đến u
                pushed = u                      #Lưu lại ô thùng vừa được đẩy tới
            else:                               #Nếu phía trước không có thùng thì chỉ đi bộ
                nb = boxes
                pushed = None
            out.append((act[i][d]               #Lấy tên hành động theo robot i và hướng d
                        , (robots[:i] + (t,) + robots[i + 1:], nb)      #Cập nhật vị trí robot i sang t, giữ nguyên các robot khác
                        , pushed))
    return out                                  #Danh sách chứa các hành động hợp lệ


def rebuild(initial_state, actions):
    """Dựng lại chuỗi SokobanState thật từ danh sách action, bằng successors() của framework.

    Mỗi state con do framework tạo nên có đúng parent, action, gval và validation.py
    chấp nhận. initial_state chỉ được đọc, không bị sửa. Chi phí O(L * 4k).
    """
    cur = initial_state                         #Bắt đầu từ trạng thái ban đầu của Sokoban
    for a in actions:                           #Duyệt lần lượt từng hành động trong lời giải
        for c in cur.successors():              #Lấy danh sách các trạng thái kế tiếp có thể đi đến từ trạng thái hiện tại
            if c.action == a:                   #Tìm trạng thái có hành động trùng với hành động cần thực hiện
                cur = c
                break                           #Đã tìm được trạng thái phù hợp, thoát khỏi vòng lặp bên trong
        else:
            #Nếu duyệt hết successors() mà không tìm thấy hành động hợp lệ,
            #nghĩa là hành động trong danh sách actions không thể thực hiện
            raise RuntimeError('action khong hop le: ' + a)
    return cur                                  #Trả về trạng thái cuối cùng sau khi thực hiện tất cả hành động


def h_zero(state):
    """Heuristic 0 của UCS/A*; O(1)."""
    return 0


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


def _assignment_cost(costs, goal_count):
    """Chi phí ghép thùng-đích nhỏ nhất, dùng chung cho h2 và _h2_mask.

    Ít đích (<= 8): DP trên tập đích đã dùng, O(B·2^G·G).
    Nhiều đích hơn: Hungarian hình chữ nhật, O(B²G).
    """
    if goal_count > 8:
        return _minimum_assignment_cost(costs)
    assignments = {0: 0}
    for row in costs:
        next_assignments = {}
        for used_goals, cost in assignments.items():
            for goal, distance in enumerate(row):
                goal_bit = 1 << goal
                if used_goals & goal_bit or distance >= INF:
                    continue
                next_mask = used_goals | goal_bit
                candidate = cost + distance
                if candidate < next_assignments.get(next_mask, INF):
                    next_assignments[next_mask] = candidate
        assignments = next_assignments
        if not assignments:
            break
    return min(assignments.values(), default=INF)


def _cache_put(board, box_mask, value):
    """Lưu h theo bitmask thùng; cache đầy thì xóa hết rồi lưu lại từ đầu."""
    cache_limit = CONFIG["h_cache_max"]
    if cache_limit > 0:
        if len(board.h_cache) >= cache_limit:
            board.h_cache.clear()
        board.h_cache[box_mask] = value


def h2(state, board=None):
    """Ghép hộp với các đích khác nhau; cache theo cấu hình hộp."""
    if board is None:
        board = build_board(state)
    box_cells = []
    box_mask = 0
    for box_x, box_y in state.boxes:
        if not (0 <= box_x < board.W and 0 <= box_y < board.H):
            return INF
        cell = box_y * board.W + box_x
        box_cells.append(cell)
        box_mask |= 1 << cell

    if not box_cells:
        return 0
    if len(box_cells) > len(board.goals):
        return INF
    cached = board.h_cache.get(box_mask)
    if cached is not None:
        return cached

    costs = [
        tuple(distances[cell] for distances in board.pdc)
        for cell in box_cells
    ]
    result = _assignment_cost(costs, len(board.goals))
    _cache_put(board, box_mask, result)
    return result


def h3(state, board=None):
    """h2 cộng cận Manhattan đi bộ trước lần đẩy đầu tiên."""
    if board is None:
        board = build_board(state)
    push_bound = h2(state, board)
    if push_bound == INF or push_bound == 0:
        return push_bound
    if not state.robots:
        return INF

    minimum_distance = min(
        abs(robot_x - box_x) + abs(robot_y - box_y)
        for robot_x, robot_y in state.robots
        for box_x, box_y in state.boxes
    )
    return push_bound + max(0, minimum_distance - 1)


HEURISTICS = {
    "h0": h_zero,
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


def _blocked_axis(board, boxes, cell, horizontal, seen):
    if horizontal:
        sides = (board.nbr[cell][3], board.nbr[cell][1])
    else:
        sides = (board.nbr[cell][0], board.nbr[cell][2])
    first, second = sides
    if first < 0 or second < 0:
        return True
    if first in board.dead and second in board.dead:
        return True
    for neighbor in sides:
        if (boxes >> neighbor) & 1 and _is_frozen(
                board, boxes, neighbor, seen):
            return True
    return False


def _is_frozen(board, boxes, cell, seen):
    if cell in seen:
        return True
    if len(seen) > FREEZE_DEPTH:        # chuỗi quá dài: bỏ cuộc, coi là chưa đóng băng
        return False
    next_seen = seen | {cell}
    return (
        _blocked_axis(board, boxes, cell, True, next_seen)
        and _blocked_axis(board, boxes, cell, False, next_seen)
    )


def has_frozen_box_deadlock(state, board, candidates=None):
    """Kiểm tra freeze quanh các hộp ứng viên; toàn bàn nếu không chỉ định."""
    boxes = 0
    cell_to_box = {}
    for box_x, box_y in state.boxes:
        if not (0 <= box_x < board.W and 0 <= box_y < board.H):
            return True
        cell = box_y * board.W + box_x
        boxes |= 1 << cell
        cell_to_box[cell] = (box_x, box_y)

    if candidates is None:
        candidates = tuple(cell_to_box)
    goals = board.goal_set
    for cell in candidates:
        if cell not in cell_to_box:
            continue
        if (cell not in goals
                and _is_frozen(board, boxes, cell, frozenset())):
            return True
    return False


def is_dead(state, board, pushed=None):
    """Báo deadlock sound; sau push chỉ kiểm ô đích và các hộp kề."""
    mode = CONFIG["dead"]
    if mode == "none":
        return False
    if mode not in ("cells", "freeze"):
        raise ValueError("CONFIG['dead'] must be 'none', 'cells', or 'freeze'")

    if pushed is None and state.parent is not None:
        previous_boxes = state.parent.boxes
        added_boxes = state.boxes - previous_boxes
        pushed = next(iter(added_boxes), None)
        if pushed is not None:
            pushed = pushed[1] * board.W + pushed[0]
        elif state.boxes == previous_boxes:
            return False

    if pushed is None:
        if is_dead_static(state, board):
            return True
    elif pushed in board.dead:
        return True
    if mode == "cells":
        return False

    if pushed is None:
        candidates = tuple(
            y * board.W + x for x, y in state.boxes)
    else:
        box_cells = {y * board.W + x for x, y in state.boxes}
        candidates = (pushed,) + tuple(
            neighbor for neighbor in board.nbr[pushed]
            if neighbor >= 0 and neighbor in box_cells
        )
    return has_frozen_box_deadlock(state, board, candidates)

# ==== Khu 4 === (A) Tìm kiếm và quản lý thời gian, bộ nhớ.
# ---- Khu 4a: biểu diễn khóa số nguyên cho engine "fast" ----
# CHƯA DÙNG TRONG BÀI NỘP: solve() chạy engine framework (Khu 4b). Phần này giữ
# lại cho thử nghiệm và cho tests/test_search.py kiểm tương đương với Khu 2, 3.
# Khóa = bitmask thùng dịch trái R*S bit, ghép với ô của từng robot (mỗi robot S bit).
# h và deadlock ở đây phải cho đúng cùng giá trị với Khu 2, 3 (tests/test_search.py kiểm).

class FastCtx:
    """Bảng tính một lần cho mỗi map, dùng cho search trên khóa số nguyên."""

    __slots__ = ('b', 'R', 'S', 'RMASK', 'rows', 'xy')


def make_ctx(board, n_robots):
    """Tạo FastCtx từ Board đã tiền xử lý; O(N·G).

    S = số bit đủ chứa chỉ số ô lớn nhất; rows[p] = chi phí đẩy từ ô p tới
    từng đích (cột của board.pdc); xy[p] = tọa độ (x, y) của ô p.
    """
    c = FastCtx()
    c.b = board
    c.R = n_robots
    c.S = max(1, board.N.bit_length())
    c.RMASK = (1 << c.S) - 1
    c.rows = tuple(tuple(d[p] for d in board.pdc) for p in range(board.N))
    c.xy = tuple((p % board.W, p // board.W) for p in range(board.N))
    return c


def pack(c, robots, boxes):
    """Ghép (ô robot, bitmask thùng) thành một số nguyên; O(R log R).

    Khi CONFIG["sym"] bật, ô robot được sắp xếp trước khi ghép, nên hai state
    chỉ khác nhau ở chỗ robot nào đứng ô nào cho ra cùng một khóa.
    """
    key = boxes
    for r in (sorted(robots) if CONFIG["sym"] else robots):
        key = (key << c.S) | r
    return key


def unpack(c, key):
    """Tách khóa thành (list ô robot, bitmask thùng); O(R)."""
    robots = [0] * c.R
    for i in range(c.R - 1, -1, -1):
        robots[i] = key & c.RMASK
        key >>= c.S
    return robots, key


def _h2_mask(c, boxes):
    """h2 tính từ bitmask thùng, dùng chung cache board.h_cache với h2 của C."""
    b = c.b
    if not boxes:
        return 0
    cached = b.h_cache.get(boxes)       # tra cache trước khi dựng ma trận, giống h2 của C
    if cached is not None:
        return cached
    costs = [c.rows[p] for p in cells(boxes)]
    if len(costs) > len(b.goals):
        return INF
    result = _assignment_cost(costs, len(b.goals))
    _cache_put(b, boxes, result)
    return result


def fast_h(c, name):
    """Trả về hàm h(robots, boxes) cho engine fast, cùng giá trị với Khu 2.

    Hỗ trợ "h0", "h1", "h2", "h3". h3 lấy Manhattan nhỏ nhất giữa robot và
    MỌI thùng, kể cả thùng đã trên đích (giữ admissible, xem tài liệu nguyên lý).
    """
    pd_min, xy = c.b.pd_min, c.xy
    if name == "h0":
        return lambda robots, boxes: 0
    if name == "h1":
        def h1_fast(robots, boxes):
            total = 0
            for p in cells(boxes):
                d = pd_min[p]
                if d == INF:
                    return INF
                total += d
            return total
        return h1_fast
    if name == "h2":
        return lambda robots, boxes: _h2_mask(c, boxes)
    if name == "h3":
        def h3_fast(robots, boxes):
            push_bound = _h2_mask(c, boxes)
            if push_bound == INF or push_bound == 0:
                return push_bound
            if not robots:
                return INF
            box_xy = [xy[p] for p in cells(boxes)]
            best = INF
            for r in robots:
                rx, ry = xy[r]
                for bx, by in box_xy:
                    d = abs(rx - bx) + abs(ry - by)
                    if d < best:
                        best = d
            return push_bound + max(0, best - 1)
        return h3_fast
    raise ValueError("engine fast chỉ hỗ trợ h0, h1, h2, h3")


def dead_after_push(b, boxes, pushed, mode):
    """Deadlock sau khi một thùng vừa được đẩy tới ô pushed; cùng kết quả với is_dead.

    boxes là bitmask thùng SAU khi đẩy. mode: "none" | "cells" | "freeze".
    Chỉ xét thùng vừa đẩy và các thùng kề nó; thùng đóng băng trên đích không tính.
    """
    if mode == "none":
        return False
    if pushed in b.dead:
        return True
    if mode == "cells":
        return False
    goal_mask = b.goal_mask
    candidates = (pushed,) + tuple(
        n for n in b.nbr[pushed] if n >= 0 and (boxes >> n) & 1)
    for cell in candidates:
        if not (goal_mask >> cell) & 1 and _is_frozen(b, boxes, cell, frozenset()):
            return True
    return False


# ---- Khu 4b: search trên SokobanState của framework (đang dùng trong solve) ----

def astar(initial_state, deadline, heuristic, is_dead,
          max_states=None, weight=1.0, bound=INF, box_only_h=False):
    """A* có trọng số và cận cắt; O(E log E) thời gian, O(E) bộ nhớ.

    - Heap xếp theo f = g + weight * h. weight = 1 là A* thường (tối ưu khi h
      admissible); weight > 1 tìm nhanh hơn, chi phí <= weight * tối ưu.
    - bound: chi phí lời giải tốt nhất đã có. Node có g + h >= bound bị cắt vì
      với h admissible, mọi lời giải đi qua nó đều dài >= bound.
    - Kiểm đích lúc lấy ra; best_g + bỏ entry cũ (lazy deletion), cho phép mở lại.
    - stats["reason"] == "exhausted" khi có bound nghĩa là: không tồn tại lời giải
      ngắn hơn bound (nếu h admissible và is_dead sound).
    - CONFIG["sym"]: khóa trong best_g dùng ô robot đã sắp xếp, nên các state chỉ
      khác nhau ở việc robot nào đứng ô nào được gộp làm một. Node vẫn là
      SokobanState thật nên chuỗi parent/action trả về vẫn đúng.
    - box_only_h=True: heuristic chỉ phụ thuộc vị trí thùng (h0, h1, h2), nên nước
      đi bộ dùng lại h của cha thay vì tính lại.
    """
    if max_states is None:              # đọc lúc gọi để bench ghi đè được
        max_states = MAX_STATES
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
    sym = CONFIG["sym"]
    if sym:
        start = (tuple(sorted(initial_state.robots)), initial_state.boxes)
    else:
        start = initial_state.hashable_state()
    best_g = {start: 0}
    heap = [(weight * h0, 0, next(tie), initial_state, start, h0)]
    push, pop = heapq.heappush, heapq.heappop
    pops = 0

    while heap:
        pops += 1
        if pops % CHECK_EVERY == 0 and time.monotonic() >= deadline:
            stats["reason"] = "timeout"
            return None, stats

        _, neg_g, _, state, key, state_h = pop(heap)
        g = -neg_g
        if g > best_g[key]:
            stats["stale"] += 1
            continue
        if sokoban_goal_state(state):
            stats["reason"] = "goal"
            return state, stats

        stats["expanded"] += 1
        child_g = g + 1
        boxes = state.boxes
        for child in state.successors():
            if sym:
                child_key = (tuple(sorted(child.robots)), child.boxes)
            else:
                child_key = child.hashable_state()
            old_g = best_g.get(child_key)
            if old_g is not None and child_g >= old_g:
                continue
            if is_dead(child):
                stats["pruned_dead"] += 1
                continue
            if box_only_h and child.boxes == boxes:
                child_h = state_h               # đi bộ: thùng không đổi nên h không đổi
            else:
                child_h = heuristic(child)
            if child_h >= INF:
                stats["pruned_h"] += 1
                continue
            if child_g + child_h >= bound:
                stats["pruned_bound"] += 1
                continue
            best_g[child_key] = child_g
            push(heap, (child_g + weight * child_h, -child_g, next(tie), child,
                        child_key, child_h))
            stats["generated"] += 1

        if len(best_g) > max_states:
            stats["reason"] = "memory_cap"
            return None, stats

    return None, stats


def anytime_search(initial_state, deadline, heuristic, deadlock,
                   weights=None, max_states=None, box_only_h=False):
    """Chạy weighted A* với w giảm dần, giữ lời giải tốt nhất; O(k·E log E).

    Mỗi pha dùng chi phí tốt nhất hiện có làm bound. Dừng sớm khi:
      - một pha chạy cạn heap (reason "exhausted"): không còn lời giải ngắn hơn,
        lời giải đang giữ (nếu có) là tối ưu;
      - pha w = 1 tìm được đích: đó là lời giải tối ưu;
      - hết giờ, hoặc khoảng chia đôi trọng số đã hẹp hơn MIN_WEIGHT_GAP.
    Pha đầu chỉ được dùng FIRST_PHASE_FRACTION ngân sách; nếu nó thất bại mà chưa
    có lời giải thì chạy một pha cứu hộ với RESCUE_WEIGHT. Khi đã có lời giải mà
    một trọng số nhỏ hơn chạm trần bộ nhớ thì thử trọng số nằm giữa (chia đôi).
    Trả về (state đích tốt nhất hoặc None, danh sách thống kê từng pha, đã chứng
    minh tối ưu hay chưa).
    """
    if weights is None:                 # đọc lúc gọi để bench ghi đè được
        weights = WEIGHTS
    if max_states is None:
        max_states = MAX_STATES
    best = None
    bound = INF
    phases = []
    proved = False
    now = time.monotonic()
    first_limit = now + (deadline - now) * FIRST_PHASE_FRACTION
    queue = list(weights)
    rescue_used = False
    ok_weight = fail_weight = None      # trọng số nhỏ nhất đã thành công / lớn nhất đã thất bại
    while queue and time.monotonic() < deadline:
        weight = queue.pop(0)
        # Chỉ giới hạn riêng pha đầu, khi còn pha cứu hộ để chuyển sang
        limit = deadline if phases else first_limit
        try:
            goal, stats = astar(initial_state, limit, heuristic, deadlock,
                                max_states=max_states, weight=weight, bound=bound,
                                box_only_h=box_only_h)
        except MemoryError:
            # Hết RAM giữa chừng: coi như chạm trần bộ nhớ.
            goal, stats = None, {"reason": "memory_cap"}
        stats["weight"] = weight
        stats["cost"] = goal.gval if goal is not None else None
        phases.append(stats)
        if goal is not None:
            best = goal
            bound = goal.gval
            ok_weight = weight
            if weight <= 1.0:
                proved = True
                break
        elif stats["reason"] in ("exhausted", "deadlock_at_start", "h_inf_at_start"):
            # Không còn lời giải ngắn hơn bound: best (nếu có) là tối ưu,
            # không có best nghĩa là map không có lời giải.
            proved = True
            break
        elif best is None:
            if rescue_used:
                break                           # cả pha cứu hộ cũng thất bại
            rescue_used = True
            fail_weight = weight
            queue = [RESCUE_WEIGHT]             # pha đầu thất bại: thử trọng số lớn
            continue
        else:
            fail_weight = weight                # quá nặng với trần bộ nhớ / thời gian
        if fail_weight is not None:
            # Trọng số fail_weight không chạy nổi: chia đôi khoảng giữa nó và
            # trọng số nhỏ nhất đã thành công để tận dụng thời gian còn lại.
            if ok_weight - fail_weight > MIN_WEIGHT_GAP:
                queue = [(ok_weight + fail_weight) / 2.0]
            else:
                break
    return best, phases, proved


def solve(initial_state, timebound=120):
    """Tìm và cải thiện lời giải trong giới hạn thời gian, trả về state đích."""
    global LAST_INFO
    LAST_INFO = {"proved": False, "phases": []}
    # Đầu vào không hợp lệ hoặc không có thời gian thì trả False ngay
    if initial_state is None or timebound is None or timebound <= 0:
        return False

    # Thời điểm phải dừng search: TIME_FRACTION (85%) của timebound
    deadline = time.monotonic() + timebound * TIME_FRACTION
    board = build_board(initial_state)
    heuristic_name = CONFIG["h"]
    if heuristic_name not in HEURISTICS:
        raise ValueError("CONFIG['h'] must name a supported heuristic")
    if heuristic_name in ("h1", "h2", "h3"):
        heuristic = lambda state: HEURISTICS[heuristic_name](state, board)
    else:
        heuristic = HEURISTICS[heuristic_name]
    deadlock = lambda state: is_dead(state, board)
    gc_was_enabled = gc.isenabled()
    if USE_GC_DISABLE:
        gc.disable()
    try:
        goal, phases, proved = anytime_search(
            initial_state, deadline, heuristic, deadlock,
            box_only_h=heuristic_name != "h3")
        LAST_INFO = {"proved": proved, "phases": phases}
    except MemoryError:
        goal = None
    finally:
        if gc_was_enabled:
            gc.enable()

    return goal if goal is not None else False
