# TODO (KHU 0 - chu A): Khai bao import, hang so va co bat/tat.
# TODO (KHU 1 - chu B): Xay board, state noi bo, successors_fast va rebuild.
# TODO (KHU 2 - chu C): Cai dat heuristic admissible.
# TODO (KHU 3 - chu D): Cai dat phat hien deadlock sound.
# TODO (KHU 4 - chu A): Cai dat tim kiem va quan ly ngan sach thoi gian.
# TODO (KHU 5 - chu A): Cai dat solve(initial_state, timebound=120).


from collections import deque
import time
from sokoban import sokoban_goal_state
TIME_FRACTION = 0.85
CHECK_EVERY = 500
MAX_STATES = 2000000 

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









def solve(initial_state, timebound=120):
    """
    BFS cơ sở cho Sokoban.

    Args:
        initial_state: Trạng thái Sokoban ban đầu.
        timebound: Giới hạn thời gian chạy, tính bằng giây.

    Returns:
        Trạng thái đích của framework nếu tìm được,
        hoặc False nếu hết thời gian/không có lời giải.
    """

    if timebound <= 0:
        return False

    deadline = time.monotonic() + timebound * TIME_FRACTION
    if sokoban_goal_state(initial_state):
        return initial_state
    queue = deque([initial_state])
    visited = {initial_state.hashable_state()}
    expanded_nodes = 0

    while queue:

        # Kiểm tra đồng hồ sau mỗi CHECK_EVERY node được mở ra
        if (expanded_nodes % CHECK_EVERY) == 0 and time.monotonic() >= deadline:
            return False

        #Dừng lại khi tràn bộ nhớ
        if len(visited) > MAX_STATES:
            return False
        
        expanded_nodes += 1
        current = queue.popleft()

        for child in current.successors():
            key = child.hashable_state()
            if key in visited:
                continue
            
            #Kiểm tra đích ngay lúc sinh node
            if sokoban_goal_state(child):
                return child
            visited.add(key)
            queue.append(child)

    return False