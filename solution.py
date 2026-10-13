# TODO (KHU 0 - chu A): Khai bao import, hang so va co bat/tat.
# TODO (KHU 1 - chu B): Xay board, state noi bo, successors_fast va rebuild.
# TODO (KHU 2 - chu C): Cai dat heuristic admissible.
# TODO (KHU 3 - chu D): Cai dat phat hien deadlock sound.
# TODO (KHU 4 - chu A): Cai dat tim kiem va quan ly ngan sach thoi gian.
# TODO (KHU 5 - chu A): Cai dat solve(initial_state, timebound=120).


from collections import deque
import time
from sokoban import sokoban_goal_state

#Tỉ lệ thời gian đc dùng
TIME_FRACTION = 0.85
#Cứ mỗi 500 node được mở ra thì kiểm tra đồng hồ
CHECK_EVERY = 500
#Số state tối đa được lưu trong visited, tránh tràn bộ nhớ
MAX_STATES = 2000000 

# ==== Khu 1 === Xay board, state noi bo, successors_fast va rebuild.
DIRS = ((0, -1), (1, 0), (0, 1), (-1, 0))     # up, right, down, left
NAMES = ("up", "right", "down", "left")

class Board:
    __slots__=('W', 'H', 'N', 'nbr', 'goals', 'goal_mask', 'act',
                'dead', 'pd_min', 'pdc', 'h_cache')


def build_board(state):
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
    #precompute_push_dist(b)                #Khu 2 điền pd_min, pdc, dead
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

    #Trả về False ngay nếu hết thời gian mà chưa tìm đc lời giải
    if timebound <= 0:
        return False

    #Thiết lập thời gian kết thúc (đang để thời gian chạy là 85% của timebound)
    deadline = time.monotonic() + timebound * TIME_FRACTION
    #Kiểm tra state ban đầu có phải là đích không, nếu đúng thì trả về luôn
    if sokoban_goal_state(initial_state):
        return initial_state
    #queue là hàng đợi FIFO, visited là tập các state đã mở ra
    queue = deque([initial_state])
    visited = {initial_state.hashable_state()}
    expanded_nodes = 0                          #Số node đã mở ra, dùng để kiểm tra đồng hồ sau mỗi CHECK_EVERY node

    while queue:

        # Kiểm tra đồng hồ sau mỗi CHECK_EVERY node được mở ra
        if (expanded_nodes % CHECK_EVERY) == 0 and time.monotonic() >= deadline:
            return False

        #Dừng lại khi tràn bộ nhớ
        if len(visited) > MAX_STATES:
            return False
        
        expanded_nodes += 1
        current = queue.popleft()               #Lấy node đầu tiên ra khỏi queue

        #Sinh các node con từ node hiện tại
        for child in current.successors():
            key = child.hashable_state()
            if key in visited:                  #Nếu state đã được mở ra trước đó thì bỏ qua
                continue
            
            #Kiểm tra đích ngay lúc sinh node
            if sokoban_goal_state(child):
                return child
            visited.add(key)
            queue.append(child)
    #Nếu hết queue mà chưa tìm được lời giải thì trả về False
    return False