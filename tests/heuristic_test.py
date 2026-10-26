
import solution as S
from sokoban import SokobanState
import unittest
from collections import deque


def build_graph(b, start, cap=3_000_000):
    """Xây dựng đồ thị trạng thái Sokoban bằng BFS."""
    pred = {start: []}
    edges = []
    q = deque([start])

    while q:
        s = q.popleft()

        # Sinh các trạng thái kế tiếp và lưu cạnh
        for _, s2, pushed in S.successors_fast(b, s):
            edges.append((s, s2, pushed))

            if s2 not in pred:
                # Giới hạn số trạng thái để tránh tràn bộ nhớ
                if len(pred) >= cap:
                    raise MemoryError(
                        "do thi qua lon, chon map nho hon"
                    )

                pred[s2] = []
                q.append(s2)

            # Lưu trạng thái tiền nhiệm để phục vụ BFS ngược
            pred[s2].append(s)

    return pred, edges


def true_dist(b, pred):
    """Tính khoảng cách tối ưu đến đích bằng BFS ngược."""
    dist = {s: 0 for s in pred if S.is_goal(b, s)}
    q = deque(dist)

    while q:
        s = q.popleft()

        # Duyệt ngược từ trạng thái đích về các tiền nhiệm
        for p in pred[s]:
            if p not in dist:
                dist[p] = dist[s] + 1
                q.append(p)

    return dist


def corridor_state():
    """Tạo bản đồ hành lang nhỏ để kiểm tra heuristic."""
    walls = frozenset(
        (x, y)
        for x in range(5)
        for y in (0, 2)
    )

    return SokobanState(
        "START",
        0,
        None,
        5,
        3,
        ((1, 1),),
        frozenset(((2, 1),)),
        frozenset(((4, 1),)),
        walls,
    )


def _make_state(template, board, internal_state):
    """Chuyển trạng thái nội bộ thành SokobanState."""
    robots, boxes = internal_state

    # Chuyển chỉ số ô một chiều sang tọa độ (x, y)
    robot_positions = tuple(
        (robot % board.W, robot // board.W)
        for robot in robots
    )

    box_positions = frozenset(
        (cell % board.W, cell // board.W)
        for cell in S.cells(boxes)
    )

    return SokobanState(
        "CHECK",
        0,
        None,
        template.width,
        template.height,
        robot_positions,
        box_positions,
        template.storage,
        template.obstacles,
    )


def check_heuristic(state, h, need_consistent=True):
    """
    Kiểm tra admissibility, consistency và giá trị vô hạn.
    Giả định mỗi cạnh trong đồ thị có chi phí bằng 1.
    """
    b = S.build_board(state)
    start = S.start_key(state)

    # Xây dựng đồ thị và tính khoảng cách thực đến đích
    pred, edges = build_graph(b, start)
    dist = true_dist(b, pred)

    # Lưu heuristic để tránh tính lại nhiều lần
    values = {s: h(b, s) for s in dist}

    # Admissibility: h(s) không được vượt chi phí tối ưu
    adm = [
        (s, values[s], d)
        for s, d in dist.items()
        if values[s] > d
    ]

    # Consistency: h(s) <= 1 + h(s')
    cons = [
        (s, s2)
        for s, s2, _ in edges
        if s in dist
        and s2 in dist
        and values[s] > 1 + values[s2]
    ]

    # Đếm trạng thái giải được nhưng heuristic trả về vô hạn
    inf_on_solvable = sum(
        1 for s in dist
        if values[s] >= S.INF
    )

    return dict(
        states=len(pred),
        solvable=len(dist),
        adm_viol=len(adm),
        cons_viol=len(cons),
        inf_on_solvable=inf_on_solvable,
        adm_examples=adm[:3],
        ok=(
            not adm
            and inf_on_solvable == 0
            and (not need_consistent or not cons)
        )
    )


class TestHeuristic(unittest.TestCase):
    """Kiểm thử ba heuristic h1, h2 và h3."""

    def test_h1(self):
        state = corridor_state()
        result = check_heuristic(
            state,
            lambda b, s: S.h1(_make_state(state, b, s), b)
        )

        self.assertGreater(result["solvable"], 0)
        self.assertEqual(result["adm_viol"], 0)
        self.assertEqual(result["cons_viol"], 0)
        self.assertEqual(result["inf_on_solvable"], 0)
        self.assertTrue(result["ok"], result)

    def test_h2(self):
        state = corridor_state()
        result = check_heuristic(
            state,
            lambda b, s: S.h2(_make_state(state, b, s), b)
        )

        self.assertGreater(result["solvable"], 0)
        self.assertEqual(result["adm_viol"], 0)
        self.assertEqual(result["cons_viol"], 0)
        self.assertEqual(result["inf_on_solvable"], 0)
        self.assertTrue(result["ok"], result)

    def test_h3(self):
        state = corridor_state()
        result = check_heuristic(
            state,
            lambda b, s: S.h3(_make_state(state, b, s), b)
        )

        self.assertGreater(result["solvable"], 0)
        self.assertEqual(result["adm_viol"], 0)
        self.assertEqual(result["cons_viol"], 0)
        self.assertEqual(result["inf_on_solvable"], 0)
        self.assertTrue(result["ok"], result)


if __name__ == "__main__":
    unittest.main()
