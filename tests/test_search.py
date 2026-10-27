"""Test Khu 4 (vai trò A) — phần 1: biểu diễn khóa số nguyên.

Kiểm rằng engine fast thấy đúng những gì Khu 1-3 thấy:
  - pack/unpack không làm mất thông tin;
  - h1, h2, h3 bản khóa số nguyên bằng đúng bản trên SokobanState của C;
  - dead_after_push bằng đúng is_dead sau mỗi lần đẩy, ở mode cells và freeze.
Chạy: python -m unittest discover -s tests -p "test_search.py" -v
"""
import random
import unittest

import solution as S
from sokoban import PROBLEMS

STEPS = 2000          # số bước đi ngẫu nhiên mỗi seed, giống test của B
SEEDS = (0, 1, 2)


def pushed_cell(board, parent, child):
    """Ô thùng vừa được đẩy tới (có ở con, không có ở cha), hoặc None nếu chỉ đi bộ."""
    W = board.W
    added = ({y * W + x for x, y in child.boxes}
             - {y * W + x for x, y in parent.boxes})
    return next(iter(added)) if added else None


def random_walk(state, seed, steps=STEPS):
    """Sinh lần lượt các SokobanState trên một đường đi ngẫu nhiên bằng framework."""
    rnd = random.Random(seed)
    cur = state
    for _ in range(steps):
        yield cur
        kids = cur.successors()
        if not kids:
            return
        cur = rnd.choice(kids)


class TestPackUnpack(unittest.TestCase):
    def test_roundtrip_all_maps(self):
        old_sym = S.CONFIG["sym"]
        try:
            for sym in (True, False):
                S.CONFIG["sym"] = sym
                for i, st in enumerate(PROBLEMS):
                    c = S.make_ctx(S.build_board(st), len(st.robots))
                    robots, boxes = S.start_key(st)
                    got_robots, got_boxes = S.unpack(c, S.pack(c, list(robots), boxes))
                    want = sorted(robots) if sym else list(robots)
                    self.assertEqual(got_robots, want, "map %d sym=%s" % (i, sym))
                    self.assertEqual(got_boxes, boxes, "map %d sym=%s" % (i, sym))
        finally:
            S.CONFIG["sym"] = old_sym

    def test_symmetric_states_share_key(self):
        """Hai state chỉ đổi chỗ hai robot cho cùng khóa khi bật sym, khác khóa khi tắt."""
        old_sym = S.CONFIG["sym"]
        try:
            st = PROBLEMS[3]                     # 2 robot
            c = S.make_ctx(S.build_board(st), len(st.robots))
            robots, boxes = S.start_key(st)
            swapped = list(reversed(robots))
            S.CONFIG["sym"] = True
            self.assertEqual(S.pack(c, list(robots), boxes), S.pack(c, swapped, boxes))
            S.CONFIG["sym"] = False
            self.assertNotEqual(S.pack(c, list(robots), boxes), S.pack(c, swapped, boxes))
        finally:
            S.CONFIG["sym"] = old_sym


class TestMatchesFramework(unittest.TestCase):
    def test_h_matches_framework(self):
        checked = 0
        for i, st in enumerate(PROBLEMS):
            board = S.build_board(st)
            c = S.make_ctx(board, len(st.robots))
            fast = {name: S.fast_h(c, name) for name in ("h1", "h2", "h3")}
            for seed in SEEDS:
                for cur in random_walk(st, seed):
                    robots, boxes = S.start_key(cur)
                    for name, h in fast.items():
                        self.assertEqual(
                            h(list(robots), boxes), S.HEURISTICS[name](cur, board),
                            "map %d seed %d %s\n%s" % (i, seed, name, cur.state_string()))
                        checked += 1
        self.assertGreater(checked, 0)

    def test_dead_matches_framework(self):
        old_mode = S.CONFIG["dead"]
        checked = 0
        try:
            for i, st in enumerate(PROBLEMS):
                board = S.build_board(st)
                for seed in SEEDS:
                    for cur in random_walk(st, seed):
                        for child in cur.successors():
                            pushed = pushed_cell(board, cur, child)
                            if pushed is None:
                                continue
                            _, boxes = S.start_key(child)
                            for mode in ("cells", "freeze"):
                                S.CONFIG["dead"] = mode
                                self.assertEqual(
                                    S.dead_after_push(board, boxes, pushed, mode),
                                    S.is_dead(child, board),
                                    "map %d seed %d mode %s\n%s" % (
                                        i, seed, mode, child.state_string()))
                                checked += 1
        finally:
            S.CONFIG["dead"] = old_mode
        self.assertGreater(checked, 0)


if __name__ == "__main__":
    unittest.main()