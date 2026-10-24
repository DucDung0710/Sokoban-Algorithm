"""Test đối chiếu Khu 1 (vai trò B) với framework """
import random
import unittest
from collections import deque

import sokoban
import solution as S
from sokoban import PROBLEMS

IS_MOCK = getattr(sokoban, "IS_MOCK", False)
OPTIMAL_0_7 = [16, 4, 21, 10, 8, 30, 16, 41]   # BFS trên map 0-7 (tài liệu mục 2.3)


def goal(state):
    return sokoban.sokoban_goal_state(state)


def internal_bfs(b, state, max_states=300000):
    """BFS trên state nội bộ. Trả danh sách action tối ưu, hoặc None nếu quá trần."""
    s0 = S.start_key(state)
    if S.is_goal(b, s0):
        return []
    prev = {s0: None}
    queue = deque([s0])
    while queue:
        s = queue.popleft()
        for a, s2, _ in S.successors_fast(b, s):
            if s2 in prev:
                continue
            prev[s2] = (s, a)
            if S.is_goal(b, s2):
                path = []
                cur = s2
                while prev[cur] is not None:
                    cur, a2 = prev[cur]
                    path.append(a2)
                return path[::-1]
            queue.append(s2)
        if len(prev) > max_states:
            return None
    return None


class TestPrimitives(unittest.TestCase):
    def test_cells(self):
        self.assertEqual(list(S.cells(0)), [])
        self.assertEqual(list(S.cells(0b10110)), [1, 2, 4])
        self.assertEqual(list(S.cells(1 << 63)), [63])

    def test_start_key(self):
        for st in PROBLEMS:
            W = st.width
            robots, boxes = S.start_key(st)
            self.assertEqual(robots, tuple(y * W + x for (x, y) in st.robots))
            self.assertEqual(sorted(S.cells(boxes)),
                             sorted(y * W + x for (x, y) in st.boxes))
            self.assertEqual(bin(boxes).count("1"), len(st.boxes))


    def test_is_goal_subset_of_goals(self):
        """Thắng khi mọi thùng ở đích, kể cả khi số đích nhiều hơn số thùng."""
        checked_more_goals = False
        for st in PROBLEMS:
            b = S.build_board(st)
            robots, boxes = S.start_key(st)
            nb = len(st.boxes)
            if nb == 0 or nb > len(b.goals):
                continue
            on_goals = sum(1 << g for g in b.goals[:nb])         # thùng nằm trong tập đích
            self.assertTrue(S.is_goal(b, (robots, on_goals)))
            self.assertTrue(S.is_goal(b, (robots, 0)))           # không còn thùng nào
            non_goal = [p for p in range(b.N) if not (b.goal_mask >> p) & 1]
            if non_goal:                                         # một thùng lệch khỏi đích
                off = sum(1 << g for g in b.goals[:nb - 1]) | (1 << non_goal[0])
                self.assertFalse(S.is_goal(b, (robots, off)))
            if len(b.goals) > nb:
                checked_more_goals = True
        self.assertTrue(checked_more_goals, "khong co map nao nhieu dich hon thung")


class TestAgainstFramework(unittest.TestCase):
    def test_random_walk(self, steps=2000, seeds=(0, 1, 2)):
        """Đi ngẫu nhiên, mỗi state so tập (action, state mới) của hai bên."""
        for pi, st in enumerate(PROBLEMS):
            b = S.build_board(st)
            for seed in seeds:
                rnd = random.Random(seed)
                cur = st
                for step in range(steps):
                    kids = cur.successors()
                    key = S.start_key(cur)
                    fw = sorted((c.action, S.start_key(c)) for c in kids)
                    fast_full = S.successors_fast(b, key)
                    fast = sorted((a, s2) for a, s2, _ in fast_full)
                    self.assertEqual(
                        fw, fast, "map %d seed %d buoc %d: %s" % (
                            pi, seed, step, cur.state_string()))
                    self.assertEqual(S.is_goal(b, key), goal(cur))
                    # pushed: đúng một thùng đổi chỗ và tới ô pushed; None thì thùng không đổi
                    for a, (r2, bx2), pushed in fast_full:
                        if pushed is None:
                            self.assertEqual(bx2, key[1])
                        else:
                            diff = bx2 ^ key[1]
                            self.assertEqual(bin(diff).count("1"), 2)
                            self.assertTrue((bx2 >> pushed) & 1)
                            self.assertFalse((key[1] >> pushed) & 1)
                    if not kids:
                        break
                    cur = rnd.choice(kids)


class TestRebuild(unittest.TestCase):
    def test_rebuild_solutions(self):
        solved = 0
        for st in PROBLEMS:
            b = S.build_board(st)
            before = S.start_key(st)
            sol = internal_bfs(b, st)
            if sol is None:
                continue
            solved += 1
            end = S.rebuild(st, sol)
            self.assertTrue(goal(end))
            self.assertEqual(end.gval, len(sol))
            # chuỗi parent/action/gval hợp lệ như validation.py kiểm
            cur, steps = end, []
            while cur is not st and cur.parent is not None:
                self.assertIn(cur.action, [c.action for c in cur.parent.successors()])
                self.assertEqual(cur.gval, cur.parent.gval + 1)
                steps.append(cur.action)
                cur = cur.parent
            self.assertEqual(steps[::-1], sol)
            self.assertEqual(S.start_key(st), before, "initial_state bi sua")
        self.assertGreater(solved, 0)

    def test_rebuild_illegal_action(self):
        with self.assertRaises(RuntimeError):
            S.rebuild(PROBLEMS[0], ["99 up"])

    @unittest.skipIf(IS_MOCK, "chi co y nghia voi framework that")
    def test_optimal_lengths_map_0_7(self):
        for i, expected in enumerate(OPTIMAL_0_7):
            st = PROBLEMS[i]
            sol = internal_bfs(S.build_board(st), st, max_states=10 ** 7)
            self.assertEqual(len(sol), expected, "map %d" % i)


if __name__ == "__main__":
    unittest.main()