from collections import deque
import unittest

from solution import INF, build_board, h1
from sokoban import SokobanState, sokoban_goal_state


def corridor_state(robot, box):
    """Tạo state trong hành lang một ô để tính khoảng cách chính xác."""
    walls = frozenset((x, y) for x in range(5) for y in (0, 2))
    return SokobanState(
        "START", 0, None, 5, 3, (robot,), frozenset((box,)),
        frozenset(((4, 1),)), walls,
    )


def shortest_solution_steps(initial):
    """Tính h* bằng BFS toàn state; O(V·R(R+B)) trong framework này."""
    queue = deque([(initial, 0)])
    visited = {initial.hashable_state()}
    while queue:
        state, distance = queue.popleft()
        if sokoban_goal_state(state):
            return distance
        for child in state.successors():
            key = child.hashable_state()
            if key not in visited:
                visited.add(key)
                queue.append((child, distance + 1))
    return None


class PushDistanceTests(unittest.TestCase):
    def test_reverse_push_requires_robot_support_square(self):
        """Ô sau thùng phải sàn; góc không thể đẩy ra được đánh dấu dead."""
        state = corridor_state((0, 1), (1, 1))
        board = build_board(state)

        self.assertEqual(board.pdc[0][4 + board.W], 0)
        self.assertEqual(board.pd_min[1 + board.W], 3)
        self.assertEqual(board.pd_min[0 + board.W], INF)
        self.assertIn(0 + board.W, board.dead)

    def test_h1_is_admissible_and_consistent_on_small_board(self):
        """So h1 với h* và kiểm mọi cạnh trong không gian một hộp nhỏ."""
        floor_cells = tuple((x, 1) for x in range(5))
        checked_solvable = 0
        checked_edges = 0

        for robot in floor_cells:
            for box in floor_cells:
                if robot == box:
                    continue
                state = corridor_state(robot, box)
                board = build_board(state)
                value = h1(state, board)
                exact = shortest_solution_steps(state)

                if value == INF:
                    self.assertIsNone(exact, "Dead square must be unsolvable")
                elif exact is not None:
                    self.assertLessEqual(value, exact, "h1 must not exceed h*")
                    checked_solvable += 1

                for child in state.successors():
                    checked_edges += 1
                    child_value = h1(child, board)
                    if value == INF:
                        self.assertEqual(child_value, INF)
                    else:
                        self.assertLessEqual(value, 1 + child_value)

        self.assertGreater(checked_solvable, 0)
        self.assertGreater(checked_edges, 0)

    def test_solve_uses_push_distance_heuristic(self):
        """Kiểm tra A* tìm được lời giải hợp lệ với h1."""
        from solution import solve

        initial = corridor_state((0, 1), (1, 1))
        goal = solve(initial, timebound=5)
        self.assertIsInstance(goal, SokobanState)
        self.assertTrue(sokoban_goal_state(goal))
        self.assertEqual(goal.gval, 3)


if __name__ == "__main__":
    unittest.main()
