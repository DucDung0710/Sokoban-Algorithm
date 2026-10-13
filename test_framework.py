# TODO: Chep file test_framework.py tu framework giang vien cung cap; chay truoc khi co solver.
"""Tests of supplied game mechanics only; no student algorithms needed."""
import unittest
from sokoban import SokobanState, sokoban_goal_state, PROBLEMS
from validation import validate_solution, signature


def board(robots=((0, 1),), boxes=((1, 1),), storage=((2, 1),), obstacles=()):
    """Tạo state test và chuyển hộp/đích/tường thành frozenset; O(R+B+G+T)."""
    return SokobanState("START", 0, None, 4, 3, robots, frozenset(boxes),
                        frozenset(storage), frozenset(obstacles))


class FrameworkTests(unittest.TestCase):
    def test_push_and_cost(self):
        """Kiểm tra đẩy hợp lệ, chi phí, parent và state đầu không đổi."""
        start = board()
        before = signature(start)
        goal = next(s for s in start.successors() if s.action == "0 right")
        self.assertTrue(sokoban_goal_state(goal))
        self.assertEqual(goal.gval, 1)
        self.assertIs(goal.parent, start)
        self.assertEqual(signature(start), before)
        self.assertTrue(validate_solution(start, goal)[0])

    def test_no_double_push(self):
        """Kiểm tra không thể đẩy hộp vào hộp khác; sinh successor O(R(R+B))."""
        self.assertNotIn("0 right", [s.action for s in board(boxes=((1, 1), (2, 1))).successors()])

    def test_robot_blocks_push(self):
        """Kiểm tra robot khác chặn ô đẩy tới; sinh successor O(R(R+B))."""
        start = board(robots=((0, 1), (2, 1)))
        self.assertNotIn("0 right", [s.action for s in start.successors()])

    def test_obstacle_blocks_push(self):
        """Kiểm tra không thể đẩy hộp vào tường; sinh successor O(R(R+B))."""
        self.assertNotIn("0 right", [s.action for s in board(obstacles=((2, 1),)).successors()])

    def test_multiple_robots_move_one_at_a_time(self):
        """Kiểm tra mỗi successor chỉ di chuyển một robot và tốn một bước."""
        start = board(robots=((0, 1), (3, 2)))
        for successor in start.successors():
            self.assertEqual(sum(a != b for a, b in zip(start.robots, successor.robots)), 1)
            self.assertEqual(successor.gval, 1)

    def test_state_key_and_storage(self):
        """Kiểm tra state key, nhận diện đích O(B) và số map mẫu."""
        start = board(boxes=((2, 1),), storage=((2, 1), (3, 2)))
        self.assertIsInstance(start.hashable_state(), tuple)
        self.assertTrue(sokoban_goal_state(start))
        self.assertEqual(len(PROBLEMS), 20)

    def test_validator_rejects_forgery(self):
        """Kiểm tra validator từ chối state đầu sai, action sai và parent lặp."""
        initial, forged = board(), board(boxes=((2, 1),))
        self.assertFalse(validate_solution(initial, forged)[0])
        forged.parent, forged.gval, forged.action = initial, 1, "illegal"
        self.assertFalse(validate_solution(initial, forged)[0])
        forged.parent = forged
        self.assertFalse(validate_solution(initial, forged)[0])


if __name__ == "__main__":
    unittest.main()
