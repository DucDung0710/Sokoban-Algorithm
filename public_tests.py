# TODO:
"""Small API examples only; no prescribed search algorithm."""
import unittest
import solution
from sokoban import SokobanState
from validation import signature, validate_solution


def simple_state(solved=False):
    """Tạo map một hộp nhỏ, tùy chọn đặt hộp sẵn trên đích."""
    return SokobanState(
        "START", 0, None, 4, 3, ((0, 1),),
        frozenset([(2, 1) if solved else (1, 1)]),
        frozenset([(2, 1)]), frozenset())


class PublicTests(unittest.TestCase):
    def test_one_box(self):
        """Kiểm tra lời giải một hộp và state đầu vào không bị sửa."""
        initial = simple_state()
        before = (signature(initial), initial.action, initial.gval, initial.parent)
        final = solution.solve(initial, timebound=120)
        self.assertEqual(
            (signature(initial), initial.action, initial.gval, initial.parent), before)
        valid, detail = validate_solution(simple_state(), final)
        self.assertTrue(valid, detail)

    def test_initial_goal(self):
        """Kiểm tra solver nhận state ban đầu đã là đích."""
        initial = simple_state(solved=True)
        final = solution.solve(initial, timebound=120)
        valid, detail = validate_solution(simple_state(solved=True), final)
        self.assertTrue(valid, detail)

    def test_zero_budget(self):
        """Kiểm tra ngân sách bằng không trả về thất bại."""
        self.assertIs(solution.solve(simple_state(), timebound=0), False)


if __name__ == "__main__":
    unittest.main()
