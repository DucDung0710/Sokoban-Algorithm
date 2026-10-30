import unittest

import solution as S
from sokoban import SokobanState
from heuristic_test import (_make_state,build_graph,corridor_state,true_dist,
)


def check_sound(state, mode, graph_builder, distance_builder):
    if state is None:
        raise ValueError("State chưa được khởi tạo.")

    if mode not in ("none", "cells", "freeze"):
        raise ValueError(f"Mode deadlock không hợp lệ: {mode!r}")

    if not callable(graph_builder):
        raise ValueError("graph_builder phải là một hàm.")

    if not callable(distance_builder):
        raise ValueError("distance_builder phải là một hàm.")

    old_mode = S.CONFIG["dead"]

    try:
        S.CONFIG["dead"] = mode

        board = S.build_board(state)
        start = S.start_key(state)

        pred, edges = graph_builder(board, start)
        dist = distance_builder(board, pred)

        # Mỗi cặp state sau push và ô nhận hộp chỉ được tính một lần.
        pushes = list(dict.fromkeys(
            (state_after, pushed)
            for _, state_after, pushed in edges
            if pushed is not None
        ))

        unsound_examples = []
        dead_pushes = []
        caught = 0

        for state_after, pushed in pushes:
            sokoban_state = _make_state(state, board, state_after)
            predicted_dead = S.is_dead(sokoban_state, board, pushed)
            actually_solvable = state_after in dist

            if predicted_dead and actually_solvable:
                unsound_examples.append((state_after, pushed))

            if not actually_solvable:
                dead_pushes.append((state_after, pushed))
                if predicted_dead:
                    caught += 1

        return {
            "states": len(pred),
            "total_pushes": len(pushes),
            "solvable_pushes": len(pushes) - len(dead_pushes),
            "dead_pushes": len(dead_pushes),
            "caught": caught,
            "catch_rate": (
                caught / len(dead_pushes)
                if dead_pushes
                else None
            ),
            "unsound": len(unsound_examples),
            "examples": unsound_examples[:3],
            "ok": not unsound_examples,
        }

    finally:
        S.CONFIG["dead"] = old_mode


class TestDeadlock(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.state = corridor_state()

    def test_soundness(self):
        result = check_sound(
            self.state,
            "freeze",
            build_graph,
            true_dist,
        )

        self.assertGreater(
            result["total_pushes"],
            0,
            "Fixture không tạo ra push nào để kiểm tra.",
        )
        self.assertGreater(
            result["solvable_pushes"],
            0,
            "Fixture cần có push dẫn tới trạng thái giải được.",
        )
        self.assertEqual(
            result["unsound"],
            0,
            f"Detector báo nhầm trạng thái giải được: {result['examples']}",
        )

    def test_detection_rate(self):
        # Open board: the box can reach the goal or be pushed into the
        # bottom-right corner, which is a dead square.
        state = SokobanState("START", 0, None, 5, 5, # dimensions
                         ((2, 1), (2, 3)), #robots
                         frozenset(((1, 1), (1, 3), (3, 1), (3, 3))), #boxes
                         frozenset(((0, 0), (0, 4), (4, 0), (4, 4))), #storage
                         frozenset(((1, 0), (2, 0), (3, 0), (1, 4), (2, 4), (3, 4))) #obstacles
                         )
        result = check_sound(
            state,
            "freeze",
            build_graph,
            true_dist,
        )

        self.assertGreater(
            result["dead_pushes"],
            0,
            "Fixture không có push nào dẫn tới trạng thái không thể giải.",
        )
        self.assertGreater(
            result["caught"],
            0,
            "Detector không bắt được deadlock nào trong fixture.",
        )
        self.assertIsNotNone(result["catch_rate"])

        print(f"\nCatch rate: {result['catch_rate'] * 100:.2f}%")


if __name__ == "__main__":
    unittest.main(verbosity=2)