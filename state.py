# TODO: Chep file state.py tu framework giang vien cung cap; khong sua file framework.
"""State interface only. No search algorithm is provided."""


class StateSpace:
    """Lớp cơ sở cho state có liên kết parent để dựng lại lời giải."""

    n = 0

    def __init__(self, action, gval, parent):
        """Lưu hành động, chi phí, parent và cấp một chỉ số tăng dần; O(1)."""
        self.action = action
        self.gval = gval
        self.parent = parent
        self.index = StateSpace.n
        StateSpace.n += 1

    def successors(self):
        """Giao diện sinh state kế tiếp; lớp con cần cài đặt."""
        raise NotImplementedError

    def hashable_state(self):
        """Giao diện trả về khóa bất biến để nhận diện cấu hình."""
        raise NotImplementedError

    def print_state(self):
        """Giao diện in thông tin của một state; lớp con cần cài đặt."""
        raise NotImplementedError

    def print_path(self):
        """Lần parent về đầu rồi in đường đi; O(L) state ngoài thời gian in."""
        path = []
        current = self
        while current is not None:
            path.append(current)
            current = current.parent
        for item in reversed(path):
            item.print_state()
