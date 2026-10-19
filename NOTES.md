# Ghi chú phần C — Heuristic và Deadlock

Phạm vi phần C trong `solution.py`: ước lượng số bước đẩy còn lại và loại bỏ
state có hộp nằm ở ô chắc chắn không thể đưa tới bất kỳ đích nào.

| Hàm | Ý nghĩa và cách hoạt động |
| --- | --- |
| `build_board(state)` | Mã hóa mỗi ô thành số nguyên, tạo bảng ô kề và gọi hàm tiền tính khoảng cách đẩy từ các đích. |
| `precompute_push_dist(board)` | BFS ngược từ từng đích; chỉ cho phép một lần đẩy ngược nếu ô hộp trước đó và ô robot đứng đều là sàn. Ghi khoảng cách, khoảng cách nhỏ nhất tới đích (`pd_min`) và ô dead (`dead`). |
| `h_zero(state)` | Trả `0`, dùng khi cần tìm kiếm không có heuristic. |
| `h_box_manhattan(state)` | Cộng khoảng cách Manhattan từ mỗi hộp tới đích gần nhất; dễ tính nhưng không xét tường. |
| `h1(state, board)` | Cộng số lần đẩy tĩnh ít nhất của mỗi hộp tới một đích. Trả vô cực nếu hộp không thể tới đích trên đồ thị đẩy; là cận dưới admissible. |
| `_minimum_assignment_cost(costs)` | Dùng Hungarian để tìm ghép chi phí nhỏ nhất giữa các hộp và các đích khác nhau; trả vô cực nếu không tồn tại ghép hợp lệ. |
| `h2(state, board)` | Tạo bảng push-distance hộp–đích rồi gọi Hungarian; tránh đánh giá hai hộp như thể cùng một đích có thể nhận cả hai. |
| `h3(state, board)` | Cộng `h2` với khoảng cách tường-ngắn nhất từ robot tới vị trí hỗ trợ của một lần đẩy đầu tiên. BFS bỏ qua hộp để phần cộng thêm vẫn là cận dưới. |
| `is_dead_static(state, board)` | Trả đúng nếu hộp nằm ngoài bàn hoặc trên ô không thể đẩy tới đích; A* dùng hàm này để cắt state đó. |
| `has_frozen_box_deadlock(state, board)` | Lan truyền các hộp bị khóa bởi tường/hộp đã khóa theo hai trục; báo deadlock nếu hộp bị khóa không nằm trên đích. |
| `is_dead(state, board)` | Kết hợp phát hiện ô dead tĩnh và freeze deadlock; được `solve` truyền cho A*. |
| `is_dead_none(state)` | Hàm mặc định không loại state nào, dùng khi gọi A* mà không truyền deadlock checker. |

`solve` tiền tính bảng một lần, chọn `h1` (cấu hình mặc định), rồi truyền
heuristic và bộ phát hiện deadlock vào `astar`. Deadlock hiện tại là deadlock
tĩnh và freeze có thể chứng minh bằng tường/hộp đã khóa; chưa xử lý mọi dạng
deadlock động hoặc mẫu deadlock phức tạp.

`h1`, `h2`, `h3` đều là cận dưới: lần lượt ghép hộp với đích gần nhất, ghép
hộp với các đích khác nhau tối ưu, và cộng thêm số bước đi bộ tối thiểu trước
lần đẩy đầu. Các heuristic có thể chọn bằng biến `HEURISTIC` trong
`solution.py`; mặc định vẫn là `h1`.

Chạy kiểm thử bằng `python -m unittest discover -s tests -v` từ thư mục dự án.
Kiểm thử đã chạy trên Python 3.15.0: 7 test phần C, 7 test framework và 3
public test đều qua. Thêm kiểm tra vét cạn map 4: trong 127,512 state đến
được bằng BFS, 12,936 state có thể giải; `h1`, `h2`, `h3` không vượt quá
khoảng cách lời giải tối ưu của bất kỳ state giải được nào.

Benchmark A* dưới đây chạy với ngân sách 1.5 giây/lượt. Ghi số state mở rộng;
deadlock là số state bị cắt. `timeout` nghĩa là chưa tìm được đích trước hạn.
Mỗi cặp số trong cột mở rộng là `tắt / bật deadlock`.

| Map | Heuristic | Mở rộng tắt / bật | Deadlock bị cắt (bật) | Kết quả |
| --- | --- | ---: | ---: | --- |
| 1 | h0 | 51,916 / 48,499 | 759 | goal / timeout |
| 1 | h1 | 6,494 / 6,490 | 4 | goal / goal |
| 1 | h2 | 6,434 / 6,434 | 4 | goal / goal |
| 1 | h3 | 5,089 / 5,089 | 4 | goal / goal |
| 4 | h0 | 10,874 / 2,814 | 1,071 | goal / goal |
| 4 | h1 | 634 / 634 | 320 | goal / goal |
| 4 | h2 | 542 / 542 | 260 | goal / goal |
| 4 | h3 | 467 / 467 | 260 | goal / goal |
| 6 | h0 | 61,999 / 30,068 | 4,009 | timeout / goal |
| 6 | h1 | 17,871 / 17,399 | 2,808 | goal / goal |
| 6 | h2 | 12,173 / 11,977 | 2,080 | goal / goal |
| 6 | h3 | 10,336 / 10,212 | 2,080 | goal / goal |

Tất cả các lượt tìm được lời giải đều trả cùng độ dài cho mỗi map: map 1 là
16 bước, map 4 là 10 bước, map 6 là 30 bước. Map tự tạo hành lang giải được
trong 3 bước với cả bốn heuristic; A* mở rộng 4 state với `h0`, 3 state với
`h1`/`h2`/`h3`. Thời gian chạy thực tế và số node có thể thay đổi theo máy.
`h3` mở rộng ít state nhất trong các lượt đã giải; kết quả này chưa chứng minh
nó luôn nhanh hơn trên mọi bản đồ.

## C - Heuristics and Deadlocks

### Phạm vi

Hoàn thiện và kiểm tra phần C (vai trò C: heuristic và deadlock) trong
`solution.py`. Không thay đổi framework Sokoban (`sokoban.py`, `state.py`) hay
hợp đồng API công khai `solve(initial_state, timebound=120)`.

### Thay đổi mã nguồn

- Giữ lại BFS ngược theo từng đích trong `precompute_push_dist`; bảng chỉ cho
  phép cạnh đẩy ngược khi ô hộp trước và ô robot hỗ trợ đều là sàn. Từ đó tạo
  `pd_min` và tập ô `dead` không thể đẩy tới bất kỳ đích nào.
- Bổ sung `is_dead_static`: cắt state nếu hộp nằm ngoài bàn hoặc trên ô dead.
- Bổ sung `_minimum_assignment_cost` dùng Hungarian và `h2`, ghép hộp với các
  đích riêng biệt theo tổng push-distance nhỏ nhất.
- Bổ sung `h3`: lấy `h2` cộng cận dưới khoảng cách đi bộ của robot tới một ô
  hỗ trợ lần đẩy đầu tiên. BFS khoảng cách đi bộ bỏ qua hộp để không đánh giá
  chi phí cao hơn thực tế.
- Bổ sung `has_frozen_box_deadlock` và `is_dead`: đánh dấu hộp bị khóa theo cả
  hai trục bởi tường/hộp đã khóa; chỉ loại nếu hộp bị khóa không nằm trên đích.
- Nối deadlock detector vào A* trong `solve`; heuristic mặc định vẫn là `h1`.
  Có thể chọn `h0`, `h1`, `h2` hoặc `h3` qua biến `HEURISTIC`.

### Kiểm thử và benchmark

- Bổ sung test cho Hungarian, admissibility của `h1`–`h3` so với BFS chính
  xác trên bản đồ nhỏ, dead square, freeze deadlock (kể cả hộp đã ở đích) và
  thao tác cắt state của A*.
- Môi trường: Python 3.15.0; không cài package ngoài.
- Kết quả `python -m unittest discover -s tests -v`: 7/7 test phần C qua.
- Kết quả `python -m unittest test_framework -v`: 7/7 test framework qua.
- Kết quả `python autograder.py`: 3/3 public test qua; đây là feedback công
  khai, không phải điểm chính thức.
- Kết quả `python check_imports.py`: qua, không in cảnh báo.
- Chạy benchmark A* trên map 1, 4, 6 và map hành lang tự tạo; so sánh `h0`–`h3`
  khi bật/tắt deadlock, với ngân sách 1.5 giây mỗi lượt. Bảng node mở rộng,
  số state bị cắt, kết quả và độ dài lời giải ở phần benchmark phía trên.
- Kiểm tra vét cạn map 4: 127.512 state được duyệt, 12.936 state giải được;
  không thấy trường hợp `h1`, `h2` hoặc `h3` lớn hơn khoảng cách tối ưu.
- `git diff --check`: sạch tại thời điểm kiểm tra.

### Tài liệu cập nhật

- `NOTES.md`: mô tả hàm, cách chọn heuristic, giới hạn, lệnh test và số liệu.
- `REPORT_TEMPLATE.md`: thêm giải thích admissible cho `h2`/`h3`, phạm vi
  freeze detection và ghi chú benchmark.
- `presentation/slides.md`: thêm nội dung slides 7–9 về heuristic, deadlock,
  kiểm thử và benchmark.
- `tests/test_solution.py`: thêm các kiểm thử phần C nêu trên.

### Giới hạn và lưu ý

- Freeze detector có chủ đích bảo thủ; chưa bao phủ mọi freeze/deadlock nhiều
  hộp hoặc deadlock động.
- `h2`/`h3` chưa được khẳng định consistent; benchmark cho thấy hiệu quả trên
  một số map mẫu, không đảm bảo tương tự trên mọi map.
- `h0` chạm timeout ở một số lượt map 1 và map 6 với giới hạn 1.5 giây; các
  kết quả này được ghi là timeout, không tính là đã tìm thấy lời giải.
- Benchmark thực nghiệm phụ thuộc máy và ngân sách thời gian; cần chạy lại
  nếu môi trường hoặc cấu hình thay đổi.

### Công cụ hỗ trợ

Ngày 19/10/2026, dùng Copilot trong VS Code hỗ trợ chỉnh sửa mã, test và tài
liệu; dùng Python 3.15.0 và các script/test có sẵn trong repository để kiểm
tra. Không thêm dependency. Không ghi nhận thay đổi hợp đồng API.
