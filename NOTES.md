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
| `h2(state, board)` | Ghép hộp–đích bằng DP bitmask khi có tối đa 8 đích, Hungarian khi nhiều hơn; cache theo bitmask hộp với giới hạn `CONFIG["h_cache_max"]`. |
| `h3(state, board)` | Cộng `h2` với `max(0, d_min - 1)`, trong đó `d_min` là khoảng cách Manhattan nhỏ nhất giữa robot và hộp. Đây là cận dưới số bước đi bộ trước lần đẩy đầu. |
| `is_dead_static(state, board)` | Trả đúng nếu hộp nằm ngoài bàn hoặc trên ô không thể đẩy tới đích; A* dùng hàm này để cắt state đó. |
| `has_frozen_box_deadlock(state, board, candidates)` | Kiểm tra các hộp ứng viên; một trục bị khóa bởi tường, hai ô dead hoặc hộp đã chứng minh frozen. |
| `is_dead(state, board, pushed)` | Hỗ trợ `CONFIG["dead"]` là `none`, `cells` hoặc `freeze`; sau push chỉ xét ô đích và các hộp kề. |
| `is_dead_none(state)` | Hàm mặc định không loại state nào, dùng khi gọi A* mà không truyền deadlock checker. |

`solve` tiền tính bảng một lần, chọn heuristic theo `CONFIG["h"]` (mặc định là
`h2`), rồi truyền heuristic và bộ phát hiện deadlock vào tìm kiếm
anytime với các trọng số giảm dần trong `WEIGHTS`. Khi tìm được lời giải, các
pha tiếp theo cố gắng cải thiện số bước trong phần thời gian còn lại. Deadlock
hiện tại là deadlock tĩnh và freeze có thể chứng minh bằng tường/hộp đã khóa;
chưa xử lý mọi dạng deadlock động hoặc mẫu deadlock phức tạp. Dùng
`CONFIG["dead"]` để bật/tắt theo từng mức: `none`, `cells`, `freeze`.

`h1`, `h2`, `h3` đều là cận dưới: lần lượt ghép hộp với đích gần nhất, ghép
hộp với các đích khác nhau tối ưu, và cộng thêm số bước đi bộ tối thiểu trước
lần đẩy đầu. `h1` và `h2` consistent theo cạnh đẩy; `h3` cũng consistent vì
phần Manhattan giảm nhiều nhất một ở bước đi bộ và bằng 0 ngay trước push.

Chạy kiểm thử bằng `python -m unittest discover -s tests -v` từ thư mục dự án.
Kiểm thử hiện tại chạy trên Python 3.13.5: 11 test phần C, 7 test framework,
3 public test và 7 test thuật toán hỗ trợ đều qua. Kiểm tra vét cạn map 4
trước đó chạy trên Python 3.15.0: trong 127,512 state đến được bằng BFS,
12,936 state có thể giải; `h1`, `h2`, `h3` không vượt quá khoảng cách lời
giải tối ưu của bất kỳ state giải được nào.

Benchmark A* dưới đây chạy với ngân sách 1.5 giây/lượt trên map 1, 4, 6
(map 0, 3, 5 trong `PROBLEMS`). Mỗi ô ghi `state mở rộng / deadlock bị cắt`;
`timeout` nghĩa là chưa tìm được đích trước hạn.

| Map | h | Không deadlock | Freeze |
| --- | --- | ---: | ---: |
| 1 | h0 | 11,999 / 0 (timeout) | 13,499 / 1,382 (timeout) |
| 1 | h1 | 6,494 / 0 (16 bước) | 6,354 / 460 (16 bước) |
| 1 | h2 | 6,434 / 0 (16 bước) | 6,298 / 456 (16 bước) |
| 1 | h3 | 6,108 / 0 (16 bước) | 5,976 / 456 (16 bước) |
| 4 | h0 | 10,874 / 0 (10 bước) | 2,814 / 1,071 (10 bước) |
| 4 | h1 | 634 / 0 (10 bước) | 634 / 320 (10 bước) |
| 4 | h2 | 542 / 0 (10 bước) | 542 / 260 (10 bước) |
| 4 | h3 | 467 / 0 (10 bước) | 467 / 260 (10 bước) |
| 6 | h0 | 15,499 / 0 (timeout) | 15,999 / 2,924 (timeout) |
| 6 | h1 | 17,871 / 0 (30 bước) | 13,993 / 2,590 (timeout) |
| 6 | h2 | 12,173 / 0 (30 bước) | 11,015 / 2,136 (30 bước) |
| 6 | h3 | 11,175 / 0 (30 bước) | 10,137 / 2,136 (30 bước) |

Tất cả lời giải tìm được đều có độ dài tối ưu đã ghi: map 1 là 16 bước, map 4
là 10 bước, map 6 là 30 bước. Số node và timeout phụ thuộc máy; freeze check
có thêm chi phí mỗi lần đẩy nên có thể chậm hơn dù số node giảm.

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

- Bổ sung test cho ghép cặp, admissibility/consistency của `h1`–`h3` so với BFS
  trên bản đồ nhỏ, cache giới hạn, chế độ deadlock, dead square, freeze và
  kiểm tra sound trên hành lang nhỏ.
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

- Freeze detector chỉ kiểm quanh hộp vừa đẩy (và các hộp kề), nên có thể bỏ
  sót freeze lan xa; bỏ sót an toàn nhưng có thể làm tăng số node.
- Test hiện kiểm admissible và consistency của `h1`–`h3` trên các bàn nhỏ;
  không thay thế được bộ kiểm vét cạn của D trên toàn bộ map 0–7.
- `h0` chạm timeout ở một số lượt map 1 và map 6 với giới hạn 1.5 giây; các
  kết quả này được ghi là timeout, không tính là đã tìm thấy lời giải.
- Benchmark thực nghiệm phụ thuộc máy và ngân sách thời gian; cần chạy lại
  nếu môi trường hoặc cấu hình thay đổi.

### Công cụ hỗ trợ

Ngày 19/10/2026, dùng Copilot trong VS Code hỗ trợ chỉnh sửa mã, test và tài
liệu; dùng Python 3.15.0 và các script/test có sẵn trong repository để kiểm
tra. Không thêm dependency. Không ghi nhận thay đổi hợp đồng API.
