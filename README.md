# Sokoban Solver — Trí tuệ nhân tạo (UET)

Bộ giải Sokoban nhiều robot viết bằng Python thuần, triển khai hàm
`solve(initial_state, timebound)` trong `solution.py`. Solver dùng
**Weighted A\* dạng anytime**: tìm nhanh một lời giải với trọng số lớn, sau đó
giảm dần trọng số để rút ngắn đường đi trong thời gian còn lại, và luôn giữ
lời giải tốt nhất đã tìm được.

> Chỉ dùng thư viện chuẩn Python 3.10+ và framework được cung cấp.
> Bài nộp chính thức là file `solution.py`.

---

## Mục lục

1. [Bài toán](#1-bài-toán)
2. [Cấu trúc repo](#2-cấu-trúc-repo)
3. [Cài đặt và chạy nhanh](#3-cài-đặt-và-chạy-nhanh)
4. [Thiết kế solver](#4-thiết-kế-solver)
5. [Cấu hình](#5-cấu-hình)
6. [Kiểm thử](#6-kiểm-thử)
7. [Đo hiệu năng](#7-đo-hiệu-năng)
8. [Kết quả](#8-kết-quả)
9. [Giới hạn đã biết](#9-giới-hạn-đã-biết)
10. [Quy ước làm việc nhóm](#10-quy-ước-làm-việc-nhóm)

---

## 1. Bài toán

- Bàn chơi gồm **robot**, **thùng**, **ô đích** (storage) và **tường**. Có thể có
  nhiều robot và nhiều ô đích hơn số thùng.
- Mỗi hành động cho **một** robot đi một ô (lên, phải, xuống, trái). Robot đẩy được
  thùng khi ô phía sau thùng trống và nằm trong bàn chơi; không kéo thùng, không
  đẩy hai thùng cùng lúc, không đi xuyên tường hay robot khác.
- Mục tiêu: mọi thùng nằm trên ô đích.
- **Chi phí = tổng số bước di chuyển của mọi robot** (đi bộ cũng tính, không chỉ
  số lần đẩy).

Giao diện bắt buộc:

```python
def solve(initial_state, timebound=120):
    """Trả về SokobanState đích có chuỗi parent hợp lệ, hoặc đúng False."""
```

Yêu cầu: không sửa trạng thái đầu vào, không đọc dữ liệu bộ chấm, không truy cập
mạng, không tạo luồng/tiến trình nền. Mỗi test chạy trong một tiến trình mới.

## 2. Cấu trúc repo

| Đường dẫn | Vai trò |
| --- | --- |
| `solution.py` | **Bài nộp.** Solver và hàm `solve()` |
| `sokoban.py`, `state.py` | Framework: trạng thái, luật chơi, hiển thị, 20 map thực hành (`PROBLEMS`) |
| `validation.py` | Kiểm tra đường đi hợp lệ |
| `autograder.py`, `public_tests.py` | Kiểm tra giao diện cơ bản (không phải điểm chính thức) |
| `test_framework.py` | Kiểm tra framework |
| `test_solution.py` | Unit test cho các khối trong `solution.py` |
| `tests/` | Test heuristic, deadlock, tương đương engine |
| `run_map.py` | Chạy solver trên từng map, mỗi map một tiến trình |
| `bench.py` | Đo hàng loạt, ghi kết quả ra JSON |
| `make_table.py` | In bảng so sánh từ các file `runs/*.json` |
| `runs/` | Kết quả đo đã lưu (ví dụ `demo_baseline_120.json`) |
| `check_imports.py` | Kiểm tra `solution.py` chỉ import thư viện chuẩn |
| `NOTES.md`, `REPORT_TEMPLATE.md`, `reports/` | Ghi chú thiết kế và báo cáo |

Framework (`sokoban.py`, `state.py`, `validation.py`, `autograder.py`,
`public_tests.py`, `test_framework.py`) giữ nguyên như đề cho.

## 3. Cài đặt và chạy nhanh

Không cần cài thêm gì ngoài Python 3.10+.

```bash
git clone https://github.com/DucDung0710/Sokoban-Algorithm.git
cd Sokoban-Algorithm
python --version          # >= 3.10
python autograder.py      # kiểm tra giao diện, cần ra 3/3
```

Chạy solver trên map thực hành:

```bash
python run_map.py 1              # map 1
python run_map.py 1 5 7-10       # nhiều map / khoảng map
python run_map.py all -t 10 -q   # cả 20 map, timebound 10 s, không in bàn chơi
python run_map.py 3 -p           # in từng bước lời giải
```

`run_map.py` chạy mỗi map trong một tiến trình riêng với giới hạn cứng
`timebound + 10 s` (giống autograder), rồi kiểm lời giải bằng `validation.py`.
Lưu ý: `run_map.py` đánh số map **từ 1**; trong code và `runs/*.json` map được
đánh số **từ 0** (`PROBLEMS[0]` = map 1 của `run_map.py`).

Dùng trực tiếp trong Python:

```python
from sokoban import PROBLEMS, SokobanState
from solution import solve

s = PROBLEMS[0]
start = SokobanState("START", 0, None, s.width, s.height,
                     s.robots, s.boxes, s.storage, s.obstacles)
goal = solve(start, timebound=30)
if goal is not False:
    print("Số bước:", goal.gval)
    goal.print_path()
```

## 4. Thiết kế solver

Luồng chạy trong bài nộp:

```
solve(initial_state, timebound)
 └─ tiền xử lý map: ô chết tĩnh, khoảng cách đẩy thùng → đích
 └─ anytime_search(deadline = TIME_FRACTION × timebound)
     ├─ pha w = 5   : Weighted A*  → lời giải đầu tiên (nhanh, chưa tối ưu)
     ├─ pha w giảm  : Weighted A*, cắt nhánh bằng chi phí lời giải tốt nhất
     └─ pha w = 1   : A* thường    → nếu chạy xong thì lời giải tối ưu
 └─ trả SokobanState đích tốt nhất, hoặc False
```

### 4.1 Tìm kiếm: Weighted A\* anytime

- Khóa sắp xếp: `f(n) = g(n) + w · h(n)`, với `g` là số bước thật.
- Với `h` admissible, lời giải pha `w` có chi phí không quá `w · C*`.
- Mỗi pha dùng chi phí lời giải tốt nhất hiện có làm cận trên: node có
  `g + h ≥ best` bị bỏ qua. Pha `w = 1` chạy hết mà không tìm được lời giải ngắn
  hơn thì lời giải hiện tại là tối ưu.
- Trạng thái lặp được loại bằng `hashable_state()` và bảng `g` tốt nhất.
- Hai giới hạn an toàn: hết thời gian (`deadline`) hoặc số trạng thái vượt
  `MAX_STATES`. Khi chạm giới hạn, solver trả lời giải tốt nhất của các pha trước.

### 4.2 Heuristic `h2`

- Khoảng cách của mỗi thùng tới mỗi ô đích được tính trước bằng BFS trên **đồ thị
  đẩy thùng** (có tính tường; một thùng chỉ đi được theo hướng mà robot đứng phía
  sau được).
- `h2` = chi phí **ghép cặp thùng–đích tối thiểu** (mỗi đích nhận tối đa một
  thùng), giải bằng quy hoạch động trên bitmask đích. Kết quả được cache theo tập
  thùng.
- `h2` là cận dưới của số lần đẩy, nên là cận dưới của số bước: admissible.
  Đã kiểm vét cạn trên map 0–7 (773.920 trạng thái): admissible và consistent.
- `h1` (Manhattan tới đích gần nhất) và `h3` vẫn có trong code để so sánh. Đo ở
  60 s, `h2` cho tổng số bước tốt hơn `h3` trên map 9–19 nên được giữ làm mặc định.

### 4.3 Phát hiện deadlock

Một trạng thái chứa thùng không thể về đích bị cắt ngay khi sinh ra:

- **Ô chết tĩnh**: ô mà từ đó thùng không đẩy được tới đích nào (góc tường, dọc
  tường không có đích). Tính một lần khi tiền xử lý.
- **Freeze deadlock**: thùng vừa đẩy bị khóa cả trục ngang lẫn dọc (bởi tường
  hoặc thùng khác cũng bị khóa) mà không nằm trên đích.

Deadlock là bắt buộc: khi tắt (ablation), map 17 và 19 không tìm ra lời giải
trong 120 s. Kiểm vét cạn map 0–7 cho thấy không cắt nhầm trạng thái giải được.

### 4.4 Quản lý thời gian và bộ nhớ

- Search dừng ở `TIME_FRACTION × timebound` (mặc định `0.85`, tức 102 s với
  120 s). Phần còn lại dành cho dọn bộ nhớ và trả kết quả (đo được khoảng 2,6 s ở
  map lớn nhất).
- `MAX_STATES = 2.000.000` trạng thái; `MemoryError` được bắt và solver trả lời
  giải tốt nhất hiện có.

### 4.5 Mã thử nghiệm (không chạy trong bài nộp)

`solution.py` còn chứa một **engine nhanh** đang phát triển: trạng thái mã hóa
thành số nguyên (`FastCtx`, `pack`/`unpack`), `successors_fast`, `fast_h`,
`dead_after_push`, `rebuild`. `solve()` **chưa gọi** các hàm này; chúng chỉ được
dùng trong test và benchmark. Tương tự, các khóa `CONFIG["engine"]`,
`CONFIG["sym"]`, `CONFIG["cap"]` chưa có tác dụng với `solve()`.

## 5. Cấu hình

Các hằng số chính ở đầu `solution.py`:

| Tên | Mặc định | Ý nghĩa |
| --- | --- | --- |
| `CONFIG["h"]` | `"h2"` | Heuristic dùng trong search |
| `WEIGHTS` | bắt đầu từ `5`, giảm dần về `1` | Trọng số các pha anytime |
| `TIME_FRACTION` | `0.85` | Phần của `timebound` dành cho search |
| `MAX_STATES` | `2_000_000` | Trần số trạng thái lưu trong một pha |

Không tăng `TIME_FRACTION`: map chậm nhất đã dùng 104,6 / 120 s.

## 6. Kiểm thử

Checklist trước mỗi lần push (khoảng 1,5 phút):

```bash
python autograder.py                                    # 3/3
python -m unittest test_framework test_solution public_tests
python -m unittest discover -s tests -p "*test*.py"
python check_imports.py
```

Khi đổi phần search hoặc heuristic, chạy thêm:

```bash
python run_map.py all -t 10 -q                          # cần 20/20 PASS
```

`tests/` gồm: tính admissible/consistent của heuristic, tính đúng của deadlock
(không cắt nhầm trạng thái giải được), và tương đương giữa engine framework với
engine nhanh.

## 7. Đo hiệu năng

`bench.py` nhận **thư mục chứa `solution.py`** làm tham số đầu, nên so sánh hai
phiên bản rất gọn:

```bash
# cú pháp: bench.py <thư mục> <timebound> <file kết quả> <số tiến trình> [config JSON] [danh sách map]
python bench.py . 120 runs/demo_baseline_120.json 2
python bench.py . 120 runs/cap1m_120.json 2 "{\"MAX_STATES\": 1000000}" 9,10,15,16,17,18,19

python make_table.py runs/demo_baseline_120.json runs/cap1m_120.json
```

Thử thay đổi lớn trên worktree riêng để không động vào bản nộp:

```bash
git worktree add ../Sokoban-fast -b a/fast-engine Demo
python bench.py ../Sokoban-fast 60 runs/fast_60.json 2
python make_table.py runs/frame_60.json runs/fast_60.json
git worktree remove ../Sokoban-fast
```

Nên đo trên cùng một máy, cùng phiên bản Python, cắm sạc, để số liệu so sánh được.

## 8. Kết quả

Trên 20 map thực hành, mỗi map một tiến trình:

| Chỉ số | Giá trị |
| --- | --- |
| Map giải được (timebound 10 s) | **20 / 20**, tổng **713 bước** |
| Map giải được (timebound 120 s) | **20 / 20**, mọi lời giải hợp lệ |
| Map chậm nhất (120 s) | 104,6 s (map 17) |
| RAM đỉnh (120 s) | 1.859 MB (map 17), 1.740 MB (map 18), 1.637 MB (map 19) |

Profile map 12 (20 s): `SokobanState.successors()` của framework chiếm khoảng 54%
thời gian, tạo đối tượng `SokobanState` thêm khoảng 14%; `h2` và deadlock chỉ
khoảng 10% và 6%. Nút thắt là framework, đây là lý do phát triển engine nhanh
(mục 4.5).

Số đo engine nhanh trên nhánh thử nghiệm (chưa nằm trong bài nộp): tốc độ node/giây
×3,1; RAM mỗi trạng thái 1,05 → 0,29 KB; tổng số bước map 9–19 ở 120 s 544 → 529;
RAM đỉnh 2,3 → 1,0 GB.

## 9. Giới hạn đã biết

- **Bộ nhớ**: khoảng 1,9 GB ở map khó. Nếu máy chấm giới hạn RAM và hệ điều hành
  kill tiến trình, lời giải đã tìm ở pha trước sẽ mất.
- **Pha đầu thất bại**: nếu pha `w = 5` chạm giới hạn thời gian hoặc bộ nhớ, solver
  trả `False`. Chưa có pha dự phòng greedy.
- **Anytime không tái dùng công sức**: mỗi pha chạy lại từ đầu (chưa phải ARA\*).
- **`LAST_INFO`** chưa được `solve()` ghi, nên `bench.py` chưa báo được map nào
  đã chứng minh tối ưu.

## 10. Quy ước làm việc nhóm

- Code phát triển trên nhánh `Demo`; `main` chỉ nhận merge khi code đã đóng băng.
  Bản nộp được gắn tag (ví dụ `v1.0-submit`).
- Thêm `.gitattributes` (`* text=auto`, `*.py text eol=lf`) để tránh diff giả do
  CRLF/LF giữa các máy.
- Không sửa framework, không viết cứng lời giải theo map.<!-- TODO: Ghi mục tiêu dự án, Python 3.10+, cách cài framework, lệnh chạy test/bench và quy trình nhóm. -->
