# Mô tả thiết kế solver Sokoban

Tài liệu này dùng để giải thích bài nộp, không có điểm riêng trong thang chấm.
Không bắt buộc một thuật toán, heuristic hay cấu trúc dữ liệu cụ thể.

**Nhóm:** Phạm Anh Trọng (25023416), Lê Đức Huy Anh (25023149), Trịnh Đức Dũng (25023196), Đỗ Quốc Long (25023307).
**Bài nộp:** `solution.py`, hàm `solve(initial_state, timebound=120)`.

**Tóm tắt.** Solver dùng Weighted A\* dạng anytime với heuristic ghép cặp thùng–đích theo khoảng cách đẩy và cắt deadlock (ô chết, freeze). Trên 10 map đã đo với `timebound = 120`:

- Cả 10 map đều có lời giải hợp lệ. Lần chạy lâu nhất là 103,5 s.
- 5 map nhỏ được chứng minh tối ưu, mỗi map dưới 0,2 s.
- Trên 5 map 8×8 có 3 robot, solver luôn có lời giải từ pha đầu rồi rút ngắn dần (ví dụ map 19 từ 82 xuống 73 bước), nhưng chưa chứng minh được tối ưu.

## 1. Thiết kế

### 1.1 Phương pháp

Solver là **Weighted A\* dạng anytime** chạy trên `SokobanState` của framework:

- **Heuristic `h2`:** ghép mỗi thùng với một đích riêng sao cho tổng **khoảng cách đẩy** nhỏ nhất. Khoảng cách đẩy được tính trước một lần cho mỗi map bằng BFS ngược từ từng đích. Bài toán ghép giải bằng quy hoạch động trên bitmask đích khi có tối đa 8 đích, và bằng thuật toán Hungarian khi nhiều hơn. Kết quả được nhớ theo cấu hình thùng.
- **Cắt deadlock:** ô chết tĩnh (ô không đẩy được thùng tới đích nào) và *freeze* (thùng bị khóa cả trục ngang lẫn trục dọc khi chưa nằm trên đích). Sau mỗi lần đẩy chỉ kiểm thùng vừa đẩy và các thùng kề nó.
- **Vòng anytime:** chạy A\* với trọng số giảm dần `5 → 3 → 2 → 1,5 → 1`. Mỗi pha lấy chi phí của lời giải tốt nhất hiện có làm cận trên để cắt nhánh. Khi một trọng số chạm trần bộ nhớ, solver chia đôi khoảng giữa trọng số đó và trọng số nhỏ nhất đã thành công rồi thử lại. Nếu pha đầu thất bại thì chạy một pha cứu hộ với `w = 25`.
- **Gộp đối xứng robot:** các robot giống hệt nhau, nên khóa trạng thái dùng danh sách ô robot đã sắp xếp. Hai trạng thái chỉ khác nhau ở việc robot nào đứng ô nào được coi là một.

### 1.2 Tổ chức chương trình

`solution.py` chia thành các Khu, mỗi Khu do một thành viên phụ trách. Các Khu nói chuyện với nhau qua giao diện hẹp: heuristic là hàm `state → số`, deadlock là hàm `state → bool`, còn tìm kiếm nhận hai hàm này làm tham số.

| Khu | Nội dung | Hàm chính |
|---|---|---|
| 0 | Hằng số, `CONFIG` | `TIME_FRACTION`, `MAX_STATES`, `WEIGHTS` |
| 1 | Tiền xử lý bàn chơi | `build_board`, `precompute_push_dist` |
| 2 | Heuristic | `h1`, `h2`, `h3`, `_assignment_cost` |
| 3 | Deadlock | `is_dead`, `_is_frozen` |
| 4 | Tìm kiếm | `astar`, `anytime_search` |
| 5 | Giao diện với framework | `solve` |

`solve()` đặt hạn dừng tìm kiếm ở 85% `timebound`, tắt GC trong lúc tìm kiếm, và trả `SokobanState` đích (chuỗi `parent` do chính `successors()` của framework tạo) hoặc đúng giá trị `False`.

### 1.3 Lý do lựa chọn

| Quyết định | Lý do |
|---|---|
| A\* thay vì BFS | Chi phí mỗi bước bằng 1 nên BFS cũng cho lời giải tối ưu, nhưng số trạng thái tăng theo hàm mũ. Bàn 8×8 có 3 robot và 6 thùng có khoảng 2·10¹³ cấu hình. |
| Khoảng cách đẩy thay vì Manhattan | Manhattan bỏ qua tường. Khoảng cách đẩy tính đúng hình dạng bàn, lại cho luôn tập ô chết. |
| `h2` (ghép cặp) thay vì `h1` | `h1` cho nhiều thùng cùng nhắm một đích nên đánh giá thấp. `h2` buộc mỗi thùng vào một đích riêng mà vẫn admissible. |
| `h2` thay vì `h3` | `h3` cộng thêm rất ít (khoảng cách tới thùng gần nhất) và phụ thuộc vị trí robot nên không nhớ được theo cấu hình thùng. Khi nhóm đo, tổng số bước không tốt hơn `h2`. |
| Anytime thay vì một trọng số cố định | Bộ chấm tính điểm theo số case giải được rồi mới tới độ dài. `w = 1` trên bàn lớn hết giờ mà không ra lời giải nào. Dãy trọng số giảm dần vừa có lời giải sớm vừa cải thiện dần. |
| Luật deadlock đơn giản | Luật deadlock phải *sound*: báo nhầm là mất lời giải. Ô chết và freeze chứng minh được và kiểm vét cạn được. |
| Dừng ở 85% `timebound` | Sau khi tìm kiếm dừng, Python còn phải giải phóng hàng triệu đối tượng. Lần chạy chậm nhất trong mục 3 trả về sau 103,5 s với `timebound = 120`. |
| Tắt GC khi tìm kiếm | Trạng thái tìm kiếm không tạo vòng tham chiếu, nên bộ thu gom vòng chỉ tốn thời gian quét. GC được bật lại trong `finally`. |

Các phương án đã cân nhắc rồi bỏ: IDA\* (đồ thị Sokoban nhiều chu trình, không có bảng `visited` thì lặp lại công việc rất nhiều), tối ưu số lần đẩy như Sokoban cổ điển (sai mục tiêu vì đề tính cả bước đi bộ), deadlock mạnh hơn như corral (khó chứng minh sound), đa luồng (đề cấm).

## 2. Chất lượng lời giải

**Cơ sở.** Hai giả thiết: `h2` admissible và luật deadlock sound.

- *Admissible:* trong mọi lời giải, mỗi thùng kết thúc ở một đích riêng và mỗi lần đẩy chỉ dời một thùng đi một ô. Vì vậy số lần đẩy ít nhất là chi phí ghép rẻ nhất, tức `h2`. Mà số lần đẩy không vượt tổng số bước, nên `h2 ≤ h*`.
- *Sound:* thùng ở ô chết không bao giờ tới được đích. Thùng bị khóa cả hai trục thì không bao giờ di chuyển được nữa.

Nhóm kiểm hai giả thiết này bằng cách vét cạn toàn bộ không gian trạng thái trên các bàn nhỏ (hơn 770 nghìn trạng thái), so với khoảng cách tối ưu thật: không có vi phạm nào. Từ hai giả thiết suy ra:

1. Phép cắt `g + h ≥ bound` dùng `h` **không nhân trọng số**, nên không bỏ sót lời giải nào ngắn hơn lời giải đang giữ. Qua các pha, chi phí lời giải chỉ giảm.
2. Một pha có trọng số `w` tìm ra đích thì chi phí của đích đó không quá `w · C*`, với `C*` là chi phí tối ưu.
3. Một pha chạy cạn hàng đợi thì không còn lời giải ngắn hơn, nên lời giải đang giữ là **tối ưu**. Solver chỉ tuyên bố tối ưu trong trường hợp này (`LAST_INFO["proved"] = True`).

**Tuyên bố cụ thể** (số liệu ở mục 3):

- Map 0–4: **tối ưu đã chứng minh**.
- Map 15–19: **chưa chứng minh tối ưu**. Kết hợp `h2` ở trạng thái đầu với hệ quả 2 cho khoảng chứa chi phí tối ưu:

| Map | Lời giải | Tìm ra ở pha | Cận dưới của `C*` | `C*` nằm trong |
|---|---:|---:|---:|---|
| 15 | 65 | w = 1,875 | 37 (`h2` lúc đầu) | [37, 65] |
| 16 | 52 | w = 1,875 | 29 (`h2` lúc đầu) | [29, 52] |
| 17 | 96 | w = 2,125 | 46 (= ⌈96 / 2,125⌉) | [46, 96] |
| 18 | 91 | w = 2,75 | 47 (`h2` lúc đầu) | [47, 91] |
| 19 | 73 | w = 2 | 40 (`h2` lúc đầu) | [40, 73] |

**Giới hạn.** Các khoảng trên rộng vì `h2` chỉ đếm số lần đẩy. Ở 9/10 map đã chạy, số lần đẩy trong lời giải tìm được **bằng đúng** `h2` lúc đầu, nghĩa là phần chênh giữa lời giải và cận dưới gần như hoàn toàn là bước đi bộ (xem mục 3 và 4).

## 3. Thực nghiệm

**Điều kiện đo:**
- Python 3.13, máy 2 nhân, 7 GB RAM.
- Mỗi map chạy trong một tiến trình riêng với `timebound = 120`, hai tiến trình chạy song song.
- Mọi lời giải được kiểm lại bằng `validate_solution` của framework.
- Map đánh số từ 0 theo thứ tự `PROBLEMS` trong `sokoban.py`.

Nhóm chọn hai nhóm map: 5 map đầu (bàn nhỏ) để thấy ưu điểm, và 5 map lớn nhất (bàn 8×8, 3 robot, 4–6 thùng) để thấy giới hạn.

| Map | Bàn | Robot | Thùng | Giải được | Số bước | Đẩy / đi bộ | Thời gian (s) | Tối ưu | RAM đỉnh |
|---:|:---:|---:|---:|:---:|---:|:---:|---:|:---:|---:|
| 0 | 5×5 | 2 | 4 | ✓ | 16 | 8 / 8 | 0,17 | đã chứng minh | 15 MB |
| 1 | 5×5 | 2 | 1 | ✓ | 4 | 2 / 2 | 0,00 | đã chứng minh | 12 MB |
| 2 | 5×5 | 1 | 4 | ✓ | 21 | 8 / 13 | 0,03 | đã chứng minh | 13 MB |
| 3 | 6×4 | 2 | 2 | ✓ | 10 | 3 / 7 | 0,03 | đã chứng minh | 13 MB |
| 4 | 6×4 | 2 | 1 | ✓ | 8 | 3 / 5 | 0,01 | đã chứng minh | 12 MB |
| 15 | 8×8 | 3 | 5 | ✓ | 65 | 37 / 28 | 103,5 | chưa | 1 169 MB |
| 16 | 8×8 | 3 | 4 | ✓ | 52 | 29 / 23 | 78,4 | chưa | 903 MB |
| 17 | 8×8 | 3 | 6 | ✓ | 96 | 47 / 49 | 63,7 | chưa | 1 269 MB |
| 18 | 8×8 | 3 | 6 | ✓ | 91 | 47 / 44 | 102,3 | chưa | 1 236 MB |
| 19 | 8×8 | 3 | 6 | ✓ | 73 | 40 / 33 | 89,2 | chưa | 1 209 MB |

**Diễn biến các pha trên map lớn** (chi phí lời giải sau mỗi pha; *mem* = chạm trần bộ nhớ, *hết giờ* = timeout):

| Map | Các pha |
|---:|---|
| 15 | w5: 72 → w3: 71 → w2: 69 → w1,5: mem → w1,75: mem → **w1,875: 65** → w1,8125: hết giờ |
| 16 | w5: 56 → w3: 55 → w2: 54 → w1,5: mem → w1,75: mem → **w1,875: 52** → w1,8125: mem |
| 17 | w5: 102 → w3: 100 → w2: mem → w2,5: 98 → w2,25: 97 → **w2,125: 96** → w2,0625: mem |
| 18 | w5: 94 → w3: 92 → w2: mem → w2,5: mem → **w2,75: 91** → w2,625: mem → w2,6875: hết giờ |
| 19 | w5: 82 → w3: 75 → **w2: 73** → w1,5 / 1,75 / 1,875 / 1,9375: mem |

**Ưu điểm thấy được:**

- **Bàn nhỏ giải tối ưu gần như tức thì:** map 0–4 đều được chứng minh tối ưu trong dưới 0,2 s, dùng dưới 16 MB.
- **Luôn có lời giải sớm:** pha `w = 5` ra lời giải ở cả 10 map. Sau đó lời giải chỉ ngắn dần, ví dụ map 19 từ 82 xuống 73, map 17 từ 102 xuống 96.
- **Chia đôi trọng số có hiệu quả thật:** 4/5 map lớn (15, 16, 17, 18) có lời giải cuối tìm ra ở trọng số sinh ra từ việc chia đôi (1,875, 2,125, 2,75). Ví dụ map 15 giảm từ 69 xuống 65 nhờ thử w = 1,875 sau khi w = 1,5 và 1,75 chạm trần.
- **Số lần đẩy gần như tối thiểu:** ở 9/10 map, số lần đẩy trong lời giải bằng đúng cận dưới `h2`.
- **Đúng hạn:** lần chạy lâu nhất là 103,5 s trên ngân sách 120 s.

**Yếu điểm thấy được:**

- **Không chứng minh được tối ưu trên bàn 8×8:** khoảng chứa `C*` còn rộng tới 43–52% chi phí lời giải.
- **Bộ nhớ, không phải thời gian, là thứ chặn tìm kiếm:** trong 35 pha chạy trên map lớn, 19 pha ra lời giải, 14 pha dừng vì chạm trần 1 000 000 trạng thái và chỉ 2 pha hết giờ (khoảng 0,9–1,3 KB mỗi trạng thái, RAM đỉnh tới 1,27 GB). Map 16, 17, 19 dừng sớm 24 s, 38 s, 13 s trước hạn vì mọi trọng số còn lại đều chạm trần.
- **Heuristic không thấy bước đi bộ:** 43–51% số bước trong lời giải map lớn là đi bộ, mà `h2` không đếm phần này.

## 4. Hạn chế và nguồn tham khảo

### 4.1 Hạn chế và hướng khắc phục

Xếp theo mức có thể khắc phục:

| # | Hạn chế | Bằng chứng (mục 3) | Khắc phục được không | Hướng khắc phục |
|---|---|---|---|---|
| 1 | **Mỗi trạng thái tốn ~1 KB bộ nhớ** vì lưu nguyên đối tượng `SokobanState`, nên trần 1 triệu trạng thái bị chạm trước khi hết giờ | 14/35 pha dừng vì trần bộ nhớ; map 16, 17, 19 bỏ phí 13–38 s | **Có**, không phải đổi thuật toán | Trong lúc tìm kiếm chỉ lưu khóa gọn (bitmask thùng + ô robot) và con trỏ cha. Khi tìm ra đích thì dựng lại chuỗi `SokobanState` bằng `successors()` của framework, nên đường đi vẫn hợp lệ. Cùng lượng RAM chứa được nhiều trạng thái hơn, đủ để chạy trọng số nhỏ hơn. Cần kiểm tương đương với cách hiện tại trước khi thay. |
| 2 | **RAM đỉnh 1,27 GB**: nếu máy chấm giới hạn RAM chặt và hệ điều hành dừng hẳn tiến trình (không ném `MemoryError`), lời giải đã tìm ở các pha trước sẽ mất | RAM đỉnh 903–1 269 MB ở map lớn | **Có**, nhanh | Hạ `MAX_STATES` nếu biết RAM máy chấm. Khắc phục #1 cũng giải quyết luôn phần lớn vấn đề này. |
| 3 | **Heuristic bỏ qua bước đi bộ**, nên cận dưới lỏng và pha `w = 1` khó chứng minh tối ưu | Đi bộ chiếm 43–51% số bước; khoảng chứa `C*` rộng 43–52% | **Một phần** | Cộng thêm cận dưới quãng đi bộ giữa các lần đẩy. `h3` hiện chỉ cộng quãng tới thùng gần nhất trước lần đẩy đầu nên tác dụng nhỏ. Một cận mạnh hơn mà vẫn admissible với nhiều robot là khó; cần chứng minh và kiểm vét cạn lại. |
| 4 | Deadlock chưa đầy đủ: chưa có corral deadlock, chưa phát hiện robot bị nhốt | Không đo riêng | Khó | Thêm luật mới thì phải chứng minh sound; rủi ro mất lời giải lớn hơn lợi ích trong thời gian còn lại. |

Nhóm ưu tiên **#1**: bằng chứng rõ (thời gian còn thừa vì chạm trần bộ nhớ), không đụng tới tính đúng của heuristic và deadlock, có cách kiểm tương đương trực tiếp, và giải quyết luôn phần lớn #2.

### 4.2 Tài liệu tham khảo

- S. Russell, P. Norvig. *Artificial Intelligence: A Modern Approach*, 4th ed., Chương 3: A\*, heuristic admissible và consistent, heuristic từ bài toán nới lỏng.
- Slide môn Trí tuệ nhân tạo, UET: Informed Search, Weighted A\*.
- M. Likhachev, G. Gordon, S. Thrun. ARA\*: Anytime A\* with provable bounds on sub-optimality. NIPS 2003.
- R. Zhou, E. Hansen. Multiple sequence alignment using anytime A\*. AAAI 2002.
- A. Junghanns, J. Schaeffer. Sokoban: Enhancing general single-agent search methods using domain knowledge. *Artificial Intelligence* 129, 2001 (deadlock, ghép cặp thùng–đích).
- H. W. Kuhn. The Hungarian method for the assignment problem. *Naval Research Logistics Quarterly* 2, 1955.

### 4.3 Mã nguồn

- Framework do giảng viên cung cấp: `sokoban.py`, `state.py`, `validation.py`, `autograder.py`. Không sửa.
- Phần còn lại của `solution.py` do nhóm tự viết. Chỉ dùng thư viện chuẩn Python (`heapq`, `collections`, `itertools`, `time`, `gc`).

### 4.4 Công cụ AI đã sử dụng

- GitHub Copilot (VS Code): hỗ trợ chỉnh sửa mã, test và tài liệu phần heuristic, deadlock.
- Claude (Anthropic): review code, đề xuất tối ưu, hỗ trợ đo đạc và soạn tài liệu (README, báo cáo).
