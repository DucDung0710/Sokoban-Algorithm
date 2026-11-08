# Báo cáo nhóm — Xây dựng solver Sokoban nhiều robot

**Môn:** Trí tuệ nhân tạo — Trường Đại học Công nghệ, ĐHQGHN
**Repo:** `github.com/DucDung0710/Sokoban-Algorithm`
**Bài nộp:** `solution.py`, hàm `solve(initial_state, timebound=120)`

---

## Mục lục

1. [Giới thiệu](#1-giới-thiệu)
2. [Nhóm và cách tổ chức](#2-nhóm-và-cách-tổ-chức)
3. [Phân tích bài toán: những câu hỏi đặt ra từ đầu](#3-phân-tích-bài-toán-những-câu-hỏi-đặt-ra-từ-đầu)
4. [Quá trình tiến hóa của thiết kế](#4-quá-trình-tiến-hóa-của-thiết-kế)
5. [Kiến trúc cuối cùng](#5-kiến-trúc-cuối-cùng)
6. [Heuristic: suy ra từ bài toán nới lỏng](#6-heuristic-suy-ra-từ-bài-toán-nới-lỏng)
7. [Deadlock: cắt nhánh mà không mất lời giải](#7-deadlock-cắt-nhánh-mà-không-mất-lời-giải)
8. [Tìm kiếm anytime và quản lý tài nguyên](#8-tìm-kiếm-anytime-và-quản-lý-tài-nguyên)
9. [Bảo đảm tính đúng](#9-bảo-đảm-tính-đúng)
10. [Các quyết định đã cân nhắc và loại bỏ](#10-các-quyết-định-đã-cân-nhắc-và-loại-bỏ)
11. [Hạn chế và hướng phát triển](#11-hạn-chế-và-hướng-phát-triển)
12. [Bài học rút ra](#12-bài-học-rút-ra)
13. [Tài liệu tham khảo và công cụ](#13-tài-liệu-tham-khảo-và-công-cụ)

---

## 1. Giới thiệu

Đề bài yêu cầu viết hàm `solve` nhận một trạng thái Sokoban nhiều robot và trả về
trạng thái đích, sao cho lần ngược chuỗi `parent` cho ra một đường đi hợp lệ.
Đề không bắt buộc thuật toán, heuristic hay cấu trúc dữ liệu nào, nên phần lớn
công sức của nhóm nằm ở việc **chọn và lý giải** các quyết định thiết kế.

Báo cáo này trình bày cách nhóm tổ chức công việc và chuỗi suy luận dẫn tới
solver cuối cùng: mỗi thành phần ra đời để giải quyết vấn đề nào, vì sao chọn
cách này mà không chọn cách khác, và làm sao nhóm biết nó đúng.

Solver cuối cùng là **Weighted A\* dạng anytime** trên không gian trạng thái của
framework, với heuristic **ghép cặp thùng–đích theo khoảng cách đẩy** (`h2`) và bộ
phát hiện deadlock gồm **ô chết tĩnh + freeze**.

## 2. Nhóm và cách tổ chức

### 2.1 Phân vai

Nhóm có 4 thành viên, không có trưởng nhóm; trình độ không đồng đều nên việc được
chia theo **thành phần kỹ thuật độc lập**, mỗi người sở hữu một vùng trong
`solution.py`:

| Vai | Thành viên | Khu trong `solution.py` | Trách nhiệm |
| --- | --- | --- | --- |
| A | Đông | Khu 0, 4, 5 | Hằng số và `CONFIG`; thuật toán tìm kiếm, anytime, quản lý thời gian và bộ nhớ; `solve()`; tích hợp, merge; công cụ đo `bench.py`; tổng hợp báo cáo |
| B | *(điền tên)* | Khu 1 | Biểu diễn bàn chơi (`Board`), tiền xử lý, `successors_fast`, `rebuild`; README |
| C | *(điền tên)* | Khu 2, 3 | Heuristic `h0`–`h3`; phát hiện deadlock; `NOTES.md`, slide phần heuristic |
| D | *(điền tên)* | `tests/` | Kiểm thử đơn vị, kiểm vét cạn, `check_imports.py` |

Ý tưởng của cách chia: các khu giao tiếp với nhau qua **giao diện hẹp** —
heuristic là hàm `state → số`, deadlock là hàm `state → bool`, tìm kiếm nhận hai
hàm đó làm tham số. Nhờ vậy C có thể đổi heuristic, D có thể kiểm heuristic độc
lập, mà A không phải sửa thuật toán tìm kiếm.

### 2.2 Quy trình làm việc

- **Nhánh:** code phát triển trên `Demo`; `main` chỉ nhận merge khi code đã đóng
  băng. Thử nghiệm lớn (engine mới) làm trên nhánh hoặc worktree riêng
  (`a/khu4`), chỉ merge khi qua toàn bộ test và đo lại.
- **Checklist trước khi push:** `autograder.py`, test framework, `tests/`,
  `check_imports.py`; thêm `run_map.py` khi sửa phần tìm kiếm.
- **Đo trước khi quyết định:** mọi thay đổi về hiệu năng đều được so A/B bằng
  `bench.py` (mỗi map một tiến trình, giống bộ chấm) và `make_table.py`. `bench.py`
  nhận thư mục chứa `solution.py` và tham số ghi đè `CONFIG`, nên so hai phiên bản
  hay tắt/bật một thành phần không cần sửa code.
- **Một nguồn sự thật cho cấu hình:** `CONFIG`, `WEIGHTS`, `MAX_STATES` nằm ở đầu
  file; các hàm đọc giá trị lúc gọi chứ không đóng băng làm tham số mặc định.

### 2.3 Tiến trình

| Giai đoạn | Thời gian | Nội dung chính |
| --- | --- | --- |
| Khởi động | 04–11/10 | Dựng repo, chia khu; BFS cơ bản làm mốc (v0) |
| Heuristic | 13/10 | Khoảng cách đẩy ngược từ đích (reverse push distance) |
| Lõi solver | 18–19/10 | Heuristic + deadlock hoàn chỉnh; A\* có trọng số; vòng anytime; `successors_fast`, `rebuild` |
| Tích hợp | 24–27/10 | Hoàn thiện phần C; unit test heuristic; khóa số nguyên (Khu 4a), test tương đương, `bench.py` |
| Kiểm chứng | 30/10–02/11 | Test deadlock, kiểm vét cạn, công cụ thống kê, dọn tham số và cache |
| Hoàn thiện | 03–08/11 | Dọn code chết, `LAST_INFO`, tài liệu, báo cáo, slide |

## 3. Phân tích bài toán: những câu hỏi đặt ra từ đầu

Trước khi viết code, nhóm trả lời một số câu hỏi. Mỗi câu trả lời trở thành một
ràng buộc thiết kế.

**Q1. Tối ưu cái gì?** Chi phí là **tổng số bước của mọi robot**, tính cả bước đi
bộ, không chỉ số lần đẩy như Sokoban cổ điển.
→ Mọi bước có chi phí 1, nên đây là bài toán đường ngắn nhất trên đồ thị đơn vị:
A\* với heuristic admissible cho lời giải tối ưu. Nhưng heuristic chỉ đếm số lần
đẩy sẽ bỏ sót phần đi bộ, nên sẽ "yếu" hơn so với Sokoban cổ điển.

**Q2. Không gian trạng thái lớn đến đâu?** Trạng thái là `(vị trí các robot, tập
thùng)`. Bàn 8×8 có 3 robot và 6 thùng có cỡ `C(64, 6) · 64³ ≈ 2 · 10¹³` cấu hình.
Nhiều robot làm số hành động mỗi bước là `4R` và sinh nhiều trạng thái tương đương
(hai robot đổi chỗ cho nhau).
→ Tìm kiếm mù không khả thi. Cần heuristic mạnh và cắt tỉa trạng thái chết.

**Q3. Bộ chấm thưởng cái gì?** Điểm dựa trên số test giải được và độ dài đường đi;
timeout hoặc crash là trượt.
→ **Có một lời giải bất kỳ quan trọng hơn có lời giải tối ưu.** Thiết kế phải ưu
tiên "luôn trả về được gì đó", rồi mới cải thiện chất lượng trong thời gian còn lại.

**Q4. Giới hạn thực thi là gì?** Mỗi test có ngân sách `timebound` (mặc định
120 s), chạy trong một tiến trình mới, chỉ dùng thư viện chuẩn, không đa luồng,
không lưu dữ liệu giữa các test. Python thuần chậm và tốn bộ nhớ cho mỗi đối tượng.
→ Không thể dựa vào tiền tính offline hay song song. Phải tự quản lý thời gian
(dừng trước hạn) và bộ nhớ (trần số trạng thái).

**Q5. Hợp đồng trả về khắt khe đến đâu?** Kết quả phải là chuỗi `SokobanState` hợp
lệ (`validation.py` kiểm lại từng bước bằng `successors()`), trả đúng `False` nếu
không tìm được, và không được sửa trạng thái đầu vào.
→ Cách an toàn nhất là để chính `successors()` của framework tạo ra các trạng thái
trên đường đi. Mọi tối ưu bằng biểu diễn riêng đều phải có bước dựng lại và được
kiểm tương đương.

Từ năm câu trả lời trên, nhóm đặt ra bốn tiêu chí theo thứ tự ưu tiên:

1. **Đúng:** không bao giờ trả lời giải sai, không crash, không sửa đầu vào.
2. **Bền:** luôn trả về kết quả trong hạn, kể cả trên máy chậm.
3. **Có lời giải:** giải được càng nhiều map càng tốt.
4. **Ngắn:** trong thời gian còn lại, rút ngắn lời giải.

## 4. Quá trình tiến hóa của thiết kế

Solver không được thiết kế xong ngay từ đầu. Mỗi phiên bản sinh ra từ giới hạn
quan sát được ở phiên bản trước.

### v0 — BFS

**Ý tưởng:** chi phí đơn vị nên BFS cho lời giải tối ưu và dễ viết đúng.
**Vai trò:** mốc so sánh và bộ kiểm chứng cho các phiên bản sau.
**Giới hạn quan sát được:** chỉ giải được bàn rất nhỏ. Số trạng thái tăng theo hàm
mũ với độ sâu lời giải; bộ nhớ cạn trước thời gian.
**Kết luận:** cần **định hướng** tìm kiếm → A\* với heuristic.

### v1 — A\* với khoảng cách đẩy

**Câu hỏi:** heuristic nào vừa admissible vừa đủ mạnh?
Manhattan từ thùng tới đích gần nhất admissible nhưng bỏ qua tường: thùng sau góc
tường có Manhattan nhỏ mà thực tế phải đi vòng xa hoặc không bao giờ tới.
**Ý tưởng:** tính trước **khoảng cách đẩy thật** trên bàn chỉ có tường bằng BFS
ngược từ mỗi đích (mục 6). Một lần tiền xử lý cho mỗi map, tra bảng O(1) mỗi node.
**Sản phẩm phụ:** ô nào không đẩy được tới đích nào là **ô chết** → nảy ra ý tưởng
cắt deadlock.

### v2 — Ghép cặp và deadlock

**Vấn đề của `h1`** (cộng khoảng cách tới đích gần nhất): nhiều thùng có thể cùng
"nhắm" một đích, nên đánh giá thấp nặng khi thùng dồn về một phía.
**Ý tưởng:** buộc mỗi thùng vào một đích riêng → bài toán gán (assignment) → `h2`.
**Vấn đề thứ hai:** A\* vẫn tốn phần lớn công sức vào các nhánh đã chết (thùng kẹt
góc, hai thùng chắn nhau dọc tường) mà heuristic không nhận ra.
**Ý tưởng:** phát hiện deadlock ngay khi đẩy và cắt nhánh (mục 7).

### v3 — Weighted A\* và anytime

**Vấn đề:** A\* thuần (`w = 1`) chứng minh được tối ưu trên bàn nhỏ, nhưng trên bàn
8×8 nhiều robot thì hết thời gian mà **không có lời giải nào** → 0 điểm cho test đó.
Điều này mâu thuẫn trực tiếp với tiêu chí 3.
**Ý tưởng:** nhân `h` với `w > 1` làm tìm kiếm "tham" hơn, tìm ra lời giải nhanh
với chất lượng bảo đảm không quá `w · C*`. Nhưng chọn một `w` cố định thì phải đánh
đổi: `w` lớn nhanh nhưng dài, `w` nhỏ ngắn nhưng có thể không kịp.
**Quyết định:** không chọn một `w` mà chạy **dãy giảm dần** `5 → 3 → 2 → 1,5 → 1`,
mỗi pha dùng lời giải tốt nhất làm cận trên để cắt nhánh. Pha đầu bảo đảm có lời
giải sớm, các pha sau cải thiện dần, pha cuối có thể chứng minh tối ưu.

### v4 — Kỹ thuật thời gian và bộ nhớ

**Vấn đề:** tìm kiếm có thể dùng hết `timebound`, nhưng sau khi dừng Python còn
phải giải phóng hàng triệu đối tượng, mất thêm vài giây → nguy cơ vượt hạn dù đã
có lời giải. Ở bàn lớn, RAM tăng tới gần 2 GB.
**Quyết định:** dừng tìm kiếm ở 85% ngân sách; trần `MAX_STATES`; bắt
`MemoryError` và trả lời giải tốt nhất hiện có; tắt GC trong lúc tìm kiếm (mục 8).

### v5 — Engine trên khóa số nguyên (thử nghiệm, chưa đưa vào bài nộp)

**Quan sát từ profile:** hơn một nửa thời gian nằm ở `successors()` và việc khởi
tạo `SokobanState` của framework; heuristic và deadlock chỉ chiếm phần nhỏ. Tối ưu
heuristic thêm nữa không giải quyết được nút thắt.
**Ý tưởng:** biểu diễn trạng thái bằng **một số nguyên** (bitmask thùng ghép với ô
của từng robot), sinh trạng thái kế bằng phép bit (`successors_fast`), và chỉ dựng
lại chuỗi `SokobanState` một lần khi trả kết quả (`rebuild`). Kèm theo: coi các
robot như nhau khi so khóa để gộp các trạng thái đối xứng.
**Trạng thái:** đã có Khu 1, Khu 4a và test tương đương (`tests/test_search.py`);
đo trên nhánh riêng cho thấy nhanh và nhẹ hơn rõ rệt. Nhóm **chủ động không đưa vào
bài nộp** vì phần còn thiếu (A\* trên khóa số nguyên, trích hành động) sẽ phải viết
sát hạn, trong khi engine framework đã đúng và ổn định. Đây là áp dụng tiêu chí 1–2
trước tiêu chí 4 (xem mục 10).

## 5. Kiến trúc cuối cùng

```
solve(initial_state, timebound)                         Khu 5 (A)
 ├─ build_board(): đánh số ô p = y·W + x, bảng ô kề,     Khu 1 (B)
 │   precompute_push_dist(): khoảng cách đẩy, ô chết
 ├─ heuristic = h2(·, board)                              Khu 2 (C)
 ├─ deadlock  = is_dead(·, board)                         Khu 3 (C, D)
 ├─ gc.disable()
 └─ anytime_search(deadline = 0,85 · timebound)           Khu 4b (A)
      for w in (5, 3, 2, 1.5, 1):
          astar(w, bound = chi phí lời giải tốt nhất)
 → ghi LAST_INFO = {proved, phases}; trả goal hoặc False
```

| Hằng số | Giá trị | Vai trò |
| --- | --- | --- |
| `CONFIG["h"]` | `"h2"` | Heuristic (`h0` / `h1` / `h2` / `h3`) |
| `CONFIG["dead"]` | `"freeze"` | Mức deadlock (`none` / `cells` / `freeze`) |
| `CONFIG["h_cache_max"]` | 100 000 | Trần cache heuristic |
| `WEIGHTS` | (5, 3, 2, 1,5, 1) | Trọng số các pha anytime |
| `TIME_FRACTION` | 0,85 | Phần ngân sách dành cho tìm kiếm |
| `CHECK_EVERY` | 500 | Số lần lấy node giữa hai lần xem đồng hồ |
| `MAX_STATES` | 2 000 000 | Trần số trạng thái mỗi pha |

Các nguyên tắc xuyên suốt:

- **Tách cơ chế khỏi chính sách.** `astar` không biết heuristic hay deadlock cụ thể
  là gì; `solve` chọn chúng theo `CONFIG`. Thử một heuristic mới chỉ cần thêm vào
  bảng `HEURISTICS`.
- **Tiền xử lý một lần, tra bảng nhiều lần.** Mọi thứ chỉ phụ thuộc vào bàn chơi
  (ô kề, khoảng cách đẩy, ô chết) được tính trong `build_board`.
- **Cache theo đúng biến phụ thuộc.** `h2` chỉ phụ thuộc vị trí thùng, còn robot di
  chuyển thường xuyên hơn thùng rất nhiều, nên cache `h2` theo bitmask thùng có tỉ
  lệ trúng cao.

## 6. Heuristic: suy ra từ bài toán nới lỏng

Nguyên tắc nhóm dùng (AIMA §3.6): chi phí tối ưu của một **bài toán nới lỏng**
(bỏ bớt ràng buộc) là heuristic admissible cho bài toán gốc, vì mọi lời giải của
bài gốc vẫn là lời giải của bài nới lỏng. Mỗi heuristic dưới đây ứng với một cách
nới lỏng khác nhau.

### 6.1 Khoảng cách đẩy

`precompute_push_dist` chạy BFS ngược từ mỗi đích trên **đồ thị đẩy**: thùng đi từ
ô `a` sang ô kề `b` nếu cả `b` lẫn ô robot đứng đẩy (phía sau `a`) đều là sàn.

**Nới lỏng:** bỏ ràng buộc "thùng khác chắn đường" và "robot phải đi tới được chỗ
đứng đẩy". Kết quả `pdc[g][p]` là số lần đẩy ít nhất để đưa thùng từ ô `p` tới
đích `g`. Tính ngược từ đích (thay vì xuôi từ thùng) để một lần BFS cho khoảng cách
từ **mọi** ô tới đích đó.

### 6.2 Các heuristic

| Tên | Công thức | Nới lỏng thêm |
| --- | --- | --- |
| `h0` | 0 | Bỏ hết; A\* thành UCS, dùng làm mốc |
| `h1` | `Σ pd_min[box]` | Cho phép nhiều thùng cùng về một đích |
| `h2` | `min_π Σ pdc[π(box)][box]` với `π` gán mỗi thùng một đích riêng | Giữ ràng buộc đích riêng — **mặc định** |
| `h3` | `h2 + max(0, d_min − 1)` | Cộng cận dưới bước đi bộ trước lần đẩy đầu |

`h2` giải bài toán gán bằng **DP trên bitmask đích** khi có ≤ 8 đích và bằng
**thuật toán Hungarian** khi nhiều hơn. Lý do chọn hai cách: với ít đích, DP đơn
giản và nhanh hơn trên thực tế; Hungarian có độ phức tạp đa thức nên an toàn cho
bàn lớn hơn ở bộ test ẩn.

### 6.3 Vì sao admissible

Mỗi lần đẩy di chuyển đúng một thùng qua một cạnh đồ thị đẩy. Trong lời giải tối
ưu, các thùng kết thúc ở các đích khác nhau, tạo thành một phép gán `π*`, và thùng
`i` bị đẩy ít nhất `pdc[π*(i)][i]` lần. Do đó

`h2 ≤ Σ pdc[π*(i)][i] ≤ số lần đẩy ≤ tổng số bước = h*`.

`h1 ≤ h2` vì bỏ ràng buộc đích riêng. Với `h3`, trước lần đẩy đầu tiên phải có
robot đi tới cạnh một thùng, tốn ít nhất `d_min − 1` bước đi bộ, phần này không
được `h2` đếm nên cộng vào vẫn là cận dưới.

### 6.4 Vì sao consistent

Cần `h(s) ≤ 1 + h(s')` cho mọi bước `s → s'`. Bước đi bộ không đổi thùng nên `h2`
giữ nguyên. Bước đẩy dời một thùng qua một cạnh, khoảng cách đẩy của nó giảm nhiều
nhất 1; lấy phép gán tối ưu của `s'` áp cho `s` thì chi phí tăng nhiều nhất 1.
Consistency quan trọng vì nó bảo đảm phép cắt `g + h ≥ bound` ở mục 8 an toàn.

### 6.5 Giới hạn nhận thức được

Mọi heuristic trên đều **đếm số lần đẩy**, trong khi đề tính cả bước đi bộ. Trên
bàn lớn nhiều robot, phần đi bộ chiếm tỉ lệ đáng kể, nên `h` cách xa `h*`. Đây là
lý do gốc khiến pha `w = 1` khó chứng minh tối ưu trên bàn lớn, và là hướng cải
thiện quan trọng nhất (mục 11).

## 7. Deadlock: cắt nhánh mà không mất lời giải

**Nguyên tắc thiết kế:** detector phải **sound** — chỉ báo deadlock khi chắc chắn
trạng thái không giải được. Bỏ sót deadlock chỉ làm tốn thêm node; báo nhầm thì
mất lời giải và phá tính tối ưu. Vì vậy nhóm chọn các luật đơn giản, chứng minh
được, thay vì luật mạnh nhưng khó kiểm.

### 7.1 Ô chết tĩnh

Ô `p` có `pd_min[p] = ∞` thì thùng ở đó không bao giờ tới đích, kể cả trên bàn chỉ
có tường. Tính sẵn trong `build_board`.

### 7.2 Freeze deadlock

Một thùng bị khóa theo một trục nếu:

1. một bên trục là tường hoặc ngoài bàn; hoặc
2. cả hai bên là ô chết; hoặc
3. một bên là thùng cũng bị khóa (kiểm đệ quy, tập `seen` chặn vòng lặp).

Thùng bị khóa cả trục ngang lẫn dọc thì vĩnh viễn không di chuyển được; nếu nó không
nằm trên đích thì trạng thái là deadlock. Luật này bắt được các mẫu ô chết không
bắt được, ví dụ hai thùng sát nhau dọc tường.

### 7.3 Chi phí kiểm tra

Một lần đẩy chỉ thay đổi một thùng, nên chỉ có thể tạo freeze mới quanh thùng đó.
`is_dead` chỉ kiểm thùng vừa đẩy và các thùng kề; bước đi bộ trả `False` ngay.
Kiểm tra cục bộ giữ chi phí gần như không đổi theo kích thước bàn.

### 7.4 Một phát hiện khi đo

Khi tắt/bật từng mức, nhóm thấy mức `cells` cho kết quả **y hệt** `none`: thùng
trên ô chết có `pdc = ∞` tới mọi đích nên `h2` đã trả `∞` và trạng thái bị cắt ở
bước kiểm heuristic. Kiểm ô chết riêng là thừa khi dùng `h2`; giá trị thật của
khu deadlock nằm ở **freeze**. Nhóm vẫn giữ kiểm ô chết vì nó rẻ và giúp các
heuristic yếu hơn (`h0`) hoạt động đúng khi chạy so sánh.

## 8. Tìm kiếm anytime và quản lý tài nguyên

### 8.1 Weighted A\* có cận (`astar`)

- Hàng đợi ưu tiên theo `(g + w·h, −g, thứ tự sinh)`: khi `f` bằng nhau, ưu tiên
  node sâu hơn (gần đích hơn).
- `best_g[key]` giữ `g` tốt nhất của mỗi cấu hình. Node con chỉ được đẩy vào hàng
  đợi nếu `g` tốt hơn hẳn; entry cũ bị bỏ khi lấy ra (lazy deletion), cho phép mở
  lại node khi tìm được đường ngắn hơn.
- Kiểm đích khi **lấy ra**, không phải khi sinh, vì đó là điều kiện để A\* tối ưu.
- Thứ tự cắt với mỗi node con, từ rẻ đến đắt: đã có `g` tốt hơn → deadlock →
  `h = ∞` → `g + h ≥ bound`.

### 8.2 Vòng anytime (`anytime_search`)

```
best ← None; bound ← ∞
for w in (5, 3, 2, 1.5, 1):
    goal, stats ← astar(w, bound)
    if goal:   best ← goal; bound ← goal.gval
               if w = 1: proved ← True; break
    elif stats.reason = "exhausted": proved ← True; break
    elif stats.reason ∈ {timeout, memory_cap}: break
return best
```

Điểm mấu chốt là phép cắt `g + h ≥ bound` dùng `h` **không nhân trọng số**. Vì `h`
admissible và deadlock sound, mọi lời giải đi qua node bị cắt đều dài ít nhất
`bound`. Từ đó:

- Mỗi pha chỉ tìm lời giải **ngắn hơn** lời giải đang giữ, nên chất lượng chỉ tăng.
- Pha nào chạy cạn hàng đợi thì **chứng minh** không có lời giải ngắn hơn: lời giải
  đang giữ là tối ưu, dừng sớm để không lãng phí thời gian.
- Pha `w = 1` tìm ra đích thì đó là lời giải tối ưu.

Cờ `proved` và thống kê từng pha được ghi vào `LAST_INFO` để công cụ đo đọc lại.

**Vì sao dãy (5, 3, 2, 1,5, 1)?** Mỗi lần giảm `w`, số node cần mở tăng khoảng
một bậc độ lớn. Dãy giảm dần theo cấp số gần nhân giúp mỗi pha tốn nhiều hơn pha
trước nhưng không nhảy quá xa; `w = 5` đủ "tham" để pha đầu gần như luôn ra lời
giải nhanh.

### 8.3 Thời gian

- Tìm kiếm dừng ở `0,85 · timebound`. 15% còn lại dành cho việc giải phóng bộ nhớ
  (Python phải hủy hàng triệu đối tượng khi các cấu trúc tìm kiếm ra khỏi phạm vi)
  và trả kết quả.
- Đồng hồ chỉ được xem mỗi 500 lần lấy node, để `time.monotonic()` không thành chi
  phí đáng kể. `monotonic` được dùng thay `time.time()` để không bị ảnh hưởng khi
  đồng hồ hệ thống thay đổi.

### 8.4 Bộ nhớ

- Mỗi trạng thái framework tốn khoảng 1 KB. `MAX_STATES = 2 000 000` giữ RAM dưới
  khoảng 2 GB. Khi chạm trần, pha hiện tại dừng và solver trả lời giải tốt nhất.
- `MemoryError` được bắt ở cả `anytime_search` và `solve()`.
- **Tắt GC trong lúc tìm kiếm:** trạng thái tìm kiếm không tạo vòng tham chiếu,
  nhưng bộ thu gom vòng vẫn quét lại toàn bộ hàng triệu đối tượng mỗi khi số đối
  tượng tăng. Tắt GC loại bỏ chi phí vô ích này; GC được bật lại trong `finally`.

### 8.5 Hợp đồng với bộ chấm

- Trạng thái đầu vào chỉ được đọc; mọi trạng thái mới do `successors()` tạo ra.
- Trả **đúng** `False` (không phải `None`) khi không có lời giải.
- Đầu vào không hợp lệ hoặc `timebound ≤ 0` trả `False` ngay.

## 9. Bảo đảm tính đúng

Vì tiêu chí số 1 là "không bao giờ sai", nhóm kiểm chứng theo ba tầng.

**Tầng 1 — Lập luận.** Admissible và consistent của heuristic (mục 6), soundness của
deadlock (mục 7), và hệ quả cho vòng anytime (mục 8.2).

**Tầng 2 — Kiểm vét cạn trên bàn nhỏ.** Trên các bàn nhỏ, nhóm sinh **toàn bộ**
không gian trạng thái đến được (hơn 770 nghìn trạng thái), tính khoảng cách tối ưu
thật `h*` cho từng trạng thái, rồi đối chiếu:

- `h(s) ≤ h*(s)` (admissible) và `h(s) ≤ 1 + h(s')` (consistent) với mọi trạng thái
  và mọi cạnh: 0 vi phạm cho `h1`, `h2`, `h3`.
- Không trạng thái giải được nào bị `is_dead` báo deadlock: 0 vi phạm.

Các luật được kiểm đều cục bộ (không phụ thuộc kích thước bàn), nên kết quả trên bàn
nhỏ là bằng chứng mạnh cho bàn lớn.

**Tầng 3 — Kiểm thử tự động.**

| Bộ | Nội dung |
| --- | --- |
| `autograder.py`, `public_tests.py` | Giao diện công khai, mỗi test một tiến trình |
| `test_framework.py` | Luật chơi của framework |
| `test_solution.py` | Khóa trạng thái, `successors_fast` so với framework (random walk), `rebuild`, độ dài tối ưu trên bàn nhỏ |
| `tests/` | Ghép cặp, admissibility/consistency, các mức deadlock, soundness, tương đương engine framework ↔ engine khóa số nguyên |
| `check_imports.py` | `solution.py` chỉ dùng thư viện chuẩn |
| `run_map.py`, `bench.py` | Mỗi lời giải được kiểm lại bằng `validation.py`; phát hiện việc sửa đầu vào |

Test tương đương giữa hai engine đặc biệt quan trọng: nó cho phép phát triển engine
nhanh song song mà vẫn chắc chắn heuristic và deadlock cho **cùng giá trị** trên
hai biểu diễn.

## 10. Các quyết định đã cân nhắc và loại bỏ

| Phương án | Lý do cân nhắc | Lý do không chọn |
| --- | --- | --- |
| A\* thuần (`w = 1`) | Tối ưu, đơn giản | Trên bàn lớn hết giờ mà không có lời giải nào; vi phạm tiêu chí "luôn có lời giải" |
| Một `w` cố định | Đơn giản hơn anytime | Không có `w` nào vừa nhanh trên bàn lớn vừa ngắn trên bàn nhỏ |
| `h3` làm mặc định | Mạnh hơn `h2`, ít node hơn | Phần cộng thêm nhỏ (chỉ tới thùng gần nhất), phụ thuộc vị trí robot nên không cache được; khi đo, tổng độ dài lời giải không tốt hơn `h2` |
| Tìm kiếm tối ưu số lần đẩy (Sokoban cổ điển) | Không gian nhỏ hơn nhiều | Sai mục tiêu: đề tính cả bước đi bộ |
| Deadlock mạnh hơn (corral, pattern) | Cắt được nhiều hơn | Khó chứng minh sound, khó kiểm; rủi ro mất lời giải lớn hơn lợi ích |
| IDA\* | Bộ nhớ tuyến tính | Đồ thị Sokoban có rất nhiều chu trình và trạng thái lặp; không có bảng `visited` thì lặp lại công việc rất nhiều |
| Đa luồng / đa tiến trình | Tận dụng nhiều nhân | Đề cấm |
| Đưa engine khóa số nguyên vào bài nộp | Nhanh hơn, nhẹ hơn rõ rệt | Phần còn thiếu phải viết sát hạn; rủi ro lỗi lớn hơn lợi ích về độ dài lời giải. Giữ lại làm hướng phát triển |

## 11. Hạn chế và hướng phát triển

**Hạn chế:**

- **Heuristic bỏ qua bước đi bộ**, nên cách xa chi phí thật trên bàn lớn nhiều robot.
- **Bộ nhớ:** gần 2 GB ở bàn lớn. Nếu máy chấm giới hạn RAM chặt và hệ điều hành kill
  tiến trình (thay vì Python ném `MemoryError`), lời giải của các pha trước bị mất.
- **Pha đầu thất bại** (hết giờ hoặc chạm trần ở `w = 5`) thì trả `False`; chưa có
  pha dự phòng greedy.
- **Anytime không tái dùng công sức:** mỗi pha xây lại hàng đợi từ đầu.
- **Deadlock chưa đầy đủ:** chưa có corral deadlock, deadlock do robot bị nhốt.
- **Không khai thác đối xứng robot** trong engine đang nộp.

**Hướng phát triển, theo thứ tự lợi ích kỳ vọng:**

1. **Hoàn thiện engine khóa số nguyên** (đã có Khu 1, Khu 4a và test tương đương):
   giảm mạnh thời gian sinh trạng thái và bộ nhớ, kèm gộp trạng thái đối xứng robot.
2. **Heuristic có tính bước đi bộ**, ví dụ cận dưới quãng đường robot phải đi giữa
   các lần đẩy.
3. **ARA\*:** giữ hàng đợi giữa các pha thay vì chạy lại.
4. **Pha dự phòng greedy** khi pha đầu thất bại; giảm `w` theo kiểu chia đôi khi
   chạm trần bộ nhớ.

## 12. Bài học rút ra

- **Đọc kỹ hàm mục tiêu trước khi chọn thuật toán.** Chi tiết "đi bộ cũng tính
  bước" quyết định cả việc chọn A\* trên trạng thái đầy đủ lẫn giới hạn của
  heuristic.
- **Có lời giải trước, tối ưu sau.** Với bộ chấm tính điểm theo số test giải được,
  một lời giải dài vẫn hơn hẳn không có gì. Anytime biến đánh đổi tốc độ–chất lượng
  thành một lịch trình thay vì một lựa chọn.
- **Đo trước khi tối ưu.** Trực giác ban đầu là "heuristic chậm"; profile cho thấy
  nút thắt thật nằm ở việc sinh trạng thái của framework.
- **Sound quan trọng hơn mạnh.** Một luật deadlock cắt nhầm một trạng thái là đủ làm
  mất lời giải. Luật đơn giản, chứng minh được, kiểm vét cạn được là lựa chọn đúng
  khi tính đúng là ưu tiên số 1.
- **Giao diện hẹp giúp làm việc nhóm.** Tách heuristic, deadlock, tìm kiếm thành
  các hàm độc lập cho phép bốn người làm song song và kiểm thử riêng từng phần.
- **Biết dừng.** Không đưa engine mới vào bài nộp sát hạn là một quyết định kỹ
  thuật có chủ đích, không phải việc bỏ dở.

## 13. Tài liệu tham khảo và công cụ

- S. Russell, P. Norvig. *Artificial Intelligence: A Modern Approach*, 4th ed.,
  Chương 3 (tìm kiếm có thông tin, A\*, heuristic từ bài toán nới lỏng).
- Slide môn Trí tuệ nhân tạo, UET: Informed Search (Weighted A\*, admissible,
  consistent).
- R. Zhou, E. Hansen. Multiple sequence alignment using anytime A\*. AAAI 2002.
- M. Likhachev, G. Gordon, S. Thrun. ARA\*: Anytime A\* with provable bounds on
  sub-optimality. NIPS 2003.
- A. Junghanns, J. Schaeffer. Sokoban: Enhancing general single-agent search methods
  using domain knowledge. *Artificial Intelligence* 129, 2001 (deadlock, ghép cặp
  thùng–đích).
- Framework Sokoban do giảng viên cung cấp (`sokoban.py`, `state.py`,
  `validation.py`, `autograder.py`).

**Công cụ AI:**

- GitHub Copilot (VS Code): hỗ trợ chỉnh sửa mã, test và tài liệu phần heuristic,
  deadlock.
- Claude (Anthropic): review code, đề xuất tối ưu, hỗ trợ đo đạc và soạn tài liệu
  (README, báo cáo).<!-- TODO: Soan bao cao chi tiet ban giay; trinh bay bai toan, thiet ke, heuristic/deadlock, thuc nghiem va han che. -->
