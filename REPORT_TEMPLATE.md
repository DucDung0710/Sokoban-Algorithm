<!-- TODO: Them template bao cao chinh thuc do giang vien cung cap; khong tu dat lai tieu chi cham. -->
# Mô tả thiết kế solver Sokoban

Tài liệu này dùng để giải thích bài nộp, không có điểm riêng trong thang chấm.
Không bắt buộc một thuật toán, heuristic hay cấu trúc dữ liệu cụ thể.

## 1. Thiết kế
Mô tả phương pháp giải, cách tổ chức chương trình và lý do lựa chọn.

## 2. Chất lượng lời giải
Giải thích phương pháp tìm hoặc cải thiện lời giải. Nếu tuyên bố tối ưu,
trình bày cơ sở cho tuyên bố đó; nếu không, nêu rõ giới hạn.

### Tính admissible và consistent của h1
Với mỗi ô đích, BFS ngược tính số lần đẩy ít nhất từ từng ô tới đích. Khi
tính bảng, chỉ xét tường; bỏ qua các hộp khác và khả năng robot khó tiếp cận
ô hỗ trợ. Vì vậy, bảng có thể cho phép đường đẩy dễ hơn thực tế nhưng không
loại bỏ đường đi hợp lệ nào. `pd_min` của một hộp là khoảng cách nhỏ nhất tới
một đích bất kỳ, nên không lớn hơn số lần hộp đó thực sự bị đẩy trong lời giải.

Mỗi bước đẩy chỉ di chuyển đúng một hộp, do đó tổng `pd_min` của các hộp không
lớn hơn tổng số lần đẩy. Đi bộ cũng tốn bước nhưng không làm tăng `h1`; vì thế
`h1` không lớn hơn tổng số bước của bất kỳ lời giải nào, đặc biệt là lời giải
tối ưu. Nếu `pd_min` là INF thì ngay cả trên bàn chỉ có tường cũng không có
chuỗi đẩy tới đích, nên trạng thái đó không thể giải được.

`h1` cũng consistent: bước đi bộ không đổi vị trí hộp nên heuristic giữ nguyên.
Một bước đẩy đưa một hộp qua một cạnh của đồ thị đẩy; khoảng cách ngắn nhất
của hộp chỉ có thể giảm tối đa một. Các hộp còn lại không đổi, nên
`h1(s) <= 1 + h1(s')` cho mọi bước hợp lệ `s -> s'`.

`h2` ghép mỗi hộp với một đích riêng bằng chi phí đẩy tĩnh nhỏ nhất. Trong mọi
lời giải, các hộp kết thúc ở các đích khác nhau; khoảng cách đẩy tĩnh của mỗi
cặp không vượt số lần đẩy thực tế của hộp đó. Tối ưu hóa trên các ghép hợp lệ
vẫn không vượt chi phí của ghép xuất hiện trong lời giải, nên `h2` admissible.

`h3` cộng `h2` với khoảng cách đi bộ nhỏ nhất từ bất kỳ robot nào tới một ô hỗ
trợ hình học cho một lần đẩy đầu tiên. BFS bỏ qua hộp nên khoảng cách này không
lớn hơn số bước đi bộ trước lần đẩy đầu thực tế. Phần đi bộ đó tách biệt với
các lần đẩy được `h2` đếm, vì vậy tổng vẫn admissible. Cần kiểm chứng thực
nghiệm trước khi tuyên bố tính consistent của `h2`/`h3`.

Freeze detector chỉ đánh dấu hộp là bị khóa khi cả hai trục đẩy đều bị chặn
bởi tường hoặc hộp đã được chứng minh khóa; chỉ cắt state nếu hộp bị khóa
không ở đích. Đây là phép kiểm tra bảo thủ, không bao phủ toàn bộ deadlock
động.

## 3. Thực nghiệm
Ghi các map đã thử, có giải được hay không và số bước của lời giải.
Có thể ghi thêm thời gian để phân tích, dù thời gian không trực tiếp tính điểm.
Chạy và lưu số liệu so sánh `h0`–`h3` cùng bật/tắt deadlock trên map 1, 4, 6
và map tự tạo; chưa điền số liệu trước khi chạy trên môi trường dự án.

## 4. Hạn chế và nguồn tham khảo
Nêu phần chưa hoàn thiện, tài liệu, mã nguồn và công cụ AI đã sử dụng.
