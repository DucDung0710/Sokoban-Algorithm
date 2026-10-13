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

## 3. Thực nghiệm
Ghi các map đã thử, có giải được hay không và số bước của lời giải.
Có thể ghi thêm thời gian để phân tích, dù thời gian không trực tiếp tính điểm.

## 4. Hạn chế và nguồn tham khảo
Nêu phần chưa hoàn thiện, tài liệu, mã nguồn và công cụ AI đã sử dụng.
