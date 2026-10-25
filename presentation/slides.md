# Nội dung phần C — Slides 7–9

## Slide 7 — Heuristic khoảng cách đẩy
- `precompute_push_dist`: BFS ngược từ từng đích, chỉ xét ô hộp và ô robot
  hỗ trợ đều thông sàn.
- `h1`: cộng khoảng cách đẩy tới đích gần nhất của từng hộp; admissible nhưng
  nhiều hộp có thể cùng chọn một đích trong ước lượng.
- `h2`: Hungarian ghép hộp với các đích khác nhau; tổng chi phí đẩy là cận
  dưới admissible.

## Slide 8 — Heuristic và deadlock
- `h3 = h2 + max(0, d_min - 1)`, với `d_min` là Manhattan nhỏ nhất giữa robot
  và hộp; trước lần đẩy đầu, robot phải đi ít nhất `d_min - 1` bước.
- `is_dead_static`: cắt hộp nằm trên ô không thể đẩy tới bất kỳ đích nào.
- Freeze detection: lan truyền các hộp bị khóa theo hai trục bởi tường/hộp
  đã khóa; sau mỗi lần đẩy chỉ kiểm tra thùng vừa đẩy và các thùng kề.
- Ablation: `CONFIG["dead"]` chọn `none`, `cells` hoặc `freeze`; cache `h2`
  bị giới hạn bởi `CONFIG["h_cache_max"]`.
- Hạn chế: chưa phát hiện mọi deadlock động; detector ưu tiên không cắt nhầm
  trạng thái có thể giải.

## Slide 9 — Kiểm chứng và demo
- Test `h1`–`h3` bằng BFS chính xác; vét cạn map 4 kiểm 127.512 state, không
  thấy vi phạm admissible trên 12.936 state giải được.
- Unit test: phần C 11/11, thuật toán hỗ trợ 7/7, framework 7/7, public 3/3 qua.
- Benchmark số state mở rộng (`tắt / bật deadlock`):

  | Map | h0 | h1 | h2 | h3 |
  | --- | ---: | ---: | ---: | ---: |
   | 1 | 11.999 / 13.499* | 6.494 / 6.354 | 6.434 / 6.298 | 6.108 / 5.976 |
  | 4 | 10.874 / 2.814 | 634 / 634 | 542 / 542 | 467 / 467 |
   | 6 | 15.499* / 15.999* | 17.871 / 13.993* | 12.173 / 11.015 | 11.175 / 10.137 |

- `*` quá hạn khoảng 1,5 giây. Mọi lượt tìm được lời giải đều có độ dài tối
   ưu tương ứng 16, 10 hoặc 30 bước. Freeze có chi phí kiểm tra nên số node
   giảm không phải lúc nào cũng làm thời gian giảm.
- Thời gian/node phụ thuộc máy; freeze detector chỉ nhận diện freeze chứng
  minh được, không bao phủ mọi deadlock động.
