# Nội dung phần C — Slides 7–9

## Slide 7 — Heuristic khoảng cách đẩy
- `precompute_push_dist`: BFS ngược từ từng đích, chỉ xét ô hộp và ô robot
  hỗ trợ đều thông sàn.
- `h1`: cộng khoảng cách đẩy tới đích gần nhất của từng hộp; admissible nhưng
  nhiều hộp có thể cùng chọn một đích trong ước lượng.
- `h2`: Hungarian ghép hộp với các đích khác nhau; tổng chi phí đẩy là cận
  dưới admissible.

## Slide 8 — Heuristic và deadlock
- `h3 = h2 +` số bước đi bộ tối thiểu (bỏ qua hộp) tới vị trí hỗ trợ của một
  lần đẩy đầu tiên; phần đi bộ và phần đẩy không đếm cùng bước.
- `is_dead_static`: cắt hộp nằm trên ô không thể đẩy tới bất kỳ đích nào.
- Freeze detection: lan truyền các hộp bị khóa theo hai trục bởi tường/hộp
  đã khóa; chỉ báo deadlock khi hộp bị khóa không ở trên đích.
- Hạn chế: chưa phát hiện mọi deadlock động; detector ưu tiên không cắt nhầm
  trạng thái có thể giải.

## Slide 9 — Kiểm chứng và demo
- Test `h1`–`h3` bằng BFS chính xác; vét cạn map 4 kiểm 127.512 state, không
  thấy vi phạm admissible trên 12.936 state giải được.
- Unit test: phần C 7/7, framework 7/7, public 3/3 qua.
- Benchmark số state mở rộng (`tắt / bật deadlock`):

  | Map | h0 | h1 | h2 | h3 |
  | --- | ---: | ---: | ---: | ---: |
  | 1 | 51.916 / 48.499* | 6.494 / 6.490 | 6.434 / 6.434 | 5.089 / 5.089 |
  | 4 | 10.874 / 2.814 | 634 / 634 | 542 / 542 | 467 / 467 |
  | 6 | 61.999* / 30.068 | 17.871 / 17.399 | 12.173 / 11.977 | 10.336 / 10.212 |

- `*` quá hạn 1,5 giây; mọi lượt khác tìm được lời giải tối ưu dài lần lượt
  16, 10 và 30 bước. Map hành lang tự tạo: cả bốn heuristic tìm lời giải 3
  bước.
- Thời gian/node phụ thuộc máy; freeze detector chỉ nhận diện freeze chứng
  minh được, không bao phủ mọi deadlock động.
