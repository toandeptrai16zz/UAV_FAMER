# V3.2.1 — Log rõ điều kiện hiệu chuẩn (CHỈ THỬ KHÔNG CÁNH)

Thay `Core/Src/uav_test.c` của project V3.2 bằng file trong gói, Refresh / Clean / Build / Flash.

KHÔNG thay `.ioc`, không sửa PID, mixer, nguồn hoặc cấu hình DShot, không mở ARM bay.

Dòng BENCH giờ ngắn hơn và có `CALSAFE`:
- `CALSAFE=1`: link tốt, `ARMCH<1320` (ARM OFF) và `KILLCH>=1500` (KILL ON).
- `CALLOW=1`: đã giữ ga <=80 liên tục >=250ms khi CALSAFE=1.
- `CALHIGH=1`: đã giữ ga >=1610 liên tục >=250ms khi CALSAFE=1.
- `THRCAL_OK=1`: cả 2 đầu đã được ghi nhận và ga nằm trong 20..1680.
- `ESC_TEST=0/1/2/3`: idle/sẵn sàng/đang phát xung thấp/hoàn thành.
- `LOCK=1`: vẫn KHÓA ARM BAY, chỉ có cơ chế thử một motor thời gian ngắn khi tháo cánh.

Trình tự kiểm tra hiệu chuẩn: TẮT ARM, BẬT KILL và giữ nguyên cả hai khi đi ga thấp >=0,3s, ga cao >=0,3s, ga thấp. Sau đó kiểm tra CALLOW, CALHIGH và THRCAL_OK. Không kết nối cánh quạt khi thử. Dừng nếu trạng thái công tắc không khớp.

Nếu màn hình còn chồng ký tự, lưu raw log, kiểm tra tốc độ LPUART1 115200 8N1, USB-to-Serial TTL và chỉ mở cổng bằng một chương trình. Không kết luận dữ liệu trong các dòng bị cắt ghép.

Đã qua test host cho rc_input và flight_pid. Chưa có kết quả Build ARM hoặc chạy trên STM32 thật.
