# Hardware - IoT Transportation Monitoring System

Thư mục này chứa toàn bộ mã nguồn phần cứng cho dự án Tốt nghiệp (DATN) về Hệ thống IoT Quan trắc và Giám sát Vận tải. Mã nguồn chính được phát triển trên nền tảng **PlatformIO** cho vi điều khiển **ESP32 (NodeMCU-32S)**.

## 1. Cấu trúc thư mục

Cấu trúc mã nguồn bên trong thư mục `Hardware/`:

```text
Hardware/
├── DATN/                              # Thư mục Project PlatformIO chính
│   ├── include/                       # Chứa các file header (.h)
│   │   ├── adxl345.h, dht11.h, gps.h, lora.h... # Khai báo cho các module phần cứng
│   │   └── vehicle_config.h, frame_codec.h...   # Khai báo cấu hình và định dạng dữ liệu
│   ├── src/                           # Chứa mã nguồn chính (.cpp)
│   │   ├── main.cpp                   # File entry-point chính của hệ thống ESP32
│   │   └── modules/                   # Chứa file thực thi (.cpp) của từng module riêng biệt
│   │       ├── adxl345.cpp            # Xử lý cảm biến gia tốc
│   │       ├── dht11.cpp              # Xử lý cảm biến nhiệt độ/độ ẩm
│   │       ├── gps.cpp                # Xử lý dữ liệu định vị GPS
│   │       ├── lora.cpp               # Xử lý giao tiếp LoRa
│   │       ├── battery.cpp            # Quản lý pin
│   │       └── ...                    
│   ├── lib/                           # Thư mục chứa thư viện cục bộ (nếu có)
│   ├── test/                          # Thư mục chứa các script Unit Test
│   ├── platformio.ini                 # File cấu hình PlatformIO (board, framework, thư viện)
│   ├── hardened_pingpong_rx.c         # File tham khảo: Code LoRa PingPong Nhận
│   ├── hardened_pingpong_tx.c         # File tham khảo: Code LoRa PingPong Phát
│   └── RX_ESP32_FIRESTORE_GUIDE.md    # Tài liệu hướng dẫn thiết lập ESP32 nhận dữ liệu và gửi lên Firestore
└── README.md                          # File hướng dẫn này
```

**Mô hình thiết kế phần mềm:**
Code được chia nhỏ thành các **modules** (trong `src/modules` và `include/`). `main.cpp` sẽ đóng vai trò điều phối các module này hoạt động song song để đọc dữ liệu từ các cảm biến (GPS, ADXL345, DHT11...), đóng gói dữ liệu (`frame_codec`) và gửi đi qua LoRa hoặc các giao thức khác.

---

## 2. Yêu cầu hệ thống (Prerequisites)

Để mở, biên dịch và nạp code vào vi điều khiển, bạn cần cài đặt:
1. **Visual Studio Code (VS Code)**
2. **Extension PlatformIO IDE** (cài đặt trực tiếp từ chợ extension của VS Code).
3. **Driver USB to UART** (ví dụ CP210x, CH340...) tương ứng với board ESP32 của bạn để máy tính nhận diện được cổng COM.

---

## 3. Cách chạy code (Build & Upload)

### Bước 1: Mở dự án trong VS Code
- Mở VS Code, chọn **File > Open Folder...**
- Điều hướng và chọn thư mục `Hardware/DATN/` (Lưu ý: phải chọn thư mục `DATN` có chứa file `platformio.ini`, **không chọn** thư mục `Hardware` ở ngoài cùng).

### Bước 2: Chờ tải thư viện
- Lần đầu tiên mở project, PlatformIO sẽ tự động đọc file `platformio.ini` và tải về các công cụ biên dịch (espressif32) cũng như các thư viện cần thiết (Adafruit DHT, ADXL345, PubSubClient, TinyGPSPlus, ArduinoJson...).
- Đợi đến khi quá trình setup hoàn tất (có thông báo Success ở Terminal).

### Bước 3: Cấu hình cổng COM
- Cắm board ESP32 vào máy tính qua cáp USB (đảm bảo cáp có chức năng truyền dữ liệu).
- Mở file `platformio.ini`. Kiểm tra và cập nhật lại thông số `upload_port` và `monitor_port` cho đúng với cổng COM của thiết bị trên máy bạn (Ví dụ: `COM3`, `COM10` trên Windows, hoặc `/dev/ttyUSB0` trên Linux/Mac).
  ```ini
  upload_port = COM10
  monitor_port = COM10
  ```

### Bước 4: Biên dịch và Nạp code (Build & Upload)
Sử dụng các biểu tượng ở thanh trạng thái (Status Bar) dưới cùng bên trái của VS Code:
- 🛠️ **Build (Dấu check):** Nhấn để biên dịch code (kiểm tra lỗi cú pháp mà không nạp).
- ➡️ **Upload (Mũi tên phải):** Nhấn để biên dịch và nạp code thẳng vào board ESP32.
- 🔌 **Serial Monitor (Biểu tượng phích cắm):** Nhấn để mở màn hình theo dõi log từ ESP32 gửi lên (Baudrate được set là `115200`).

*Hoặc dùng lệnh trong Terminal của PlatformIO:*
- Build: `pio run`
- Upload: `pio run -t upload`
- Serial Monitor: `pio device monitor`

---

## 4. Lưu ý khi nạp code
- Trong quá trình nạp (Upload), nếu Terminal báo lỗi `Connecting...` và đứng im, bạn cần nhấn và giữ nút **BOOT** trên board ESP32 vài giây cho đến khi quá trình nạp bắt đầu chạy %.
- Kiểm tra kết nối các chân cảm biến theo đúng định nghĩa trong các file `.h` tương ứng ở thư mục `include/` trước khi cấp nguồn chạy thực tế.
