# Hexapod Controller (ESP32-S3)

[English](#english) | [Tiếng Việt](#tiếng-việt)

---

## English

Firmware for an **18-servo hexapod robot** using **ESP32-S3** + **2x PCA9685**.  
Control is provided over **Wi-Fi Access Point** and **WebSocket (JSON)**.

### Features

- 18-servo control (3 DOF × 6 legs)
- Inverse kinematics per leg
- Gait engine for walking modes
- WebSocket server for app commands
- Sensor manager (touch + battery)
- Serial calibration/debug commands

### Architecture

Mobile App (Wi-Fi) ↔ WebSocket(JSON) ↔ ESP32 (AP mode)

Internal modules:
- `gait_engine.*`
- `kinematics.*`
- `servo_controller.*`
- `sensor_manager.*`
- `websocket_server.*`
- `HexapodController.ino` (entry point)

PWM hardware:
- PCA9685 #1: `0x40`
- PCA9685 #2: `0x41`

### Hardware

- ESP32-S3 DevKitC-1 (N16R8)
- 2x PCA9685
- 18x servos
- Dedicated 5V servo power supply (high current recommended)
- Common GND between ESP32, PCA9685, and servo PSU

> Do **not** power all servos directly from ESP32.

### Arduino IDE Setup

#### 1) Board package
Install **esp32 by Espressif Systems** in Boards Manager.

#### 2) Required libraries
Install from Library Manager:
- Adafruit PWM Servo Driver Library
- WebSockets by Markus Sattler
- ArduinoJson (7.x)

#### 3) Recommended board settings (ESP32-S3)
- **Board**: `ESP32S3 Dev Module`
- **USB Mode**: `Hardware CDC and JTAG`
- **Upload Mode**: `UART0 / Hardware CDC` (or Hardware CDC)
- **USB CDC On Boot**: `Enabled`
- **USB DFU On Boot**: `Disabled`
- **Flash Mode**: `QIO 80MHz`
- **Flash Size**: `16MB`
- **PSRAM**: `OPI PSRAM`
- **Partition Scheme**: `Default 4MB with spiffs`
- **Upload Speed**: `921600` (use `460800` if unstable)

### Build & Upload

1. Open `HexapodController.ino`
2. Select board and port
3. Upload firmware
4. Open Serial Monitor at `115200`

### After Upload

ESP32 starts a Wi-Fi AP:
- **SSID**: `Hexapod-Robot`
- **Password**: `hexapod123`
- **IP**: `192.168.4.1`

WebSocket endpoint:
- `ws://192.168.4.1:81`

### Serial Commands (calibration/debug)

- `center` → move all servos to center
- `stop` → stop gait
- `disable` → disable servo torque
- `<id> <angle>` e.g. `0 90` (`id`: 0..17, `angle`: 0..180)

### Servo Calibration (recommended before gait)

1. Lift robot off the ground
2. Run `center`
3. Verify each joint direction
4. Set `SERVO_CAL[i].inverted` if direction is reversed
5. Tune `SERVO_CAL[i].trim` if center offset exists

### Common Issues

#### Watchdog reset (`TG1WDT_SYS_RST`)
Possible causes:
- Long blocking loops without `delay()/yield()`
- I2C error loops
- Servo power noise during startup

Fixes:
- Add `delay(1)` in heavy init/loops
- Initialize `Wire.begin(...)` only once in `setup()`
- Use separate servo PSU + common ground

#### I2C `ESP_ERR_INVALID_STATE`
- Avoid multiple `Wire.begin()` calls
- Verify SDA/SCL wiring
- Use 100kHz I2C clock
- Check servo power noise

#### Linker errors (`undefined reference`)
- Ensure function signatures in `.h` and `.cpp` match exactly
- Clean build cache and recompile

---

## Tiếng Việt

Firmware cho robot **hexapod 18 servo** dùng **ESP32-S3** + **2x PCA9685**.  
Điều khiển qua **Wi-Fi Access Point** và **WebSocket (JSON)**.

### Tính năng

- Điều khiển 18 servo (3 DOF × 6 chân)
- Inverse kinematics cho từng chân
- Gait engine cho các chế độ di chuyển
- WebSocket server nhận lệnh từ app
- Sensor manager (touch + battery)
- Lệnh Serial để calibration/debug

### Kiến trúc

Mobile App (Wi-Fi) ↔ WebSocket(JSON) ↔ ESP32 (AP mode)

Các module chính:
- `gait_engine.*`
- `kinematics.*`
- `servo_controller.*`
- `sensor_manager.*`
- `websocket_server.*`
- `HexapodController.ino` (entry point)

PWM phần cứng:
- PCA9685 #1: `0x40`
- PCA9685 #2: `0x41`

### Phần cứng

- ESP32-S3 DevKitC-1 (N16R8)
- 2x PCA9685
- 18x servo
- Nguồn servo 5V riêng (khuyến nghị dòng lớn)
- Nối chung GND giữa ESP32, PCA9685 và nguồn servo

> Không cấp trực tiếp toàn bộ servo từ ESP32.

### Cài đặt Arduino IDE

#### 1) Board package
Cài **esp32 by Espressif Systems** trong Boards Manager.

#### 2) Thư viện cần thiết
Cài từ Library Manager:
- Adafruit PWM Servo Driver Library
- WebSockets by Markus Sattler
- ArduinoJson (7.x)

#### 3) Cấu hình board khuyến nghị (ESP32-S3)
- **Board**: `ESP32S3 Dev Module`
- **USB Mode**: `Hardware CDC and JTAG`
- **Upload Mode**: `UART0 / Hardware CDC` (hoặc Hardware CDC)
- **USB CDC On Boot**: `Enabled`
- **USB DFU On Boot**: `Disabled`
- **Flash Mode**: `QIO 80MHz`
- **Flash Size**: `16MB`
- **PSRAM**: `OPI PSRAM`
- **Partition Scheme**: `Default 4MB with spiffs`
- **Upload Speed**: `921600` (không ổn định thì dùng `460800`)

### Build & Nạp firmware

1. Mở `HexapodController.ino`
2. Chọn đúng board và port
3. Upload firmware
4. Mở Serial Monitor ở `115200`

### Sau khi nạp

ESP32 tạo Wi-Fi AP:
- **SSID**: `Hexapod-Robot`
- **Password**: `hexapod123`
- **IP**: `192.168.4.1`

WebSocket endpoint:
- `ws://192.168.4.1:81`

### Lệnh Serial (calibration/debug)

- `center` → đưa tất cả servo về vị trí giữa
- `stop` → dừng gait
- `disable` → ngắt lực servo
- `<id> <góc>` ví dụ `0 90` (`id`: 0..17, `góc`: 0..180)

### Calibration servo (nên làm trước khi chạy gait)

1. Nhấc robot khỏi mặt đất
2. Gõ `center`
3. Kiểm tra chiều quay từng khớp
4. Đặt `SERVO_CAL[i].inverted` nếu ngược chiều
5. Chỉnh `SERVO_CAL[i].trim` nếu lệch tâm

### Lỗi thường gặp

#### Reset watchdog (`TG1WDT_SYS_RST`)
Nguyên nhân:
- Vòng lặp block lâu, không `delay()/yield()`
- I2C lỗi lặp liên tục
- Nhiễu/sụt nguồn servo lúc khởi động

Cách xử lý:
- Thêm `delay(1)` ở các đoạn init nặng/loop dài
- Chỉ `Wire.begin(...)` một lần trong `setup()`
- Dùng nguồn servo riêng + nối chung mass

#### I2C `ESP_ERR_INVALID_STATE`
- Tránh gọi `Wire.begin()` nhiều nơi
- Kiểm tra lại dây SDA/SCL
- Dùng clock I2C 100kHz
- Kiểm tra nhiễu từ nguồn servo

#### Lỗi linker (`undefined reference`)
- Đảm bảo khai báo hàm trong `.h` và `.cpp` khớp tuyệt đối
- Xóa cache build rồi compile lại

---

## Project Structure / Cấu trúc dự án

```text
HexapodController/
├─ HexapodController.ino
├─ config.h
├─ servo_controller.h
├─ servo_controller.cpp
├─ kinematics.h
├─ kinematics.cpp
├─ gait_engine.h
├─ gait_engine.cpp
├─ sensor_manager.h
├─ sensor_manager.cpp
├─ websocket_server.h
└─ websocket_server.cpp
```

## License

Add your preferred license here (MIT/Apache-2.0/...).  
Thêm license bạn muốn sử dụng (MIT/Apache-2.0/...).