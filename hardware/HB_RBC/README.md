# HB_RBC — H-Bridge DC Motor Driver (Robocon DKH 2026)

**HB_RBC** là mạch điều khiển động cơ DC (Full H-Bridge Driver) công suất cao được thiết kế phục vụ cho đội tuyển **Robocon DKH 2026**. 

Mạch hướng tới sự nhỏ gọn, độ tin cậy cao trong thi đấu thực tế, tích hợp sẵn vi điều khiển trung tâm, mạch lái cổng chống trùng dẫn, cổng phản hồi encoder vòng kín và giao tiếp mạng bus RS-485.

---

## 🚀 Tính năng & Cấu hình chính

- **Vi điều khiển trung tâm**: STM32G030F6Px (ARM Cortex-M0+ @ 64MHz) điều khiển băm xung PWM và xử lý thuật toán vòng kín.
- **Tầng công suất Cầu H**: 
  - 4x N-MOSFET **BSC040N10NS5** ($100\text{V}$, $4.0\text{ m}\Omega$, gói TDSON-8) trở kháng siêu thấp, tỏa nhiệt ít khi chạy tải nặng.
  - Diode xả tự do Schottky (SS310) bảo vệ chống xung ngược cảm ứng từ động cơ.
- **Mạch lái cổng (Gate Driver)**: 
  - 2x IC lái nửa cầu **IR2184** tích hợp sẵn deadtime phần cứng $500\text{ns}$ chống chập nguồn (shoot-through).
  - Tích hợp đường ngắt khẩn cấp phần cứng (`/S`), cho phép khóa cầu H ngay lập tức khi có sự cố.
- **Nguồn cấp**: 
  - Điện áp danh định: $12\text{V} \sim 24\text{V}$ DC (bảo vệ quá dòng qua cầu chì đầu vào).
  - Hạ áp 3 tầng riêng biệt: $24\text{V} \to 15\text{V}$ (Lái Gate) $\to 5\text{V}$ $\to 3.3\text{V}$ (Logic & MCU).
- **Giao tiếp & Cảm biến**:
  - **RS-485 (MAX3485)**: Giao tiếp vi sai chống nhiễu, tích hợp sẵn trở tải $120\Omega$ và mạch phân cực bus.
  - **Encoder AB**: Cổng nhận xung kênh đôi hỗ trợ kiểm soát tốc độ / vị trí động cơ.
  - **Debug SWD**: Header chuẩn nạp chương trình bằng ST-Link.
- **Quy cách PCB**: $50 \times 60\text{ mm}$, 2 lớp đồng, pad hàn dây nguồn và động cơ khổ lớn $4 \times 8\text{ mm}$, 4 lỗ ốc M3.

---

## 🔌 Sơ đồ kết nối ngoại vi

| Cổng | Loại đầu nối | Chức năng | Chi tiết chân |
| :--- | :--- | :--- | :--- |
| **J3 / J4** | Solder Pad $4 \times 8\text{ mm}$ | **Nguồn vào DC** | `J3`: +24V (qua cầu chì) \| `J4`: GND |
| **J1 / J2** | Solder Pad $4 \times 8\times\text{ mm}$ | **Đầu ra Động cơ** | `J1`: M+ \| `J2`: M- |
| **J5** | JST-XH 2.50mm 2P | **Bus RS-485** | Pin 1: B \| Pin 2: A |
| **J6** | JST-XH 2.50mm 4P | **Cảm biến Encoder** | Pin 1: +3.3V \| Pin 2: Kênh A (PA6) \| Pin 3: Kênh B (PA7) \| Pin 4: GND |
| **J7** | JST-XH 2.50mm 4P | **Cổng nạp SWD** | Pin 1: +3.3V \| Pin 2: SWDIO (PA13) \| Pin 3: SWCLK (PA14) \| Pin 4: GND |

---

## 👨‍💻 Thông tin dự án

- **Đơn vị phát triển**: Robocon DKH 2026
- **Thiết kế phần cứng**: Vũ Đức Du ([@SuperDuu](https://github.com/SuperDuu))
- **Công cụ thiết kế**: KiCad EDA
