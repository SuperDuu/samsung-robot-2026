# DEBUG & HARDWARE CONFIGURATION LOG

## Phiên làm việc: 2026-10-27
### Nhiệm vụ: Cấu hình các label chân MCU STM32F405RGT6 trong schematic `MainRB.kicad_sch` dựa trên file `.ioc` tại `/home/du/STM32CubeIDE/workspace_1.19.0/MainSamsungRB/MainSamsungRB.ioc`.

---

### 1. Phân tích nguồn sự thật (Ground Truth Analysis)
- Đọc và bóc tách toàn bộ 51 chân I/O được cấu hình trong `MainSamsungRB.ioc` và `Core/Inc/main.h`:
  - **Nhóm GPIO có User Label trong .ioc (24 chân)**:
    - Line Sensor Indicator LEDs (11 chân):
      - PC15 -> `L1`
      - PC14 -> `L2`
      - PC13 -> `L3`
      - PB7  -> `L4`
      - PB6  -> `L5`
      - PB5  -> `L6`
      - PB4  -> `L7`
      - PB3  -> `L8`
      - PD2  -> `L9`
      - PC12 -> `L10`
      - PC11 -> `L11`
    - 74HC4067 MUX Address Lines (4 chân):
      - PC3  -> `s0`
      - PC2  -> `s1`
      - PC1  -> `s2`
      - PC0  -> `s3`
    - Laser / Distance Sensors (4 chân):
      - PC8  -> `LS1`
      - PC7  -> `LS2`
      - PC6  -> `LS3`
      - PB13 -> `LS4`
    - Màn hình LCD ILI9341 (3 chân GPIO):
      - PA4  -> `MH_CS`
      - PC5  -> `MH_DC`
      - PC4  -> `MH_RST`
    - Status LEDs (2 chân):
      - PB12 -> `Led1`
      - PB2  -> `Led2`
    - Switches / Buttons (2 chân):
      - PC10 -> `SW1`
      - PA15 -> `SW2`
  - **Nhóm Peripheral Pins (27 chân)**:
    - ADC: PA0 -> `ADC1_IN0`, PA1 -> `ADC1_IN1`
    - UART: PA9 -> `USART1_TX`, PA10 -> `USART1_RX`, PA2 -> `USART2_TX`, PA3 -> `USART2_RX`
    - SPI1: PA5 -> `SPI1_SCK`, PA6 -> `SPI1_MISO`, PA7 -> `SPI1_MOSI`
    - I2C1: PB8 -> `I2C1_SCL`, PB9 -> `I2C1_SDA`
    - I2C2: PB10 -> `I2C2_SCL`, PB11 -> `I2C2_SDA`
    - I2C3: PA8 -> `I2C3_SCL`, PC9 -> `I2C3_SDA`
    - PWM Timers: PB0 -> `TIM3_CH3`, PB1 -> `TIM3_CH4`, PB14 -> `TIM12_CH1`, PB15 -> `TIM12_CH2`
    - USB FS: PA11 -> `USB_DM`, PA12 -> `USB_DP`
    - SWD: PA13 -> `SWDIO`, PA14 -> `SWCLK`
    - HSE Clock: PH0 -> `OSC_IN`, PH1 -> `OSC_OUT`
  - **Nhóm System & Power Pins (13 chân)**:
    - System: Pin 7 -> `NRST`, Pin 60 -> `BOOT0`, Pin 31 -> `VCAP_1`, Pin 47 -> `VCAP_2`
    - Power Top: Pin 1 -> `VBAT`, Pin 13 -> `VDDA`, Pins 19, 32, 48, 64 -> `+3.3V`
    - Ground Bottom: Pins 18, 63 -> `GND`, Pin 12 -> `VSSA`

---

### 2. Thiết kế tọa độ & căn chỉnh hình học (Geometry & Grid Alignment)
- Vị trí MCU U1 trong schematic: `(at 97.79 92.71 0)`.
- Khoảng cách pin symbol:
  - Cánh trái: connection point tại `X = 77.47`. Wire stub kéo sang trái `5.08 mm` (2 grid steps 2.54mm) tới `X = 72.39`. Label đặt tại `(at 72.39 Y 0)` với thuộc tính `(justify right bottom)` giúp chữ mở rộng sang trái, tuyệt đối không đè lên dây.
  - Cánh phải: connection point tại `X = 118.11`. Wire stub kéo sang phải `5.08 mm` tới `X = 123.19`. Label đặt tại `(at 123.19 Y 0)` với thuộc tính `(justify left bottom)` giúp chữ mở rộng sang phải.
  - Đỉnh trên (Power): connection point tại `Y = 44.45`. Wire stub kéo lên `5.08 mm` tới `Y = 39.37`. Label đặt tại `(at X 39.37 90)` với `(justify left bottom)`.
  - Đáy dưới (Ground): connection point tại `Y = 140.97`. Wire stub kéo xuống `5.08 mm` tới `Y = 146.05`. Label đặt tại `(at X 146.05 270)` với `(justify left bottom)`.

---

### 3. Kết quả kiểm chứng độc lập (Automated Test Harness)
1. **Kiểm tra xuất Netlist (`kicad-cli sch export netlist`)**:
   - Toàn bộ 64/64 chân của MCU U1 đã kết nối chính xác vào 60 net tương ứng (không còn bất kỳ chân nào ở trạng thái unconnected).
2. **Kiểm tra xuất PDF & Render hình ảnh (`kicad-cli sch export pdf`)**:
   - Xuất file [MainRB.pdf](file:///home/du/Desktop/samsung-robot-2026/hardware/MainRB/MainRB.pdf).
   - Render ảnh PNG phân giải 150 DPI kiểm tra trực quan: 100% các label nằm cách biệt ngoài MCU, không bị chồng đè văn bản, chuẩn chỉnh thẩm mỹ.
