CC ?= gcc
CFLAGS ?= -Wall -Wextra -O2 -fsanitize=address,undefined -DUNIT_TEST_HOST
INCLUDES = -Itests -Ifirmware/MainSamsungRB/Core/Inc -Ifirmware/MainRB2/Core/Inc
LDFLAGS = -lm

SRCS = tests/run_all_tests.c \
       tests/mock_hal.c \
       firmware/MainSamsungRB/Core/Src/drive_kinematics.c \
       firmware/MainSamsungRB/Core/Src/rf_hc12.c \
       firmware/MainSamsungRB/Core/Src/rs485_bus.c \
       firmware/MainSamsungRB/Core/Src/bno055.c \
       firmware/MainSamsungRB/Core/Src/bno055_stm32.c \
       firmware/MainSamsungRB/Core/Src/bno08x.c \
       firmware/MainSamsungRB/Core/Src/cd4067.c \
       firmware/MainRB2/Core/Src/flash_cal.c \
       firmware/MainRB2/Core/Src/itc2026_auto.c \
       firmware/MainRB2/Core/Src/tof_vl53l1x.c \
       firmware/MainRB2/Core/Src/vl53l1_platform.c \
       firmware/MainRB2/Core/Src/VL53L1X_api.c

TARGET = tests/run_all_tests

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) $(SRCS) $(LDFLAGS) -o $@

test: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all test clean
