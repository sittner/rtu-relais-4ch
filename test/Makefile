# Host tests: firmware modules with a fake HAL, built with the native gcc

CC      ?= gcc
CFLAGS  += -std=gnu11 -Wall -Wextra -g -O0 -fsanitize=address,undefined
CFLAGS  += -Istubs -I../App/Inc -I../third_party/nanomodbus
CFLAGS  += -DFW_VERSION_MAJOR=1 -DFW_VERSION_MINOR=0
CFLAGS  += -DNMBS_CLIENT_DISABLED -DNMBS_STRERROR_DISABLED \
           -DNMBS_SERVER_READ_DISCRETE_INPUTS_DISABLED \
           -DNMBS_SERVER_READ_FILE_RECORD_DISABLED \
           -DNMBS_SERVER_WRITE_FILE_RECORD_DISABLED \
           -DNMBS_SERVER_READ_WRITE_REGISTERS_DISABLED

BUILD   := build
COMMON  := test_common.c stubs/fake_hal.c ../App/Src/mb_server.c ../App/Src/config.c \
           ../App/Src/relay.c ../third_party/nanomodbus/nanomodbus.c

all: run

$(BUILD)/test_plain: test_plain.c ../App/Src/profile_plain.c $(COMMON) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ $^

$(BUILD):
	mkdir -p $@

run: $(BUILD)/test_plain
	$(BUILD)/test_plain

clean:
	rm -rf $(BUILD)

.PHONY: all run clean
