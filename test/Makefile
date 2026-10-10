# SPDX-License-Identifier: GPL-3.0-or-later
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

$(BUILD)/test_heater: test_heater.c ../App/Src/profile_heater.c ../App/Src/heater_ctrl.c $(COMMON) | $(BUILD)
	$(CC) $(CFLAGS) -DFW_PROFILE_HEATER -o $@ $^

# Simulators for tools/test_device.py (no sanitizers, runs forever)
SIM_SRC := sim.c ../App/Src/mb_server.c ../App/Src/config.c ../App/Src/relay.c \
           ../third_party/nanomodbus/nanomodbus.c
SIM_CFLAGS := $(filter-out -fsanitize=%,$(CFLAGS))

$(BUILD)/sim_plain: $(SIM_SRC) ../App/Src/profile_plain.c | $(BUILD)
	$(CC) $(SIM_CFLAGS) -o $@ $^

$(BUILD)/sim_heater: $(SIM_SRC) ../App/Src/profile_heater.c ../App/Src/heater_ctrl.c | $(BUILD)
	$(CC) $(SIM_CFLAGS) -DFW_PROFILE_HEATER -o $@ $^

sim: $(BUILD)/sim_plain $(BUILD)/sim_heater

$(BUILD):
	mkdir -p $@

run: $(BUILD)/test_plain $(BUILD)/test_heater
	$(BUILD)/test_plain
	$(BUILD)/test_heater

clean:
	rm -rf $(BUILD)

.PHONY: all run sim clean
