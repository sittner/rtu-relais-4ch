#include "mb_server.h"

#include <string.h>

#include "main.h"

#include "app.h"
#include "config.h"
#include "profile.h"
#include "relay.h"
#include "rs485.h"

/* Input registers, common */
#define IREG_VERSION     0x0000
#define IREG_PROFILE_ID  0x0001
#define IREG_RELAY_MASK  0x0002
#define IREG_STATUS      0x0003
#define IREG_UPTIME_HI   0x0004
#define IREG_UPTIME_LO   0x0005
#define IREG_CRC_ERRORS  0x0006
#define IREG_UART_ERRORS 0x0007

/* Holding registers, common */
#define HREG_COMM_TIMEOUT 0x0000
#define HREG_SLAVE_ADDR   0x0100
#define HREG_BAUD         0x0101
#define HREG_PARITY       0x0102
#define HREG_COMMAND      0x010F

#define CMD_APPLY_SERIAL 1
#define CMD_DEFAULTS     2

/* Status flags, common */
#define STATUS_COMM_TIMEOUT 0x0001
#define STATUS_DEFAULTS     0x0002
#define STATUS_WDT_RESET    0x0004
#define STATUS_FLASH_ERROR  0x0008

static nmbs_t nmbs;
static nmbs_platform_conf platform;
static nmbs_callbacks callbacks;

/* Serial settings written but not yet applied */
static uint8_t staged_addr;
static uint8_t staged_baud;
static uint8_t staged_parity;

/* Actions run after the reply has been sent */
static bool pending_save;
static bool pending_apply_serial;
static bool pending_defaults;

static uint32_t last_control_ms;
static bool control_write;
static bool comm_timeout;
static uint16_t crc_errors;

static void stage_from_config(void)
{
    staged_addr = config.slave_addr;
    staged_baud = config.baud_idx;
    staged_parity = config.parity;
}

static bool in_profile_range(uint16_t addr)
{
    return addr >= PROFILE_REG_FIRST && addr <= PROFILE_REG_LAST;
}

/* ---- coils ---------------------------------------------------------- */

static nmbs_error cb_read_coils(uint16_t addr, uint16_t qty, nmbs_bitfield out, uint8_t unit_id, void *arg)
{
    (void)unit_id;
    (void)arg;
    if ((uint32_t)addr + qty > RELAY_COUNT) {
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
    for (uint16_t i = 0; i < qty; i++) {
        nmbs_bitfield_write(out, i, relay_get((uint8_t)(addr + i)) ? 1 : 0);
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_write_multiple_coils(uint16_t addr, uint16_t qty, const nmbs_bitfield coils, uint8_t unit_id,
                                          void *arg)
{
    (void)unit_id;
    (void)arg;
    if ((uint32_t)addr + qty > RELAY_COUNT) {
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
    nmbs_error err = profile.write_coils(addr, qty, coils);
    if (err == NMBS_ERROR_NONE) {
        control_write = true;
    }
    return err;
}

static nmbs_error cb_write_single_coil(uint16_t addr, bool value, uint8_t unit_id, void *arg)
{
    nmbs_bitfield coils = { 0 };
    nmbs_bitfield_write(coils, 0, value ? 1 : 0);
    return cb_write_multiple_coils(addr, 1, coils, unit_id, arg);
}

/* ---- input registers ------------------------------------------------ */

static uint16_t status_flags(void)
{
    uint16_t flags = profile.status_flags() & 0xFF00;
    if (comm_timeout) {
        flags |= STATUS_COMM_TIMEOUT;
    }
    if (config_defaults_used()) {
        flags |= STATUS_DEFAULTS;
    }
    if (app_wdt_reset()) {
        flags |= STATUS_WDT_RESET;
    }
    if (config_flash_error()) {
        flags |= STATUS_FLASH_ERROR;
    }
    return flags;
}

static nmbs_error read_ireg(uint16_t addr, uint16_t *val)
{
    switch (addr) {
    case IREG_VERSION:
        *val = (FW_VERSION_MAJOR << 8) | FW_VERSION_MINOR;
        return NMBS_ERROR_NONE;
    case IREG_PROFILE_ID:
        *val = profile.id;
        return NMBS_ERROR_NONE;
    case IREG_RELAY_MASK:
        *val = relay_get_mask();
        return NMBS_ERROR_NONE;
    case IREG_STATUS:
        *val = status_flags();
        return NMBS_ERROR_NONE;
    case IREG_UPTIME_HI:
        *val = (uint16_t)(app_uptime_s() >> 16);
        return NMBS_ERROR_NONE;
    case IREG_UPTIME_LO:
        *val = (uint16_t)app_uptime_s();
        return NMBS_ERROR_NONE;
    case IREG_CRC_ERRORS:
        *val = crc_errors;
        return NMBS_ERROR_NONE;
    case IREG_UART_ERRORS:
        *val = (uint16_t)(rs485_rx_error_count() + rs485_rx_overflow_count());
        return NMBS_ERROR_NONE;
    default:
        if (in_profile_range(addr)) {
            return profile.read_ireg(addr, val);
        }
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
}

static nmbs_error cb_read_input_registers(uint16_t addr, uint16_t qty, uint16_t *out, uint8_t unit_id, void *arg)
{
    (void)unit_id;
    (void)arg;
    for (uint16_t i = 0; i < qty; i++) {
        nmbs_error err = read_ireg((uint16_t)(addr + i), &out[i]);
        if (err != NMBS_ERROR_NONE) {
            return err;
        }
    }
    return NMBS_ERROR_NONE;
}

/* ---- holding registers ---------------------------------------------- */

static nmbs_error read_hreg(uint16_t addr, uint16_t *val)
{
    switch (addr) {
    case HREG_COMM_TIMEOUT:
        *val = config.comm_timeout_s;
        return NMBS_ERROR_NONE;
    case HREG_SLAVE_ADDR:
        *val = staged_addr;
        return NMBS_ERROR_NONE;
    case HREG_BAUD:
        *val = staged_baud;
        return NMBS_ERROR_NONE;
    case HREG_PARITY:
        *val = staged_parity;
        return NMBS_ERROR_NONE;
    case HREG_COMMAND:
        *val = 0;
        return NMBS_ERROR_NONE;
    default:
        if (in_profile_range(addr)) {
            return profile.read_hreg(addr, val);
        }
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
}

static nmbs_error write_hreg(uint16_t addr, uint16_t val, bool apply)
{
    switch (addr) {
    case HREG_COMM_TIMEOUT:
        if (apply) {
            if (config.comm_timeout_s != val) {
                config.comm_timeout_s = val;
                pending_save = true;
            }
            /* Restart the timeout with the new value */
            control_write = true;
        }
        return NMBS_ERROR_NONE;
    case HREG_SLAVE_ADDR:
        if (val < CONFIG_ADDR_MIN || val > CONFIG_ADDR_MAX) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
        }
        if (apply) {
            staged_addr = (uint8_t)val;
        }
        return NMBS_ERROR_NONE;
    case HREG_BAUD:
        if (val >= CONFIG_BAUD_COUNT) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
        }
        if (apply) {
            staged_baud = (uint8_t)val;
        }
        return NMBS_ERROR_NONE;
    case HREG_PARITY:
        if (val > CONFIG_PARITY_ODD) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
        }
        if (apply) {
            staged_parity = (uint8_t)val;
        }
        return NMBS_ERROR_NONE;
    case HREG_COMMAND:
        if (val != CMD_APPLY_SERIAL && val != CMD_DEFAULTS) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
        }
        if (apply) {
            if (val == CMD_APPLY_SERIAL) {
                pending_apply_serial = true;
            } else {
                pending_defaults = true;
            }
        }
        return NMBS_ERROR_NONE;
    default:
        if (in_profile_range(addr)) {
            bool control = false;
            nmbs_error err = profile.write_hreg(addr, val, apply, &control);
            if (apply && err == NMBS_ERROR_NONE && control) {
                control_write = true;
            }
            return err;
        }
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
}

static nmbs_error cb_read_holding_registers(uint16_t addr, uint16_t qty, uint16_t *out, uint8_t unit_id, void *arg)
{
    (void)unit_id;
    (void)arg;
    for (uint16_t i = 0; i < qty; i++) {
        nmbs_error err = read_hreg((uint16_t)(addr + i), &out[i]);
        if (err != NMBS_ERROR_NONE) {
            return err;
        }
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_write_multiple_registers(uint16_t addr, uint16_t qty, const uint16_t *regs, uint8_t unit_id,
                                              void *arg)
{
    (void)unit_id;
    (void)arg;
    /* Validate everything first, then apply: all or nothing */
    for (uint16_t i = 0; i < qty; i++) {
        nmbs_error err = write_hreg((uint16_t)(addr + i), regs[i], false);
        if (err != NMBS_ERROR_NONE) {
            return err;
        }
    }
    for (uint16_t i = 0; i < qty; i++) {
        write_hreg((uint16_t)(addr + i), regs[i], true);
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_write_single_register(uint16_t addr, uint16_t val, uint8_t unit_id, void *arg)
{
    return cb_write_multiple_registers(addr, 1, &val, unit_id, arg);
}

/* ---- device identification (FC 2B/0E) ------------------------------- */

static nmbs_error cb_read_device_identification_map(nmbs_bitfield_256 map)
{
    nmbs_bitfield_set(map, 0x00); /* VendorName */
    nmbs_bitfield_set(map, 0x01); /* ProductCode */
    nmbs_bitfield_set(map, 0x02); /* MajorMinorRevision */
    return NMBS_ERROR_NONE;
}

#define STR_(x) #x
#define STR(x)  STR_(x)

static nmbs_error cb_read_device_identification(uint8_t object_id, char buffer[NMBS_DEVICE_IDENTIFICATION_STRING_LENGTH])
{
    switch (object_id) {
    case 0x00:
        strcpy(buffer, "rtu-relais-4ch");
        return NMBS_ERROR_NONE;
    case 0x01:
        strcpy(buffer, profile.name);
        return NMBS_ERROR_NONE;
    case 0x02:
        strcpy(buffer, STR(FW_VERSION_MAJOR) "." STR(FW_VERSION_MINOR));
        return NMBS_ERROR_NONE;
    default:
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
}

/* ---- server --------------------------------------------------------- */

static void platform_flush(nmbs_t *n, void *arg)
{
    (void)n;
    (void)arg;
    rs485_flush();
}

static void server_create(void)
{
    if (nmbs_server_create(&nmbs, config.slave_addr, &platform, &callbacks) != NMBS_ERROR_NONE) {
        Error_Handler();
    }
    nmbs_set_read_timeout(&nmbs, 0);
    nmbs_set_byte_timeout(&nmbs, rs485_byte_timeout_ms());
}

static void apply_serial(void)
{
    rs485_init(config_baud_rates[config.baud_idx], config.parity);
    server_create();
}

void mb_server_init(void)
{
    nmbs_platform_conf_create(&platform);
    platform.transport = NMBS_TRANSPORT_RTU;
    platform.read = rs485_read;
    platform.write = rs485_write;
    platform.flush = platform_flush;

    nmbs_callbacks_create(&callbacks);
    callbacks.read_coils = cb_read_coils;
    if (profile.write_coils != NULL) {
        callbacks.write_single_coil = cb_write_single_coil;
        callbacks.write_multiple_coils = cb_write_multiple_coils;
    }
    callbacks.read_holding_registers = cb_read_holding_registers;
    callbacks.read_input_registers = cb_read_input_registers;
    callbacks.write_single_register = cb_write_single_register;
    callbacks.write_multiple_registers = cb_write_multiple_registers;
    callbacks.read_device_identification_map = cb_read_device_identification_map;
    callbacks.read_device_identification = cb_read_device_identification;

    stage_from_config();
    apply_serial();
    last_control_ms = HAL_GetTick();
}

void mb_server_poll(uint32_t now_ms)
{
    if (nmbs_server_poll(&nmbs) == NMBS_ERROR_CRC) {
        crc_errors++;
    }

    /* The reply has been sent, now run the deferred actions */
    if (pending_defaults) {
        pending_defaults = false;
        pending_apply_serial = false;
        config_set_defaults();
        config_save();
        stage_from_config();
        apply_serial();
    }
    if (pending_apply_serial) {
        pending_apply_serial = false;
        config.slave_addr = staged_addr;
        config.baud_idx = staged_baud;
        config.parity = staged_parity;
        config_save();
        apply_serial();
    }
    if (pending_save) {
        pending_save = false;
        config_save();
    }

    if (control_write) {
        control_write = false;
        last_control_ms = now_ms;
        comm_timeout = false;
    }
    if (config.comm_timeout_s != 0 && !comm_timeout &&
        now_ms - last_control_ms >= (uint32_t)config.comm_timeout_s * 1000) {
        comm_timeout = true;
        profile.safe_state();
    }
}

bool mb_server_comm_timeout(void)
{
    return comm_timeout;
}
