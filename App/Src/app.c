#include "app.h"

#include "main.h"
#include "iwdg.h"

#include "config.h"
#include "led.h"
#include "mb_server.h"
#include "profile.h"
#include "relay.h"

/* Factory reset: SWDIO (PA13) held low for this long after power-up */
#define FACTORY_RESET_SAMPLE_MS 50

static bool wdt_reset;
static uint32_t uptime_s;
static uint32_t uptime_ms_acc;
static uint32_t last_tick;

/* PA13 stays in SWD mode, IDR still reflects the pin (internal pull-up) */
static bool factory_reset_requested(void)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < FACTORY_RESET_SAMPLE_MS) {
        if (GPIOA->IDR & GPIO_PIN_13) {
            return false;
        }
    }
    return true;
}

void app_init(void)
{
    /* Stop the IWDG while the core is halted by the debugger */
    __HAL_RCC_DBGMCU_CLK_ENABLE();
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

    wdt_reset = __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST);
    __HAL_RCC_CLEAR_RESET_FLAGS();

    relay_init();

    config_init();
    if (factory_reset_requested()) {
        config_set_defaults();
        config_save();
        led_ack();
    }

    profile.init();
    mb_server_init();

    last_tick = HAL_GetTick();
}

void app_loop(void)
{
    uint32_t now = HAL_GetTick();

    uptime_ms_acc += now - last_tick;
    last_tick = now;
    while (uptime_ms_acc >= 1000) {
        uptime_ms_acc -= 1000;
        uptime_s++;
    }

    mb_server_poll(now);
    profile.poll(now);

    led_set_mode(mb_server_comm_timeout() ? LED_MODE_TIMEOUT : LED_MODE_NORMAL);
    led_poll(now);

    /* The watchdog is only refreshed here, at the end of the main loop */
    HAL_IWDG_Refresh(&hiwdg);
}

bool app_wdt_reset(void)
{
    return wdt_reset;
}

uint32_t app_uptime_s(void)
{
    return uptime_s;
}
