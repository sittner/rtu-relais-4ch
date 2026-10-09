#include "app.h"

#include "main.h"
#include "iwdg.h"

void app_init(void)
{
    /* Stop the IWDG while the core is halted by the debugger */
    __HAL_RCC_DBGMCU_CLK_ENABLE();
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

void app_loop(void)
{
    /* The watchdog is only refreshed here, at the end of the main loop */
    HAL_IWDG_Refresh(&hiwdg);
}
