#pragma once

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/watchdog.h>

#ifdef __cplusplus
extern "C" {
#endif

int IWDG_Init(const struct device *iwdg_dev);
int IWDG_Feed(const struct device *iwdg_dev, int channel_id);

#ifdef IWDG_INTERNAL
extern int IWDG_Channel_ID;         /* iwdg.c 内部：可写 */
#else
extern const int IWDG_Channel_ID;   /* 外部：只读 */
#endif

#ifdef __cplusplus
}
#endif
