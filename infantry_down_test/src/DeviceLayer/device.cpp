#include "device.hpp"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(device, LOG_LEVEL_INF);

int Device_Init(void)
{
    int ret = 0;

    ret = Remote_Init();
    if (ret != 0)
    {
        LOG_ERR("Remote_Init failed with error code: %d", ret);
        return ret;
    }

    return 0;
}