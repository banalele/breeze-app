#define IWDG_INTERNAL  /* 本文件内 IWDG_Channel_ID 可写 */
#include "iwdg.h"

/* 
*   日志模块注册
*   改成LOG_LEVEL_DBG即可输出DBG级别的日志
*   输出DBG级别日志需要注意 CONFIG_LOG_PROCESS_THREAD_STACK_SIZE 配置，防止日志过多导致栈溢出
*/
LOG_MODULE_REGISTER(iwdg, LOG_LEVEL_INF);
       
/* 在调试暂停的时候暂停看门狗计数 */
// 实测用处不大，在看门狗处同时打两个断点gdb调试就会崩溃
#define DEBUG
#ifndef DEBUG
    int flags = 0;
#else
    int flags = WDT_OPT_PAUSE_HALTED_BY_DBG;
#endif

static int iwdg_channel_id;
static struct wdt_timeout_cfg iwdg_config;
int IWDG_Channel_ID;  /* 全局定义，头文件 extern const 保证外部只读 */

int IWDG_Init(const struct device *iwdg_dev)
{
    /* 配置看门狗超时参数 */
    iwdg_config = (struct wdt_timeout_cfg){
        .window = {
            .max = 2000U,                   /* 超时时间: 2000 毫秒 */
        },
        .callback = NULL,                   /* STM32 IWDG 不支持中断回调，仅支持超时复位 */
        .flags = WDT_FLAG_RESET_SOC,        /* 超时后执行系统复位 */
    };

    /* 安装超时配置，返回通道ID */
    iwdg_channel_id = wdt_install_timeout(iwdg_dev, &iwdg_config);
    if (iwdg_channel_id < 0) {
        LOG_ERR("Failed to install iwdg timeout: %d", iwdg_channel_id);
        return -1;
    }
    IWDG_Channel_ID = iwdg_channel_id;

    /* 检查看门狗设备是否就绪 */
    if (!device_is_ready(iwdg_dev)) {
        LOG_ERR("Watchdog device %s is not ready", iwdg_dev->name);
        return -1;
    }

    /* 启动看门狗 */
    if (wdt_setup(iwdg_dev, flags) < 0) {
        LOG_ERR("Failed to setup watchdog: %s", iwdg_dev->name);
        return -1;
    }

    LOG_INF("Watchdog configured and started.");

    return 0;
}

int IWDG_Feed(const struct device *iwdg_dev, int channel_id)
{
    /* 喂狗，重置看门狗计数器 */
    int ret = wdt_feed(iwdg_dev, channel_id);
    if (ret < 0) {
        LOG_ERR("Failed to feed watchdog: %d", ret);
        return -1;
    } else {
        LOG_DBG("Watchdog fed successfully.");
    }
    return 0;
}
