/**
 ******************************************************************************
 * @file        imu_uart.c
 * @brief       ATOM-IMU 串口传输层（Zephyr）：USART10 中断接收
 *
 * 对应 HAL 版 drv_uart.c 中 USART1 + DMA 循环 + 空闲中断的部分：
 *   中断里只把字节丢进环形缓冲（快进快出），
 *   主循环每 1 ms 调用 imu_update() 组帧解析（不阻塞中断）。
 *
 * 调用关系：USART10 中断 -> 环形缓冲 -> imu_update() -> imu_encode()
 *
 * 设备树：damiao_mc02 的 usart10（RX=PE2, TX=PE3, 2000000 8N1, 已开 FIFO）
 ******************************************************************************
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include "imu_uart.h"
#include "imu_xrobot.h"

LOG_MODULE_REGISTER(imu_uart, LOG_LEVEL_INF);

#define IMU_UART_NODE DT_NODELABEL(usart10)

/* 环形缓冲：2000000 baud 下约 20 ms 的数据量 */
#define IMU_RX_RING_SIZE 4096
/* 解析暂存区，对应 HAL 版的 USART1_RX_BUF_LEN */
#define IMU_RX_STAGE_SIZE 128

static const struct device *const imu_dev = DEVICE_DT_GET(IMU_UART_NODE);

/* ISR 写、线程读 */
RING_BUF_DECLARE(imu_ring_buffer, IMU_RX_RING_SIZE);

/* 未解析完的尾部数据暂存（跨 imu_update() 调用保留） */
static uint8_t stage_buf[IMU_RX_STAGE_SIZE];
static size_t stage_len;

/**
 * @brief USART10 中断回调：把硬件 FIFO 里的字节搬进环形缓冲
 *
 * 中断里只做搬运、不做解析，保证快进快出。
 * 线程侧由 imu_update() 从环形缓冲取数据并解析。
 */
static void USART10_rxDataHandler(const struct device *dev, void *user_data)
{
	uint8_t buff[32];
	int n;

	ARG_UNUSED(user_data);

	uart_irq_update(dev);

	while (uart_irq_rx_ready(dev))
	{
		n = uart_fifo_read(dev, buff, sizeof(buff));
		if (n <= 0)
		{
			break;
		}

		/* ring 满时多余字节会被丢掉，所以主循环要每 1 ms 调用 imu_update() */
		(void)ring_buf_put(&imu_ring_buffer, buff, (uint32_t)n);
	}
}

int imu_uart_start(void)
{
	int ret;

	if (!device_is_ready(imu_dev))
	{
		LOG_ERR("USART10 未就绪，请检查设备树 DT_NODELABEL(usart10)");
		return -ENODEV;
	}

	imu_xrobot_init(); /* 清空 imu_data */

	stage_len = 0; /* 丢掉可能残留的半帧 */

	ret = uart_irq_callback_user_data_set(imu_dev, USART10_rxDataHandler, NULL);
	if (ret < 0)
	{
		LOG_ERR("注册 UART 中断回调失败 (%d)，需要 CONFIG_UART_INTERRUPT_DRIVEN=y", ret);
		return ret;
	}

	uart_irq_rx_enable(imu_dev);

	LOG_INF("ATOM-IMU 接收已启动：%s", imu_dev->name);

	return 0;
}

void imu_update(void)
{
	uint16_t used;

	/* 一次调用可能积压多帧（ring 比暂存区大），循环取到只剩半帧为止 */
	for (;;)
	{
		/* 1. 环形缓冲 -> 暂存区尾部（stage_len 同时充当写指针） */
		stage_len += ring_buf_get(&imu_ring_buffer, &stage_buf[stage_len],
								  sizeof(stage_buf) - stage_len);

		/* 2. 解析，返回真正吃掉多少字节 */
		used = imu_encode(stage_buf, (uint16_t)stage_len);

		if (used == 0)
		{
			break; /* 凑不满一帧，等下一批数据 */
		}

		/* 3. 丢弃已解析部分，把残余的半帧挪到 buf 开头 */
		stage_len -= used;
		memmove(stage_buf, &stage_buf[used], stage_len);
	}
}
