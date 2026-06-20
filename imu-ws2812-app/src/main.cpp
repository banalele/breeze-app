#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

LOG_MODULE_REGISTER(ws2812, LOG_LEVEL_INF);//输出日志

///ws2812注册
/* default configuration */
#define SPI_FRAME_BITS 8
#define BITS_PER_COLOR_CHANNEL 8

// get ws2812 node
#define LED_WS2812_NODE DT_NODELABEL(rgb_led)

/* WS2812 device */
static const struct device *ws2812_dev;
static struct led_rgb *pixels;

//ws2812控制块和栈
struct k_thread led_thread_data;
#define K_LED_TASK_STACK_SIZE 1024
#define K_LED_TASK_PRIORITY 5  
K_THREAD_STACK_DEFINE(led_stack_area, K_LED_TASK_STACK_SIZE);

//imu注册
#define IMU_ACCEL_NODE DT_NODELABEL(bmi08x_accel)
static const struct device *accel_dev;
struct k_thread accel_thread_data;
#define K_ACCEL_TASK_STACK_SIZE 1024
#define K_ACCEL_TASK_PRIORITY 4
K_THREAD_STACK_DEFINE(accel_stack_area, K_ACCEL_TASK_STACK_SIZE);

typedef enum
{
    LED_COLOR_RED,
    LED_COLOR_GREEN,
    LED_COLOR_BLUE,
    LED_COLOR_YELLOW,
    LED_COLOR_CYAN,
    LED_COLOR_MAGENTA,
    LED_COLOR_WHITE,
    LED_COLOR_PURPLE,
    LED_COLOR_ORANGE,
    LED_COLOR_PINK
} led_color_e;

static inline struct led_rgb led_color_to_rgb(led_color_e color)
{
    switch (color)
    {
    case LED_COLOR_RED:
        return {255, 0, 0};
    case LED_COLOR_GREEN:
        return {0, 255, 0};
    case LED_COLOR_BLUE:
        return {0, 0, 255};
    case LED_COLOR_YELLOW:
        return {255, 255, 0};
    case LED_COLOR_CYAN:
        return {0, 255, 255};
    case LED_COLOR_MAGENTA:
        return {255, 0, 255};
    case LED_COLOR_WHITE:
        return {255, 255, 255};
    case LED_COLOR_PURPLE:
        return {128, 0, 128};
    case LED_COLOR_ORANGE:
        return {255, 128, 0};
    case LED_COLOR_PINK:
        return {255, 192, 203};
    default:
        return {0, 0, 0};
    }
}   

int Init_ws2812(void)
{
    ws2812_dev = DEVICE_DT_GET(LED_WS2812_NODE);
    if (!device_is_ready(ws2812_dev)) {
        LOG_ERR("WS2812 device is not ready");
        return -ENODEV;
    }

    auto num_leds = led_strip_length(ws2812_dev);
    if (num_leds == 0) {
        LOG_ERR("WS2812 device has zero length");
        return -EINVAL;
    }   

    pixels = (struct led_rgb *)k_malloc(sizeof(struct led_rgb) * num_leds);
    if (pixels == NULL) {
        LOG_ERR("Failed to allocate WS2812 pixels buffer");
        return -ENOMEM;
    }

    for (uint8_t i = 0; i < num_leds; i++) {
        pixels[i].r = 0;
        pixels[i].g = 0;
        pixels[i].b = 0;
    }

    LOG_INF("WS2812 initialized with %u LEDs", num_leds);
    return 0;
}

int set_led_color(const struct device *ws2812_dev, uint8_t index,  led_color_e color)
{
    auto num_leds = led_strip_length(ws2812_dev);
    struct led_rgb rgb = led_color_to_rgb(color);
    uint8_t r = rgb.r;
    uint8_t g = rgb.g;
    uint8_t b = rgb.b;
    if (index < num_leds) {
        pixels[index].r = r;
        pixels[index].g = g;
        pixels[index].b = b;
    }
    else {
        LOG_ERR("LED index: %d out of range: %d", index, num_leds);
        return -EINVAL;
    }

    int ret = led_strip_update_rgb(ws2812_dev, pixels, num_leds);
    if (ret < 0) {
        LOG_ERR("Failed to update LED color: %d", ret);
        return ret;
    }
    return 0;
}

void led_task_test(void *arg1, void *arg2, void *arg3)
{
    while (1)
    {
        set_led_color(ws2812_dev, 0, LED_COLOR_RED);
        k_sleep(K_MSEC(500));
        set_led_color(ws2812_dev, 0, LED_COLOR_GREEN);
        k_sleep(K_MSEC(500));
        set_led_color(ws2812_dev, 0, LED_COLOR_ORANGE);
        k_sleep(K_MSEC(500));
    }
}

void Create_led_task_test_thread(void)
{
    k_tid_t led_tid = k_thread_create(
        &led_thread_data,                        // 线程控制块
        led_stack_area,                          // 栈空间
        K_THREAD_STACK_SIZEOF(led_stack_area),   // 栈大小
        led_task_test,                               // 入口函数
        NULL, NULL, NULL,                        // 参数
        K_LED_TASK_PRIORITY,                // 优先级（数字越小优先级越高）
        K_USER,                                       // 线程选项
        K_NO_WAIT                                // 启动
    );
    k_thread_name_set(led_tid, "led_task_test");
    if (led_tid == NULL){
        LOG_ERR("Failed to create led_task_test thread");
    }
}

int Init_accel(void)
{
    accel_dev = DEVICE_DT_GET(IMU_ACCEL_NODE);
    if (!device_is_ready(accel_dev)) {
        LOG_ERR("Accelerometer device is not ready");
        return -ENODEV;
    }
    LOG_INF("Accelerometer device %p name is %s", accel_dev, accel_dev->name);
    return 0;
}

void accel_task(void *arg1, void *arg2, void *arg3)
{
    struct sensor_value acc[3];
    while (true) {
        sensor_sample_fetch(accel_dev);
        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
        LOG_INF("Accelerometer: X=%d.%06d m/s^2, Y=%d.%06d m/s^2, Z=%d.%06d m/s^2",
        acc[0].val1, acc[0].val2, acc[1].val1, acc[1].val2, acc[2].val1, acc[2].val2);
        k_msleep(1000);
    }
}

void Create_accel_task_thread(void)
{
    k_tid_t accel_tid = k_thread_create(
        &accel_thread_data,                        // 线程控制块
        accel_stack_area,                          // 栈空间
        K_THREAD_STACK_SIZEOF(accel_stack_area),   // 栈大小
        accel_task,                               // 入口函数
        NULL, NULL, NULL,                        // 参数
        K_ACCEL_TASK_PRIORITY,                // 优先级（数字越小优先级越高）
        K_USER,                                       // 线程选项
        K_NO_WAIT                                // 启动
    );
    k_thread_name_set(accel_tid, "accel_task");
    if (accel_tid == NULL){
        LOG_ERR("Failed to create accel_task thread");
    }
}

int main() {
    Init_ws2812();
    Init_accel();
    Create_led_task_test_thread();
    Create_accel_task_thread();

    while (true) {
        k_msleep(500);
    }
}
