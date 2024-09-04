#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <behaviors/behavior_key_press.h>
#include <logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define PIM447_I2C_ADDR 0x0A
#define PIM447_LED_WHITE_REG 0x03

static int on_pim447_test(struct zmk_behavior_binding *binding,
                          struct zmk_behavior_binding_event event)
{
    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    int ret = i2c_reg_write_byte(i2c_dev, PIM447_I2C_ADDR, PIM447_LED_WHITE_REG, 200);
    if (ret != 0) {
        LOG_ERR("Failed to write to PIM447 LED register: %d", ret);
        return ret;
    }

    LOG_INF("Successfully wrote to PIM447 LED register");
    return ZMK_BEHAVIOR_OPAQUE;
}

ZMK_BEHAVIOR_IMPL(pim447_test, NULL, on_pim447_test);