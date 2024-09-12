#define DT_DRV_COMPAT pimoroni_pim447

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

#include "pim447.h"

LOG_MODULE_REGISTER(pim447, CONFIG_SENSOR_LOG_LEVEL);

struct pim447_config {
    const struct device *i2c_dev;
    uint8_t i2c_addr;
};

void write_led_brightness(const struct device *dev, uint8_t brightness_value) {

    const struct pim447_config *config = dev->config;

    if (!dev) {
        printk("I2C: Device not found\n");
        return;
    }

    uint8_t reg = 0x03;  // LED control register
    uint8_t data[2];

    data[0] = reg;              // First byte is the register address
    data[1] = brightness_value; // Second byte is the value to write (brightness)

    // Perform the I2C write
    int ret = i2c_write(dev, data, sizeof(data), config->i2c_addr);
    if (ret != 0) {
        printk("Error writing to the trackball: %d\n", ret);
    } else {
        printk("LED brightness set to %d\n", brightness_value);
    }
}



int pim447_init(const struct device *dev)
{
    const struct pim447_config *config = dev->config;


    LOG_INF("PIM447 I2C device: %s", config->i2c_dev->name);
    LOG_INF("PIM447 I2C address: 0x%02x", config->i2c_addr);
    LOG_INF("PIM447 initialized");

    write_led_brightness(i2c_dev, 150);

    return 0;
}

#define PIM447_INIT(n)                                              \
    static const struct pim447_config pim447_cfg_##n = {            \
        .i2c_dev = DEVICE_DT_GET(DT_INST_BUS(n)),                   \
        .i2c_addr = DT_INST_REG_ADDR(n),                            \
    };                                                              \
    DEVICE_DT_INST_DEFINE(n, pim447_init, NULL,                     \
                          NULL, &pim447_cfg_##n,                    \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(PIM447_INIT)