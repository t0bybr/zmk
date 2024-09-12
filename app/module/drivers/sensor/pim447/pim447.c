#define DT_DRV_COMPAT pimoroni_pim447

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

#include "pim447.h"

LOG_MODULE_REGISTER(pim447, CONFIG_SENSOR_LOG_LEVEL);

struct pim447_config {
    const struct device *i2c_dev;
    uint8_t i2c_addr;
};

void test_basic_i2c_write(void) {
    const struct device *i2c_dev = device_get_binding("I2C_1");  // Adjust I2C bus name
    uint8_t reg = 0x03;
    uint8_t brightness = 150;
    uint8_t data[2] = { reg, brightness };

    if (!i2c_dev) {
        printk("I2C device not found\n");
        return;
    }

    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return;
    }

    // Perform the I2C write
    int ret = i2c_write(i2c_dev, data, sizeof(data), 0x0A);  // Replace with your address
    if (ret != 0) {
        printk("I2C write failed: %d\n", ret);
    } else {
        printk("I2C write successful\n");
    }
}


int pim447_init(const struct device *dev)
{
    const struct pim447_config *config = dev->config;


    LOG_INF("PIM447 I2C device: %s", config->i2c_dev->name);
    LOG_INF("PIM447 I2C address: 0x%02x", config->i2c_addr);
    LOG_INF("PIM447 initialized");

    test_basic_i2c_write();


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