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



int pim447_init(const struct device *dev)
{
    const struct pim447_config *config = dev->config;


    LOG_INF("PIM447 I2C device: %s", config->i2c_dev->name);
    LOG_INF("PIM447 I2C address: 0x%02x", config->i2c_addr);
    LOG_INF("PIM447 initialized");

    if (!device_is_ready(config->i2c_dev)) {
        LOG_ERR("I2C device %s not ready", config->i2c_dev->name);
        return;
    }

    uint8_t data[2];
    data[0] = 0x03;
    data[1] = 150;

    // Optional: Introduce a short delay to avoid timing issues
    k_msleep(1);

    // Perform the I2C write and check for errors
    int ret = i2c_write(config->i2c_dev, data, sizeof(data), config->i2c_addr);
    if (ret != 0) {
        LOG_ERR("I2C write failed with error %d", ret);
    } else {
        LOG_INF("LED brightness set to %d", 150);
    }

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