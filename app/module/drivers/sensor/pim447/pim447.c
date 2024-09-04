#define DT_DRV_COMPAT pimoroni_pim447

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "pim447.h"

LOG_MODULE_REGISTER(pim447, CONFIG_SENSOR_LOG_LEVEL);

struct pim447_config {
    const struct device *i2c_dev;
    uint8_t i2c_addr;
};

static void i2c_write_complete(const struct device *dev, int status)
{
    if (status == 0) {
        LOG_INF("I2C write completed successfully");
    } else {
        LOG_ERR("I2C write failed with status: %d", status);
    }
}

int pim447_init(const struct device *dev)
{
    const struct pim447_config *config = dev->config;

    LOG_INF("PIM447 I2C device: %s", config->i2c_dev->name);
    LOG_INF("PIM447 I2C address: 0x%02x", config->i2c_addr);

    uint8_t led_on_cmd[] = {0x03, 200};
    struct i2c_msg msg = {
        .buf = led_on_cmd,
        .len = sizeof(led_on_cmd),
        .flags = I2C_MSG_WRITE | I2C_MSG_STOP
    };

    int ret = i2c_transfer(config->i2c_dev, &msg, 1, config->i2c_addr);
    if (ret < 0) {
        LOG_ERR("Failed to initiate I2C write: %d", ret);
        return ret;
    }

    LOG_INF("I2C write initiated");

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