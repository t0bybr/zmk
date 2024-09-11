#define DT_DRV_COMPAT pimoroni_pim447

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include "pim447.h"

LOG_MODULE_REGISTER(pim447, CONFIG_SENSOR_LOG_LEVEL);

struct pim447_config {
    struct i2c_dt_spec i2c;
    const struct device *i2c_dev;
    uint8_t i2c_addr;
};

static int write_register(const struct device *dev, uint8_t reg, uint16_t value)
{
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }

    const struct pim447_config *config = dev->config;

    uint8_t data[2] = {0};
    sys_put_be16(value, &data[0]);

    return i2c_burst_write_dt(&config->i2c, reg, &data[0], sizeof(data));
}

int pim447_init(const struct device *dev)
{
    const struct pim447_config *config = dev->config;

    // Using the new write_register function
    int ret = write_register(dev, 0x03, 200);
    if (ret < 0) {
        LOG_ERR("Failed to initialize PIM447");
        return ret;
    }

    LOG_INF("PIM447 I2C device: %s", config->i2c_dev->name);
    LOG_INF("PIM447 I2C address: 0x%02x", config->i2c_addr);
    LOG_INF("PIM447 initialized");

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