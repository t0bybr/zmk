#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include "pim447.h"

LOG_MODULE_REGISTER(pim447, CONFIG_SENSOR_LOG_LEVEL);

struct pim447_config {
    const struct device *i2c_dev;
    uint8_t i2c_addr;
};

static int write_i2c_register(const struct device *dev, uint8_t reg_addr, uint8_t data)
{
    const struct pim447_config *config = dev->config;
    uint8_t buf[2];

    if (!device_is_ready(config->i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    buf[0] = reg_addr;
    buf[1] = data;

    return i2c_write(config->i2c_dev, buf, sizeof(buf), config->i2c_addr);
}

int pim447_init(const struct device *dev)
{
    const struct pim447_config *config = dev->config;

    LOG_ERR("PIM447 I2C device: %s", config->i2c_dev->name);
    LOG_ERR("PIM447 I2C address: 0x%02x", config->i2c_addr);

    // Delay for 10 seconds
    k_sleep(K_SECONDS(10));



    int ret = write_i2c_register(dev, 0x03, 200);
    if (ret != 0) {
        LOG_ERR("Failed to write to I2C register");
        return ret;
    }

    return 0;
}

#define PIM447_INIT(n)                                              \
    static const struct pim447_config pim447_cfg_##n = {            \
        .i2c_dev = DEVICE_DT_GET(DT_INST_BUS(n)),                   \
        .i2c_addr = DT_INST_REG_ADDR(n),                            \
    };                                                              \
    static struct pim447_data pim447_data_##n;                      \
    DEVICE_DT_INST_DEFINE(n, pim447_init, NULL,                     \
                          &pim447_data_##n, &pim447_cfg_##n,        \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(PIM447_INIT)