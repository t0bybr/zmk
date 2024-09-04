#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include "pim447.h"

LOG_MODULE_REGISTER(pim447, CONFIG_SENSOR_LOG_LEVEL);

struct pim447_config {
    const struct device *i2c_dev;
    uint8_t i2c_addr;
};

struct pim447_data {
    struct k_work_delayable init_work;
};

static void i2c_write_complete(const struct device *dev, int status, void *userdata)
{
    if (status == 0) {
        LOG_INF("I2C write completed successfully");
    } else {
        LOG_ERR("I2C write failed with status: %d", status);
    }
}

static void pim447_init_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct pim447_data *data = CONTAINER_OF(dwork, struct pim447_data, init_work);
    const struct device *dev = CONTAINER_OF(data, struct device, data);
    const struct pim447_config *config = dev->config;

    uint8_t led_on_cmd[] = {0x03, 200};

    int ret = i2c_write_async(config->i2c_dev, led_on_cmd, sizeof(led_on_cmd),
                              config->i2c_addr, i2c_write_complete, NULL);
    if (ret < 0) {
        LOG_ERR("Failed to initiate asynchronous I2C write: %d", ret);
    } else {
        LOG_INF("Asynchronous I2C write initiated");
    }
}

int pim447_init(const struct device *dev)
{
    const struct pim447_config *config = dev->config;
    struct pim447_data *data = dev->data;

    LOG_INF("PIM447 I2C device: %s", config->i2c_dev->name);
    LOG_INF("PIM447 I2C address: 0x%02x", config->i2c_addr);

    k_work_init_delayable(&data->init_work, pim447_init_work_handler);
    k_work_schedule(&data->init_work, K_MSEC(100));  // Schedule the work after 100ms

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