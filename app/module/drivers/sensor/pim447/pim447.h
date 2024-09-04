#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

struct pim447_data {
    struct k_work_delayable init_work;
};

int pim447_init(const struct device *dev);