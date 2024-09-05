#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

int pim447_init(const struct device *dev);