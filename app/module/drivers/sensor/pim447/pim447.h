#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

int write_i2c_register(uint8_t reg_addr, uint8_t data);
int pim447_init(const struct device *dev);