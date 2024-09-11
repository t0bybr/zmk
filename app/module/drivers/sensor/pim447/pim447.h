#pragma once

#include <zephyr/device.h>


int pim447_write_byte(const struct device *dev, uint8_t reg_addr, uint8_t data); 

int pim447_init(const struct device *dev);