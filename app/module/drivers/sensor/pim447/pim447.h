/* 2024 t0bybr */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#define DEFAULT_I2C_ADDRESS 0x0A
#define I2C_ADDRESS_ALTERNATIVE 0x0B
#define DEFAULT_TIMEOUT 5
#define CHIP_ID 0xBA11
#define VERSION 1

struct pim447_state {
    uint8_t left;
    uint8_t right;
    uint8_t up;
    uint8_t down;
    bool sw_changed;
    bool sw_pressed;
};

struct pim447_config {
    const struct device *i2c_dev;
    uint8_t i2c_addr;
    uint32_t timeout;
};

struct pim447_data {
    struct pim447_state state;
};


int pim447_init(const struct device *dev);

static int pim447_sample_fetch(const struct device *dev, enum sensor_channel chan);
static int pim447_channel_get(const struct device *dev, enum sensor_channel chan,
                              struct sensor_value *val);

int pim447_set_rgbw(const struct device *dev, uint8_t r, uint8_t g, uint8_t b, uint8_t w);
