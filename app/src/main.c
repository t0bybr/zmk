/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/settings/settings.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/display.h>

int main(void) {
    printk("Starting PIM447 LED test\n");

    int ret = test_pim447_led();
    if (ret == 0) {
        printk("PIM447 LED test successful\n");
    } else {
        printk("PIM447 LED test failed\n");
    }
    LOG_INF("Welcome to ZMK!\n");

#if IS_ENABLED(CONFIG_SETTINGS)
    settings_subsys_init();
    settings_load();
#endif

#ifdef CONFIG_ZMK_DISPLAY
    zmk_display_init();
#endif /* CONFIG_ZMK_DISPLAY */

    return 0;
}

#include <zephyr/drivers/i2c.h>

#define PIM447_I2C_ADDR 0x0A
#define PIM447_LED_WHITE_REG 0x03

int test_pim447_led(void)
{
    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return -ENODEV;
    }

    int ret = i2c_reg_write_byte(i2c_dev, PIM447_I2C_ADDR, PIM447_LED_WHITE_REG, 200);
    if (ret != 0) {
        printk("Failed to write to PIM447 LED register: %d\n", ret);
        return ret;
    }

    printk("Successfully wrote to PIM447 LED register\n");
    return 0;
}
