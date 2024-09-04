
#define DT_DRV_COMPAT pimoroni_pim447

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "pim447.h"



// Define and register the module log
LOG_MODULE_REGISTER(pim447, CONFIG_SENSOR_LOG_LEVEL);

// Register addresses
enum {
    REG_LED_RED     = 0x00,
    REG_LED_GRN     = 0x01,
    REG_LED_BLU     = 0x02,
    REG_LED_WHT     = 0x03,
    REG_LEFT        = 0x04,
    REG_RIGHT       = 0x05,
    REG_UP          = 0x06,
    REG_DOWN        = 0x07,
    REG_SWITCH      = 0x08,
    REG_USER_FLASH  = 0xD0,
    REG_FLASH_PAGE  = 0xF0,
    REG_INT         = 0xF9,
    REG_CHIP_ID_L   = 0xFA,
    REG_CHIP_ID_H   = 0xFB,
    REG_VERSION     = 0xFC,
    REG_I2C_ADDR    = 0xFD,
    REG_CTRL        = 0xFE,
};

#define MSK_SWITCH_STATE 0x80

#define MSK_INT_TRIGGERED 0x01
#define MSK_INT_OUT_EN 0x02

#define MSK_CTRL_SLEEP 0x01
#define MSK_CTRL_RESET 0x02
#define MSK_CTRL_FREAD 0x04
#define MSK_CTRL_FWRITE 0x08

// Define the driver API
static const struct sensor_driver_api pim447_driver_api = {
    .sample_fetch = pim447_sample_fetch,
    .channel_get = pim447_channel_get,
    // .init = pim447_init,
    // .read = pim447_read,
    // .set_rgbw = pim447_set_rgbw,
};


int pim447_init(const struct device *dev) {
    // struct pim447_data *drv_data = dev->data;
    const struct pim447_config *drv_cfg = dev->config;
    uint8_t chip_id_h, chip_id_l;
    uint16_t chip_id;

    LOG_INF("PIM447 init");


    // Read chip ID
    if (i2c_reg_read_byte(drv_cfg->i2c_dev, drv_cfg->i2c_addr, REG_CHIP_ID_H, &chip_id_h) ||
        i2c_reg_read_byte(drv_cfg->i2c_dev, drv_cfg->i2c_addr, REG_CHIP_ID_L, &chip_id_l)) {
        LOG_ERR("Failed to read chip ID");
        return -EIO;
    }

    chip_id = (chip_id_h << 8) | chip_id_l;

    if (chip_id != CHIP_ID) {
        LOG_ERR("Invalid chip ID: 0x%04x", chip_id);
        return -ENODEV;
    }

    pim447_set_rgbw(dev, 0, 0, 0, 100);

    // Enable interrupt
    uint8_t int_val;
    i2c_reg_read_byte(config->i2c_dev, config->i2c_addr, REG_INT, &int_val);
    int_val |= MSK_INT_OUT_EN;
    i2c_reg_write_byte(config->i2c_dev, config->i2c_addr, REG_INT, int_val);

    return 0;
}

static int pim447_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    // struct pim447_data *data = dev->data;
    const struct pim447_config *config = dev->config;

    // Implement the logic to fetch samples from the sensor
    // This is just a placeholder, you'll need to implement the actual logic
    return 0;
}

static int pim447_channel_get(const struct device *dev, enum sensor_channel chan,
                              struct sensor_value *val)
{
    // struct pim447_data *data = dev->data;
    const struct pim447_config *config = dev->config;

    // Implement the logic to get channel data
    // This is just a placeholder, you'll need to implement the actual logic
    return 0;
}

// static int pim447_read(const struct device *dev, struct pim447_state *state)
// {
//     struct pim447_data *data = dev->data;
//     const struct pim447_config *config = dev->config;
//     uint8_t sw_state;

//     i2c_reg_read_byte(config->i2c_dev, config->i2c_addr, REG_LEFT, &state->left);
//     i2c_reg_read_byte(config->i2c_dev, config->i2c_addr, REG_RIGHT, &state->right);
//     i2c_reg_read_byte(config->i2c_dev, config->i2c_addr, REG_UP, &state->up);
//     i2c_reg_read_byte(config->i2c_dev, config->i2c_addr, REG_DOWN, &state->down);
//     i2c_reg_read_byte(config->i2c_dev, config->i2c_addr, REG_SWITCH, &sw_state);

//     state->sw_changed = sw_state & ~MSK_SWITCH_STATE;
//     state->sw_pressed = (sw_state & MSK_SWITCH_STATE) > 0;

//     return 0;
// }

int pim447_set_rgbw(const struct device *dev, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    // struct pim447_data *data = dev->data;
    const struct pim447_config *config = dev->config;

    i2c_reg_write_byte(config->i2c_dev, config->i2c_addr, REG_LED_RED, r);
    i2c_reg_write_byte(config->i2c_dev, config->i2c_addr, REG_LED_GRN, g);
    i2c_reg_write_byte(config->i2c_dev, config->i2c_addr, REG_LED_BLU, b);
    i2c_reg_write_byte(config->i2c_dev, config->i2c_addr, REG_LED_WHT, w);

    return 0;
}


// Device definition
#define PIM447_INIT(n)                                              \
    static struct pim447_data pim447_data_##n;                      \
    static const struct pim447_config pim447_cfg_##n = {            \
        .i2c_dev = DEVICE_DT_GET(DT_INST_BUS(n)),                   \
        .i2c_addr = DT_INST_REG_ADDR(n),                            \
        .interrupt_gpio = DT_INST_GPIO_PIN(n, interrupt_gpios),     \
        .timeout = DT_INST_PROP_OR(n, timeout, DEFAULT_TIMEOUT),    \
    };                                                              \
    DEVICE_DT_INST_DEFINE(n, pim447_init, NULL,                     \
                          &pim447_data_##n, &pim447_cfg_##n,        \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
                          &pim447_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PIM447_INIT)
