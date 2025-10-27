#include "ina3221.h"
#include "esp_log.h"

static const char *TAG = "ina3221";

#define INA3221_I2C_TIMEOUT 100
#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf, but device supports up to 2.44Mhz

#define INA3221_REG_CONFIG                      (0x00)
#define INA3221_REG_SHUNTVOLTAGE_1              (0x01)
#define INA3221_REG_BUSVOLTAGE_1                (0x02)
#define INA3221_REG_CRITICAL_ALERT_1            (0x07)
#define INA3221_REG_WARNING_ALERT_1             (0x08)
#define INA3221_REG_SHUNT_VOLTAGE_SUM           (0x0D)
#define INA3221_REG_SHUNT_VOLTAGE_SUM_LIMIT     (0x0E)
#define INA3221_REG_MASK                        (0x0F)
#define INA3221_REG_VALID_POWER_UPPER_LIMIT     (0x10)
#define INA3221_REG_VALID_POWER_LOWER_LIMIT     (0x11)

#define RETURN_ON_ERROR(x) do {        \
    esp_err_t __err_rc = (x);          \
    if (__err_rc != ESP_OK) return __err_rc; \
} while (0)

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static esp_err_t ina3221_read(ina3221_handle_t *handle, const uint8_t reg, uint16_t *const val)
{
    CHECK_ARG(val);

    RETURN_ON_ERROR(i2c_master_transmit_recive(handle->i2c_master_dev_handle, (uint8_t[]){reg}, 1, val, 2, INA3221_I2C_TIMEOUT));

    *val = (*val >> 8) | (*val << 8);  // Swap

    return ESP_OK;
}

static esp_err_t ina3221_write(ina3221_handle_t *handle, uint8_t reg, uint16_t val)
{
    CHECK_ARG(val);
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = val & 0xFF;
    buf[2] = (val>>8) & 0xFF;

    RETURN_ON_ERROR(i2c_master_transmit(handle->i2c_master_dev_handle, buf, 3, INA3221_I2C_TIMEOUT));

    return ESP_OK;
}

static esp_err_t write_config(ina3221_handle_t *handle)
{
    return ina3221_write(handle, INA3221_REG_CONFIG, handle->config.config_register);
}

static esp_err_t write_mask(ina3221_handle_t *handle)
{
    return ina3221_write(handle, INA3221_REG_MASK, handle->mask.mask_register & INA3221_MASK_CONFIG);
}

///////////////////////////////////////////////////////////////////////////////////

esp_err_t ina3221_init(ina3221_handle_t *handle, uint8_t addr, i2c_master_bus_handle_t *i2c_bus)
{
    CHECK_ARG(handle && addr && i2c_bus);
    memset(handle, 0, sizeof(ina3221_handle_t));

    if (addr < INA3221_I2C_ADDR_GND || addr > INA3221_I2C_ADDR_SCL)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ
    };

    RETURN_ON_ERROR(i2c_master_bus_add_device(*i2c_bus, &dev_cfg, &handle->i2c_master_dev_handle));
    handle->i2c_master_bus_handle = *i2c_bus;
    return ESP_OK;
}

esp_err_t ina3221_sync(ina3221_handle_t *handle)
{
    CHECK_ARG(handle);

    uint16_t data;

    // Sync config register
    RETURN_ON_ERROR(ina3221_read(handle, INA3221_REG_CONFIG, &data));
    if (data != handle->config.config_register)
        RETURN_ON_ERROR(write_config(handle));

    // Sync mask register
    RETURN_ON_ERROR(ina3221_read(handle, INA3221_REG_MASK, &data));
    if ((data & INA3221_MASK_CONFIG) != (handle->mask.mask_register & INA3221_MASK_CONFIG))
        RETURN_ON_ERROR(write_mask(handle));

    return ESP_OK;
}

esp_err_t ina3221_handle_trigger(ina3221_handle_t *handle)
{
    return write_config(handle);
}

esp_err_t ina3221_get_status(ina3221_handle_t *handle)
{
    return ina3221_read(handle, INA3221_REG_MASK, &handle->mask.mask_register);
}

esp_err_t ina3221_set_options(ina3221_handle_t *handle, bool mode, bool bus, bool shunt)
{
    CHECK_ARG(handle);

    handle->config.mode = mode;
    handle->config.ebus = bus;
    handle->config.esht = shunt;
    return write_config(handle);
}

esp_err_t ina3221_enable_channel(ina3221_handle_t *handle, bool ch1, bool ch2, bool ch3)
{
    CHECK_ARG(handle);

    handle->config.ch1 = ch1;
    handle->config.ch2 = ch2;
    handle->config.ch3 = ch3;
    return write_config(handle);
}

esp_err_t ina3221_enable_channel_sum(ina3221_handle_t *handle, bool ch1, bool ch2, bool ch3)
{
    CHECK_ARG(handle);

    handle->mask.scc1 = ch1;
    handle->mask.scc2 = ch2;
    handle->mask.scc3 = ch3;
    return write_mask(handle);
}

esp_err_t ina3221_enable_latch_pin(ina3221_handle_t *handle, bool warning, bool critical)
{
    CHECK_ARG(handle);

    handle->mask.wen = warning;
    handle->mask.cen = critical;
    return write_mask(handle);
}

esp_err_t ina3221_set_average(ina3221_handle_t *handle, ina3221_avg_t avg)
{
    CHECK_ARG(handle);

    handle->config.avg = avg;
    return write_config(handle);
}

esp_err_t ina3221_set_bus_conversion_time(ina3221_handle_t *handle, ina3221_ct_t ct)
{
    CHECK_ARG(handle);

    handle->config.vbus = ct;
    return write_config(handle);
}

esp_err_t ina3221_set_shunt_conversion_time(ina3221_handle_t *handle, ina3221_ct_t ct)
{
    CHECK_ARG(handle);

    handle->config.vsht = ct;
    return write_config(handle);
}

esp_err_t ina3221_reset(ina3221_handle_t *handle)
{
    CHECK_ARG(handle);

    handle->config.config_register = INA3221_DEFAULT_CONFIG;
    handle->mask.mask_register = INA3221_DEFAULT_CONFIG;
    handle->config.rst = 1;
    return write_config(handle);
}

esp_err_t ina3221_get_bus_voltage(ina3221_handle_t *handle, ina3221_channel_t channel, float *voltage)
{
    CHECK_ARG(handle && voltage);

    int16_t raw;

    RETURN_ON_ERROR(ina3221_read(handle, INA3221_REG_BUSVOLTAGE_1 + channel * 2, (uint16_t *)&raw));
    *voltage = raw * 0.001;

    return ESP_OK;
}

esp_err_t ina3221_get_shunt_value(ina3221_handle_t *handle, ina3221_channel_t channel, float *voltage, float *current)
{
    CHECK_ARG(handle);
    CHECK_ARG(voltage || current);
    if (current && !handle->shunt[channel])
    {
        ESP_LOGE(TAG, "No shunt configured for channel %u in device [0x%02x at %d]", channel, handle->dev_addr);
        return ESP_ERR_INVALID_ARG;
    }

    int16_t raw;
    RETURN_ON_ERROR(ina3221_read(handle, INA3221_REG_SHUNTVOLTAGE_1 + channel * 2, (uint16_t *)&raw));
    float mvolts = raw * 0.005; // mV, 40uV step

    if (voltage)
        *voltage = mvolts;

    if (current)
        *current = mvolts * 1000.0 / handle->shunt[channel];  // mA

    return ESP_OK;
}

esp_err_t ina3221_get_sum_shunt_value(ina3221_handle_t *handle, float *voltage)
{
    CHECK_ARG(handle && voltage);

    int16_t raw;

    RETURN_ON_ERROR(ina3221_read(handle, INA3221_REG_SHUNT_VOLTAGE_SUM, (uint16_t *)&raw));
    *voltage = raw * 0.02; // mV

    return ESP_OK;
}

esp_err_t ina3221_set_critical_alert(ina3221_handle_t *handle, ina3221_channel_t channel, float current)
{
    CHECK_ARG(handle);

    int16_t raw = current * handle->shunt[channel] * 0.2;
    return ina3221_write(handle, INA3221_REG_CRITICAL_ALERT_1 + channel * 2, *(uint16_t *)&raw);
}

esp_err_t ina3221_set_warning_alert(ina3221_handle_t *handle, ina3221_channel_t channel, float current)
{
    CHECK_ARG(handle);

    int16_t raw = current * handle->shunt[channel] * 0.2;
    return ina3221_write(handle, INA3221_REG_WARNING_ALERT_1 + channel * 2, *(uint16_t *)&raw);
}

esp_err_t ina3221_set_sum_warning_alert(ina3221_handle_t *handle, float voltage)
{
    CHECK_ARG(handle);

    int16_t raw = voltage * 50.0;
    return ina3221_write(handle, INA3221_REG_SHUNT_VOLTAGE_SUM_LIMIT, *(uint16_t *)&raw);
}

esp_err_t ina3221_set_power_valid_upper_limit(ina3221_handle_t *handle, float voltage)
{
    CHECK_ARG(handle);
    if (!handle->config.ebus)
    {
        ESP_LOGE(TAG, "Bus is not enabled in device [0x%02x at %d]", handle->dev_addr);
        return ESP_ERR_NOT_SUPPORTED;
    }

    int16_t raw = voltage * 1000.0;
    return ina3221_write(handle, INA3221_REG_VALID_POWER_UPPER_LIMIT, *(uint16_t *)&raw);
}

esp_err_t ina3221_set_power_valid_lower_limit(ina3221_handle_t *handle, float voltage)
{
    CHECK_ARG(handle);
    if (!handle->config.ebus)
    {
        ESP_LOGE(TAG, "Bus is not enabled in device [0x%02x at %d]", handle->dev_addr);
        return ESP_ERR_NOT_SUPPORTED;
    }

    int16_t raw = voltage * 1000.0;
    return ina3221_write(handle, INA3221_REG_VALID_POWER_LOWER_LIMIT, *(uint16_t *)&raw);
}