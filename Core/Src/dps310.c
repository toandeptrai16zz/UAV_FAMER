#include "dps310.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* DPS310 register map */
#define REG_PRS_B2          0x00u
#define REG_TMP_B2          0x03u
#define REG_PRS_CFG         0x06u
#define REG_TMP_CFG         0x07u
#define REG_MEAS_CFG        0x08u
#define REG_CFG_REG         0x09u
#define REG_RESET           0x0Cu
#define REG_PRODUCT_ID      0x0Du
#define REG_COEF_START      0x10u
#define REG_TMP_COEF_SRCE   0x28u

#define MEAS_SENSOR_READY   (1u << 6)
#define MEAS_COEF_READY     (1u << 7)
#define MEAS_TMP_READY      (1u << 5)
#define MEAS_PRS_READY      (1u << 4)

#define MODE_BACKGROUND_PRS_TMP  0x07u
#define I2C_TIMEOUT_MS            5u
#define INIT_TIMEOUT_MS           200u
#define POLL_INTERVAL_MS          8u
#define ALTITUDE_ALPHA            0.20f

/* Runtime I2C reads use interrupt mode so the DShot/control loop is not
   blocked while waiting for the barometer bus. */
enum {
    DPS310_ASYNC_IDLE = 0u,
    DPS310_ASYNC_STATUS_IN_FLIGHT,
    DPS310_ASYNC_STATUS_DONE,
    DPS310_ASYNC_TEMP_IN_FLIGHT,
    DPS310_ASYNC_TEMP_DONE,
    DPS310_ASYNC_PRESS_IN_FLIGHT,
    DPS310_ASYNC_PRESS_DONE
};

static DPS310_HandleTypeDef *active_dps310;

static HAL_StatusTypeDef dps310_read_regs(DPS310_HandleTypeDef *dev,
                                          uint8_t reg,
                                          uint8_t *buffer,
                                          uint16_t length)
{
    if ((dev == NULL) || (dev->hi2c == NULL) || (buffer == NULL) ||
        (length == 0u)) {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(dev->hi2c,
                            (uint16_t)(dev->address_7bit << 1),
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buffer,
                            length,
                            I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef dps310_write_reg(DPS310_HandleTypeDef *dev,
                                           uint8_t reg,
                                           uint8_t value)
{
    if ((dev == NULL) || (dev->hi2c == NULL)) {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(dev->hi2c,
                             (uint16_t)(dev->address_7bit << 1),
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1u,
                             I2C_TIMEOUT_MS);
}

static int32_t sign_extend(uint32_t value, uint8_t bit_count)
{
    const uint32_t sign_bit = 1u << (bit_count - 1u);
    const uint32_t modulus = 1u << bit_count;

    if ((value & sign_bit) != 0u) {
        return (int32_t)value - (int32_t)modulus;
    }
    return (int32_t)value;
}

static int32_t parse_s24(const uint8_t *b)
{
    uint32_t value = ((uint32_t)b[0] << 16) |
                     ((uint32_t)b[1] << 8) |
                     (uint32_t)b[2];
    return sign_extend(value, 24u);
}

static bool read_coefficients(DPS310_HandleTypeDef *dev, uint8_t *temp_source)
{
    uint8_t c[18];
    uint8_t source;

    if (dps310_read_regs(dev, REG_COEF_START, c, sizeof(c)) != HAL_OK) {
        dev->error_count++;
        return false;
    }

    /* c0 and c1: 12-bit signed values. */
    dev->c0 = sign_extend(((uint32_t)c[0] << 4) | (c[1] >> 4), 12u);
    dev->c1 = sign_extend(((uint32_t)(c[1] & 0x0Fu) << 8) | c[2], 12u);

    /* c00 and c10: 20-bit signed values. */
    dev->c00 = sign_extend(((uint32_t)c[3] << 12) |
                           ((uint32_t)c[4] << 4) |
                           ((uint32_t)c[5] >> 4), 20u);
    dev->c10 = sign_extend(((uint32_t)(c[5] & 0x0Fu) << 16) |
                           ((uint32_t)c[6] << 8) |
                           c[7], 20u);

    /* Remaining compensation coefficients are 16-bit signed values. */
    dev->c01 = sign_extend(((uint32_t)c[8] << 8) | c[9], 16u);
    dev->c11 = sign_extend(((uint32_t)c[10] << 8) | c[11], 16u);
    dev->c20 = sign_extend(((uint32_t)c[12] << 8) | c[13], 16u);
    dev->c21 = sign_extend(((uint32_t)c[14] << 8) | c[15], 16u);
    dev->c30 = sign_extend(((uint32_t)c[16] << 8) | c[17], 16u);

    if (dps310_read_regs(dev, REG_TMP_COEF_SRCE, &source, 1u) != HAL_OK) {
        dev->error_count++;
        return false;
    }

    *temp_source = (uint8_t)((source >> 7) & 0x01u);
    return true;
}

static float compensate_temperature(const DPS310_HandleTypeDef *dev,
                                    int32_t raw_temperature)
{
    const float scaled = (float)raw_temperature / dev->temperature_scale;
    return ((float)dev->c0 * 0.5f) + ((float)dev->c1 * scaled);
}

static float compensate_pressure(const DPS310_HandleTypeDef *dev,
                                 int32_t raw_pressure,
                                 int32_t raw_temperature)
{
    const float p = (float)raw_pressure / dev->pressure_scale;
    const float t = (float)raw_temperature / dev->temperature_scale;
    return (float)dev->c00 +
           p * ((float)dev->c10 + p * ((float)dev->c20 + p * (float)dev->c30)) +
           t * (float)dev->c01 +
           t * p * ((float)dev->c11 + p * (float)dev->c21);
}

static void update_relative_altitude(DPS310_HandleTypeDef *dev)
{
    float ratio;
    float altitude_raw;

    if (!dev->altitude_reference_valid || (dev->reference_pressure_pa <= 0.0f) ||
        (dev->pressure_pa <= 0.0f)) {
        dev->altitude_valid = false;
        dev->altitude_m = 0.0f;
        dev->altitude_filtered_m = 0.0f;
        return;
    }

    ratio = dev->pressure_pa / dev->reference_pressure_pa;
    altitude_raw = 44330.0f * (1.0f - powf(ratio, 0.19029495f));
    dev->altitude_m = altitude_raw;

    if (!dev->altitude_filter_initialized) {
        dev->altitude_filtered_m = altitude_raw;
        dev->altitude_filter_initialized = true;
    } else {
        dev->altitude_filtered_m += ALTITUDE_ALPHA *
            (altitude_raw - dev->altitude_filtered_m);
    }
    dev->altitude_valid = true;
}

bool DPS310_Init(DPS310_HandleTypeDef *dev,
                 I2C_HandleTypeDef *hi2c,
                 uint8_t address_7bit)
{
    uint8_t product_id = 0u;
    uint8_t status = 0u;
    uint8_t temp_source = 0u;
    uint32_t start_ms;
    uint8_t temp_cfg;

    if ((dev == NULL) || (hi2c == NULL) ||
        ((address_7bit != DPS310_I2C_ADDR_DEFAULT) &&
         (address_7bit != DPS310_I2C_ADDR_ALT))) {
        return false;
    }

    if (active_dps310 == dev) {
        active_dps310 = NULL;
    }
    memset(dev, 0, sizeof(*dev));
    dev->hi2c = hi2c;
    dev->address_7bit = address_7bit;

    if (HAL_I2C_IsDeviceReady(hi2c,
                              (uint16_t)(address_7bit << 1),
                              3u,
                              20u) != HAL_OK) {
        dev->error_count++;
        return false;
    }

    /* Wait until the internal sensor and factory coefficients are ready. */
    start_ms = HAL_GetTick();
    do {
        if (dps310_read_regs(dev, REG_MEAS_CFG, &status, 1u) != HAL_OK) {
            dev->error_count++;
            return false;
        }
        if ((status & (MEAS_SENSOR_READY | MEAS_COEF_READY)) ==
            (MEAS_SENSOR_READY | MEAS_COEF_READY)) {
            break;
        }
        HAL_Delay(1u);
    } while ((HAL_GetTick() - start_ms) < INIT_TIMEOUT_MS);

    if ((status & (MEAS_SENSOR_READY | MEAS_COEF_READY)) !=
        (MEAS_SENSOR_READY | MEAS_COEF_READY)) {
        dev->error_count++;
        return false;
    }

    if (dps310_read_regs(dev, REG_PRODUCT_ID, &product_id, 1u) != HAL_OK) {
        dev->error_count++;
        return false;
    }

    /* Register 0x0D: PROD_ID is the low nibble (DPS310 = 0x0);
       REV_ID is the high nibble and may vary between silicon revisions. */
    if ((product_id & 0x0Fu) != 0x00u) {
        dev->error_count++;
        return false;
    }

    if (!read_coefficients(dev, &temp_source)) {
        return false;
    }

    /* OSR 8x pressure -> kP=7,864,320; OSR 1x temperature -> kT=524,288. */
    dev->pressure_scale = 7864320.0f;
    dev->temperature_scale = 524288.0f;

    /* 64 pressure samples/s, OSR=8x. */
    if (dps310_write_reg(dev, REG_PRS_CFG, 0x63u) != HAL_OK) {
        dev->error_count++;
        return false;
    }

    /* 1 temperature sample/s, OSR=1x. TMP_EXT follows the factory source bit. */
    temp_cfg = (temp_source != 0u) ? 0x80u : 0x00u;
    if (dps310_write_reg(dev, REG_TMP_CFG, temp_cfg) != HAL_OK) {
        dev->error_count++;
        return false;
    }

    /* No result shift is needed for OSR <= 8x; FIFO and IRQ routing stay off. */
    if (dps310_write_reg(dev, REG_CFG_REG, 0x00u) != HAL_OK) {
        dev->error_count++;
        return false;
    }

    /* Continuous pressure + temperature background measurements. */
    if (dps310_write_reg(dev, REG_MEAS_CFG, MODE_BACKGROUND_PRS_TMP) != HAL_OK) {
        dev->error_count++;
        return false;
    }

    dev->async_state = DPS310_ASYNC_IDLE;
    dev->last_poll_ms = HAL_GetTick();
    dev->initialized = true;
    active_dps310 = dev;
    return true;
}

static bool start_async_read(DPS310_HandleTypeDef *dev,
                             uint8_t reg,
                             uint8_t *buffer,
                             uint16_t length,
                             uint8_t in_flight_state)
{
    HAL_StatusTypeDef status;

    dev->async_state = in_flight_state;
    status = HAL_I2C_Mem_Read_IT(dev->hi2c,
                                 (uint16_t)(dev->address_7bit << 1),
                                 reg,
                                 I2C_MEMADD_SIZE_8BIT,
                                 buffer,
                                 length);
    if (status != HAL_OK) {
        dev->error_count++;
        dev->async_state = DPS310_ASYNC_IDLE;
        return false;
    }
    return true;
}

bool DPS310_Update(DPS310_HandleTypeDef *dev)
{
    bool pressure_updated = false;
    uint32_t now;

    if ((dev == NULL) || !dev->initialized) {
        return false;
    }

    /* Consume completed interrupt-driven transactions in thread context. */
    __DMB();
    switch (dev->async_state) {
    case DPS310_ASYNC_STATUS_DONE:
        if ((dev->async_status & MEAS_TMP_READY) != 0u) {
            (void)start_async_read(dev, REG_TMP_B2,
                                   dev->raw_temperature_bytes, 3u,
                                   DPS310_ASYNC_TEMP_IN_FLIGHT);
        } else if ((dev->async_status & MEAS_PRS_READY) != 0u) {
            (void)start_async_read(dev, REG_PRS_B2,
                                   dev->raw_pressure_bytes, 3u,
                                   DPS310_ASYNC_PRESS_IN_FLIGHT);
        } else {
            dev->async_state = DPS310_ASYNC_IDLE;
        }
        break;

    case DPS310_ASYNC_TEMP_DONE:
        dev->raw_temperature = parse_s24(dev->raw_temperature_bytes);
        dev->temperature_c = compensate_temperature(dev, dev->raw_temperature);
        dev->temperature_valid = isfinite(dev->temperature_c);
        if ((dev->async_status & MEAS_PRS_READY) != 0u) {
            (void)start_async_read(dev, REG_PRS_B2,
                                   dev->raw_pressure_bytes, 3u,
                                   DPS310_ASYNC_PRESS_IN_FLIGHT);
        } else {
            dev->async_state = DPS310_ASYNC_IDLE;
        }
        break;

    case DPS310_ASYNC_PRESS_DONE:
    {
        int32_t raw_pressure = parse_s24(dev->raw_pressure_bytes);

        dev->async_state = DPS310_ASYNC_IDLE;
        if (!dev->temperature_valid) {
            /* Wait for the first valid temperature sample for compensation. */
            break;
        }

        dev->pressure_pa = compensate_pressure(dev,
                                                raw_pressure,
                                                dev->raw_temperature);
        if (!isfinite(dev->pressure_pa) ||
            (dev->pressure_pa < 30000.0f) ||
            (dev->pressure_pa > 120000.0f)) {
            dev->error_count++;
            break;
        }

        dev->pressure_valid = true;
        pressure_updated = true;

        /* Average the first 64 valid samples as the local ground reference.
           Keep the aircraft stationary on the bench during this period. */
        if (!dev->altitude_reference_valid) {
            dev->reference_pressure_accumulator += (double)dev->pressure_pa;
            if (dev->reference_sample_count < DPS310_BASELINE_SAMPLES) {
                dev->reference_sample_count++;
            }
            if (dev->reference_sample_count >= DPS310_BASELINE_SAMPLES) {
                dev->reference_pressure_pa = (float)(
                    dev->reference_pressure_accumulator /
                    (double)DPS310_BASELINE_SAMPLES);
                dev->altitude_reference_valid = true;
                dev->altitude_filter_initialized = false;
            }
        }

        update_relative_altitude(dev);
        break;
    }

    default:
        break;
    }

    now = HAL_GetTick();
    if ((dev->async_state == DPS310_ASYNC_IDLE) &&
        ((now - dev->last_poll_ms) >= POLL_INTERVAL_MS)) {
        dev->last_poll_ms = now;
        (void)start_async_read(dev, REG_MEAS_CFG,
                               &dev->async_status, 1u,
                               DPS310_ASYNC_STATUS_IN_FLIGHT);
    }

    return pressure_updated;
}

/* HAL callbacks only publish completion state. Compensation and altitude
   calculations remain in the main loop, never inside the I2C interrupt. */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    DPS310_HandleTypeDef *dev = active_dps310;

    if ((dev == NULL) || (hi2c != dev->hi2c)) {
        return;
    }

    __DMB(); /* Make the I2C RX buffer visible before publishing DONE. */
    switch (dev->async_state) {
    case DPS310_ASYNC_STATUS_IN_FLIGHT:
        dev->async_state = DPS310_ASYNC_STATUS_DONE;
        break;
    case DPS310_ASYNC_TEMP_IN_FLIGHT:
        dev->async_state = DPS310_ASYNC_TEMP_DONE;
        break;
    case DPS310_ASYNC_PRESS_IN_FLIGHT:
        dev->async_state = DPS310_ASYNC_PRESS_DONE;
        break;
    default:
        break;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    DPS310_HandleTypeDef *dev = active_dps310;

    if ((dev == NULL) || (hi2c != dev->hi2c)) {
        return;
    }

    dev->error_count++;
    dev->async_state = DPS310_ASYNC_IDLE;
    dev->last_poll_ms = HAL_GetTick();
}

bool DPS310_GetData(const DPS310_HandleTypeDef *dev, DPS310_Data *data)
{
    if ((dev == NULL) || (data == NULL)) {
        return false;
    }

    data->pressure_pa = dev->pressure_pa;
    data->temperature_c = dev->temperature_c;
    data->altitude_m = dev->altitude_m;
    data->altitude_filtered_m = dev->altitude_filtered_m;
    data->reference_pressure_pa = dev->reference_pressure_pa;
    data->error_count = dev->error_count;
    data->reference_sample_count = dev->reference_sample_count;
    data->initialized = dev->initialized;
    data->pressure_valid = dev->pressure_valid;
    data->temperature_valid = dev->temperature_valid;
    data->altitude_reference_valid = dev->altitude_reference_valid;
    data->altitude_valid = dev->altitude_valid;

    return dev->pressure_valid;
}
