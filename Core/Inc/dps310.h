#ifndef DPS310_H
#define DPS310_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DPS310_I2C_ADDR_DEFAULT       0x77u
#define DPS310_I2C_ADDR_ALT           0x76u

#define DPS310_PRESSURE_ODR_HZ        64u
#define DPS310_PRESSURE_OVERSAMPLING  8u
#define DPS310_TEMPERATURE_ODR_HZ     1u
#define DPS310_TEMPERATURE_OVERSAMPLING 1u

#define DPS310_BASELINE_SAMPLES       64u

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t address_7bit;

    int32_t c0;
    int32_t c1;
    int32_t c00;
    int32_t c10;
    int32_t c01;
    int32_t c11;
    int32_t c20;
    int32_t c21;
    int32_t c30;
    int32_t raw_temperature;
    uint8_t async_status;
    uint8_t raw_temperature_bytes[3];
    uint8_t raw_pressure_bytes[3];
    volatile uint8_t async_state;
    uint32_t last_poll_ms;

    float pressure_scale;
    float temperature_scale;
    float temperature_c;
    float pressure_pa;
    float reference_pressure_pa;
    float altitude_m;
    float altitude_filtered_m;

    double reference_pressure_accumulator;
    volatile uint32_t error_count;
    uint16_t reference_sample_count;

    bool initialized;
    bool temperature_valid;
    bool pressure_valid;
    bool altitude_reference_valid;
    bool altitude_valid;
    bool altitude_filter_initialized;
} DPS310_HandleTypeDef;

typedef struct
{
    float pressure_pa;
    float temperature_c;
    float altitude_m;
    float altitude_filtered_m;
    float reference_pressure_pa;
    uint32_t error_count;
    uint16_t reference_sample_count;
    bool initialized;
    bool pressure_valid;
    bool temperature_valid;
    bool altitude_reference_valid;
    bool altitude_valid;
} DPS310_Data;

bool DPS310_Init(DPS310_HandleTypeDef *dev,
                 I2C_HandleTypeDef *hi2c,
                 uint8_t address_7bit);

/* Poll the status register and consume fresh pressure/temperature results. */
bool DPS310_Update(DPS310_HandleTypeDef *dev);

/* Copy the latest values. Returns true when pressure data is valid. */
bool DPS310_GetData(const DPS310_HandleTypeDef *dev, DPS310_Data *data);

#ifdef __cplusplus
}
#endif

#endif /* DPS310_H */
