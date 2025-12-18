/**
 * @file drv2605.h
 * @author Isak Keyes
 * @brief Header file for the DRV2605 haptic driver
 *
 * This file contains the function prototypes and definitions for the DRV2605
 * haptic driver.
 *
 * The haptic actuator information (LRA resonant frequency, rated voltage, etc.)
 * are fixed and defined in drv2605.c
 *
 * @date 2025-10-06
 */

#ifndef __DRV2605_H__
#define __DRV2605_H__

#include "main.h"
#include "drv2605_config.h"

/**
 * @brief Struct to hold the DRV2605 calibration values
 *
 * These are the values output by the auto-calibration process, which
 * can be written to the device registers to circumvent running
 * auto-calibration on every power-up.
 */
typedef struct
{
    uint8_t bemf_gain;    // Bits[1:0] of Feedback Control Register
    uint8_t autocal_comp; // Auto-Calibration Compensation Result
    uint8_t autocal_bemf; // Auto-Calibration Back-EMF Result
} drv2605_calibration_t;

/**
 * @brief Struct to hold the DRV2605 configuration parameters
 *
 * These parameters are specific to the actuator being used
 * and are defined in drv2605_config.h
 */
typedef struct
{
    uint8_t n_erm_lra;
    uint8_t rated_voltage;
    uint8_t od_clamp;
} drv2605_config_t;

// Global instance of the DRV2605 configuration
static const drv2605_config_t DRV_CONFIG = {.n_erm_lra = N_ERM_LRA,
                                            .rated_voltage = RATED_VOLTAGE,
                                            .od_clamp = OD_CLAMP};

cy_rslt_t drv2605_read_calibration(drv2605_calibration_t *calibration);
cy_rslt_t drv2605_write_calibration(const drv2605_calibration_t *calibration);
cy_rslt_t drv2605_run_autocal(void);

cy_rslt_t drv2605_dev_reset(void);
cy_rslt_t drv2605_rtp_init(void);
cy_rslt_t drv2605_set_amplitude(uint8_t amplitude);

#endif /* __DRV2605_H__ */
