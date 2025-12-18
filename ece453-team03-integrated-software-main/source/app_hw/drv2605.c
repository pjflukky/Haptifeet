/**
 * @file drv2605.c
 * @author Isak Keyes
 * @brief Source file for the DRV2605 haptic driver
 * @date 2025-10-06
 */
#include "drv2605.h"
#include "task_console.h"
#include "i2c.h"

#define DRV2605_I2C_ADDRESS 0x5A // 7-bit I2C address for DRV2605
#define I2C_TIMEOUT_MS 10
#define I2C_SEMAPHORE_WAIT portMAX_DELAY
#define I2C_NUM_RETRIES 10 // Number of times to retry I2C operations on failure
#define I2C_RETRY_DELAY_MS 0 // Delay between I2C retries

#define RTP_DATA_FORMAT 1 // 0: signed int, 1: unsigned int

/******************************************************************************/
/* Enumeration Definitions                                                    */
/******************************************************************************/

/**
 * @brief Enumeration of DRV2605 Register Addresses
 * 
 * @note Not all registers are included here, only those relevant to this project
 */
typedef enum
{
    DRV2605_STATUS_REG = 0x00, // Status Register
    DRV2605_MODE_REG = 0x01, // Mode Register
    DRV2605_RTP_INPUT_REG = 0x02, // Real-Time Playback Input Register
    DRV2605_LIB_SEL_REG = 0x03, // Library Selection Register
    //...
    DRV2605_GO_REG = 0x0C, // Go Register
    //...
    DRV2605_RATED_VOLT_REG = 0x16, // Rated Voltage Register
    DRV2605_OD_CLAMP_REG = 0x17, // Overdrive Clamp Voltage Register
    DRV2605_AUTOCAL_COMP_RSLT_REG = 0x18, // Auto-Calibration Compensation Result Register
    DRV2605_AUTOCAL_BEMF_RSLT_REG = 0x19, // Auto-Calibration Back-EMF Result Register
    //...
    DRV2605_FB_CTRL_REG = 0x1A, // Feedback Control Register
    DRV2605_CTRL1_REG = 0x1B, // Control1 Register
    DRV2605_CTRL2_REG = 0x1C, // Control2 Register
    DRV2605_CTRL3_REG = 0x1D, // Control3 Register
    DRV2605_CTRL4_REG = 0x1E, // Control4 Register
    //...
    DRV2605_LRA_RES_PRD_REG = 0x22 // LRA Resonance Period Register
} drv2605_reg_addr_t;

/* DRV2605 Modes */
typedef enum
{
    DRV2605_MODE_INTTRIG = 0x00,
    //...
    DRV2605_MODE_RTP = 0x05,
    DRV2605_MODE_DIAGNOS = 0x06,
    DRV2605_MODE_AUTOCAL = 0x07
} drv2605_mode_t;

/******************************************************************************/
/* DRV2605 Register Bitfields                                                 */
/******************************************************************************/

/* DRV2605 Status Register (0x00) Bitfield - Default = 0xE0 */
typedef union
{
    struct {
        uint8_t oc_detect : 1;
        uint8_t over_temp : 1;
        uint8_t _reserved_1 : 1;
        uint8_t diag_result : 1;
        uint8_t _reserved_2 : 1;
        uint8_t device_id : 3;
    } fields;
    uint8_t raw;
} drv2605_status_reg_t;

/* DRV2605 Mode Register (0x01) Bitfield - Default = 0x40 */
typedef union
{
    struct {
        uint8_t mode : 3;
        uint8_t _reserved_1 : 3;
        uint8_t standby : 1;
        uint8_t dev_reset : 1;
    } fields;
    uint8_t raw;
} drv2605_mode_reg_t;

/* DRV2605 Feedback Control Register (0x1A) Bitfield - Default = 0x36 */
typedef union
{
    struct {
        uint8_t bemf_gain : 2;
        uint8_t loop_gain : 2;
        uint8_t fb_brake_factor : 3;
        uint8_t n_erm_lra : 1;
    } fields;
    uint8_t raw;
} drv2605_fb_ctrl_reg_t;

/* DRV2605 Control Register 1 (0x1B) Bitfield - Default = 0x93 */
typedef union
{
    struct {
        uint8_t drive_time : 5;
        uint8_t ac_couple : 1;
        uint8_t _reserved_1 : 1;
        uint8_t startup_boost : 1;
    } fields;
    uint8_t raw;
} drv2605_ctrl1_reg_t;

/* DRV2605 Control Register 2 (0x1C) Bitfield */
typedef union
{
    struct {
        uint8_t idiss_time : 2;
        uint8_t blanking_time : 2;
        uint8_t sample_time : 2;
        uint8_t brake_stabilizer : 1;
        uint8_t bidir_input : 1;
    } fields;
    uint8_t raw;
} drv2605_ctrl2_reg_t;

/* DRV2605 Control Register 3 (0x1D) Bitfield - Default = 0xA0 */
typedef union
{
    struct {
        uint8_t lra_open_loop : 1;
        uint8_t n_pwm_analog : 1;
        uint8_t lra_drive_mode : 1;
        uint8_t data_format_rtp : 1;
        uint8_t supply_comp_dis : 1;
        uint8_t erm_open_loop : 1;
        uint8_t ng_thresh : 2;
    } fields;
    uint8_t raw;
} drv2605_ctrl3_reg_t;

/* DRV2605 Control Register 4 (0x1E) Bitfield */
typedef union
{
    struct {
        uint8_t otp_program : 1;
        uint8_t _reserved_1 : 1;
        uint8_t otp_status : 1;
        uint8_t _reserved_2 : 1;
        uint8_t auto_cal_time : 2;
        uint8_t zc_det_time : 2;
    } fields;
    uint8_t raw;
} drv2605_ctrl4_reg_t;

/******************************************************************************/
/* Static Function Definitions                                                */
/******************************************************************************/

/**
 * @brief Read a value from a DRV2605 register
 * 
 * @param reg Register address (see drv2605_reg_addr_t)
 * @param value Pointer to store the read value
 * @return cy_rslt_t Result of the operation
 */
static cy_rslt_t drv2605_reg_read(uint8_t reg, uint8_t *value)
{
    cy_rslt_t rslt;
    int i2c_attempts = 0;

    /* There may be multiple I2C devices on the same bus, so we must take
    the semaphore used to ensure mutual exclusion before we begin any
    I2C transactions */
    xSemaphoreTake(Semaphore_I2C, I2C_SEMAPHORE_WAIT);

    // Write register address to DRV2605
    while (1) {
        rslt = cyhal_i2c_master_write(
            &i2c_master_obj,
            DRV2605_I2C_ADDRESS,
            &reg,
            1,
            I2C_TIMEOUT_MS,
            false);
        if (rslt != CY_RSLT_SUCCESS && i2c_attempts++ < I2C_NUM_RETRIES)
        {
            vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_DELAY_MS));
        } else {
            break;
        }
    }

    // Read register value from DRV2605
    i2c_attempts = 0;
    if (rslt == CY_RSLT_SUCCESS)
    {
        while (1) {
            rslt = cyhal_i2c_master_read(
                &i2c_master_obj,
                DRV2605_I2C_ADDRESS,
                value,
                1,
                I2C_TIMEOUT_MS,
                true);
            if (rslt != CY_RSLT_SUCCESS && i2c_attempts++ < I2C_NUM_RETRIES)
            {
                vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_DELAY_MS));
            } else {
                break;
            }
        }
    }

    xSemaphoreGive(Semaphore_I2C);

    return rslt;
}

/**
 * @brief Write a value to a DRV2605 register
 * 
 * @param reg Register address
 * @param value Value to write
 * @return cy_rslt_t Result of the operation
 */
static cy_rslt_t drv2605_reg_write(uint8_t reg, uint8_t value)
{
    cy_rslt_t rslt;
    int i2c_attempts = 0;
    uint8_t write_buffer[2] = {reg, value};

    /* There may be multiple I2C devices on the same bus, so we must take
    the semaphore used to ensure mutual exclusion before we begin any
    I2C transactions */
    xSemaphoreTake(Semaphore_I2C, I2C_SEMAPHORE_WAIT);

    // Write register address and value to DRV2605
    while (1) {
        rslt = cyhal_i2c_master_write(
            &i2c_master_obj, 
            DRV2605_I2C_ADDRESS,
            write_buffer,
            sizeof(write_buffer),
            I2C_TIMEOUT_MS,
            true);
        if (rslt != CY_RSLT_SUCCESS && i2c_attempts++ < I2C_NUM_RETRIES)
        {
            vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_DELAY_MS));
        } else {
            break;
        }
    }

    xSemaphoreGive(Semaphore_I2C);

    return rslt;
}

/**
 * @brief Read-modify-write a value to a DRV2605 register
 * 
 * @param reg Register address (see drv2605_reg_addr_t)
 * @param mask Bitmask of bits to modify (1 = modify, 0 = preserve)
 * @param value New value of register, preserved bits will be ignored
 * @return cy_rslt_t Result of the operation
 */
static cy_rslt_t drv2605_reg_rmw(uint8_t reg, uint8_t mask, uint8_t value)
{
    cy_rslt_t rslt;
    uint8_t current_value;
    uint8_t new_value;

    // Read current register value
    rslt = drv2605_reg_read(reg, &current_value);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    // Modify only the bits specified by the mask
    new_value = (current_value & ~mask) | (value & mask);

    // Write the modified value back to the register
    return drv2605_reg_write(reg, new_value);
}

/**
 * @brief Write the auto-calibration input parameters to their
 * respective registers on the DRV2605.
 * 
 * @return cy_rslt_t Result of the operation
 */
static cy_rslt_t drv2605_write_autocal_params(void)
{
    cy_rslt_t rslt;

    // Set Feedback Control Register
    drv2605_fb_ctrl_reg_t fb_ctrl_mask = { .raw = ~0 };
    fb_ctrl_mask.fields.bemf_gain = 0; // Mask to modify everything EXCEPT bemf_gain
    drv2605_fb_ctrl_reg_t fb_ctrl = {
        .fields.n_erm_lra = N_ERM_LRA,
        .fields.fb_brake_factor = FB_BRAKE_FACTOR,
        .fields.loop_gain = LOOP_GAIN,
    };
    rslt = drv2605_reg_rmw(DRV2605_FB_CTRL_REG, fb_ctrl_mask.raw, fb_ctrl.raw);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    // Set Control1 Register
    drv2605_ctrl1_reg_t ctrl1_mask = {.raw = 0};
    ctrl1_mask.fields.drive_time = ~0; // Mask to modify ONLY drive_time
    drv2605_ctrl1_reg_t ctrl1 = {
        .fields.drive_time = DRIVE_TIME,
    };
    rslt = drv2605_reg_rmw(DRV2605_CTRL1_REG, ctrl1_mask.raw, ctrl1.raw);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    // // Set Control2 Register TODO: remove? (only used to set brake_stabilizer to 0)
    // drv2605_ctrl2_reg_t ctrl2_mask = {.raw = 0};
    // ctrl2_mask.fields.brake_stabilizer = ~0; // Mask to modify ONLY brake_stabilizer
    // drv2605_ctrl2_reg_t ctrl2 = {.raw = ~0};
    // rslt = drv2605_reg_rmw(DRV2605_CTRL2_REG, ctrl2_mask.raw, ctrl2.raw);
    // if (rslt != CY_RSLT_SUCCESS)
    // {
    //     return rslt;
    // }

    // Set Control3 Register
    drv2605_ctrl3_reg_t ctrl3_mask = {.raw = 0};
    ctrl3_mask.fields.erm_open_loop = ~0; // Mask to modify ONLY erm_open_loop
    drv2605_ctrl3_reg_t ctrl3 = {.raw = 0};
    rslt = drv2605_reg_rmw(DRV2605_CTRL3_REG, ctrl3_mask.raw, ctrl3.raw);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    // // Set Control4 Register TODO: remove? (only used to set auto_cal_time)
    // drv2605_ctrl4_reg_t ctrl4_mask = {.raw = 0};
    // ctrl4_mask.fields.auto_cal_time = ~0; // Mask to modify ONLY auto_cal_time
    // drv2605_ctrl4_reg_t ctrl4 = {
    //     .fields.auto_cal_time = 0x3, // 1000ms (minimum) - 1200ms (maximum)
    // };
    // rslt = drv2605_reg_rmw(DRV2605_CTRL4_REG, ctrl4_mask.raw, ctrl4.raw);
    // if (rslt != CY_RSLT_SUCCESS)
    // {
    //     return rslt;
    // }

    // Set Rated Voltage Register
    rslt = drv2605_reg_write(DRV2605_RATED_VOLT_REG, RATED_VOLTAGE);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    // Set Overdrive Clamp Voltage Register
    rslt = drv2605_reg_write(DRV2605_OD_CLAMP_REG, OD_CLAMP);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    return CY_RSLT_SUCCESS;
}

/**
 * @brief Set the DRV2605 operating mode and take it out of standby
 * 
 * @param mode Operating mode to set
 * @return cy_rslt_t Result of the operation
 */
static cy_rslt_t drv2605_set_mode(uint8_t mode)
{
    drv2605_mode_reg_t mode_reg_mask = { .raw = 0 };
    mode_reg_mask.fields.mode = ~0; // Mask to modify ONLY mode bits
    mode_reg_mask.fields.standby = ~0; // Mask to modify ONLY standby bit
    drv2605_mode_reg_t mode_reg = {
        .fields.mode = mode,
        .fields.standby = 0 // Take device out of standby
    };
    return drv2605_reg_rmw(DRV2605_MODE_REG, mode_reg_mask.raw, mode_reg.raw);
}

/******************************************************************************/
/* Public Function Definitions                                                */
/******************************************************************************/

/**
 * @brief Read the calibration values (auto-calibration results) from the DRV2605
 * 
 * @param calibration Pointer to drv2605_calibration_t struct to store results
 * @return cy_rslt_t Result of the read operation
 */
cy_rslt_t drv2605_read_calibration(drv2605_calibration_t *calibration)
{
    cy_rslt_t rslt;

    // Read Feedback Control Register
    drv2605_fb_ctrl_reg_t fb_ctrl;
    rslt = drv2605_reg_read(DRV2605_FB_CTRL_REG, &fb_ctrl.raw);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }
    calibration->bemf_gain = fb_ctrl.fields.bemf_gain;

    // Read Auto-Calibration Compensation Result Register
    rslt = drv2605_reg_read(DRV2605_AUTOCAL_COMP_RSLT_REG, &calibration->autocal_comp);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    // Read Auto-Calibration Back-EMF Result Register
    rslt = drv2605_reg_read(DRV2605_AUTOCAL_BEMF_RSLT_REG, &calibration->autocal_bemf);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    return CY_RSLT_SUCCESS;
}

/**
 * @brief Write calibration values to the DRV2605
 * 
 * @param calibration Pointer to drv2605_calibration_t struct containing calibration values
 * @return cy_rslt_t Result of the operation
 */
cy_rslt_t drv2605_write_calibration(const drv2605_calibration_t *calibration)
{
    cy_rslt_t rslt;

    // Write Feedback Control Register
    drv2605_fb_ctrl_reg_t fb_ctrl_mask = { .raw = 0 };
    fb_ctrl_mask.fields.bemf_gain = ~0; // Mask to modify ONLY bemf_gain
    drv2605_fb_ctrl_reg_t fb_ctrl = {
        .fields.bemf_gain = calibration->bemf_gain
    };
    rslt = drv2605_reg_rmw(DRV2605_FB_CTRL_REG, fb_ctrl_mask.raw, fb_ctrl.raw);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    // Write Auto-Calibration Compensation Result Register
    rslt = drv2605_reg_write(DRV2605_AUTOCAL_COMP_RSLT_REG, calibration->autocal_comp);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    // Write Auto-Calibration Back-EMF Result Register
    rslt = drv2605_reg_write(DRV2605_AUTOCAL_BEMF_RSLT_REG, calibration->autocal_bemf);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    return CY_RSLT_SUCCESS;
}

/**
 * @brief Run auto-calibration on the DRV2605
 * 
 * This can be run on startup every time the device is powered on,
 * stored in our own NVM to be reapplied on each startup,
 * or stored permanently in the on-board OTP memory.
 * 
 * @note Full calibration guide in section 8.5.6 of the DRV2605 datasheet,
 * and in the DRV2605 setup guide.
 * 
 * @return cy_rslt_t Result of the operation
 */
cy_rslt_t drv2605_run_autocal(void)
{
    cy_rslt_t rslt;

    // Set to auto-calibration mode
    rslt = drv2605_set_mode(DRV2605_MODE_AUTOCAL);
    // rslt = drv2605_set_mode(DRV2605_MODE_DIAGNOS); //TODO: remove / set back to AUTOCAL
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }
    
    // Write auto-calibration input parameters
    rslt = drv2605_write_autocal_params();
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }
    
    // Start auto-calibration by writing to GO register
    rslt = drv2605_reg_write(DRV2605_GO_REG, 0x01);
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("DRV2605: Failed to write GO_REG (%lu)", rslt);
        return rslt;
    }
    
    // Wait for calibration to complete (GO bit cleared)
    uint8_t go_reg = 0x01;
    uint16_t attempts = 0;
    const uint16_t max_attempts = 1000; // give up after specified # of attempts
    while (attempts < max_attempts)
    {
        rslt = drv2605_reg_read(DRV2605_GO_REG, &go_reg);
        if (!(go_reg & 0x01))
        {
            break; // GO bit cleared; calibration complete
        }
        attempts++;
        vTaskDelay(pdMS_TO_TICKS(1)); // Delay between attempts
    }
    if (attempts == max_attempts)
    {
        // max attempts reached, calibration did not complete
        task_print_error("DRV2605: Auto-calibration timed out");
        return rslt;
    }

    // Check status of the DIAG_RESULT bit in status register
    drv2605_status_reg_t status_reg;
    rslt = drv2605_reg_read(DRV2605_STATUS_REG, &status_reg.raw);
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("DRV2605: Failed to read STATUS_REG (%lu)", rslt);
        return rslt;
    }
    if (status_reg.fields.diag_result != 0)
    {
        // Auto-calibration failed
        task_print_error("DRV2605: Auto-calibration failed (DIAG_RESULT asserted)");
        return CY_RSLT_TYPE_ERROR;
    }
    return CY_RSLT_SUCCESS;
}

/**
 * @brief Resets the DRV2605 device 
 * 
 * @return cy_rslt_t Result of the operation
 */
cy_rslt_t drv2605_dev_reset(void)
{
    // Set the DEV_RESET bit in the MODE register
    drv2605_mode_reg_t mode_reg_mask = { .raw = 0 };
    mode_reg_mask.fields.dev_reset = ~0; // Mask to modify ONLY dev_reset bit
    drv2605_mode_reg_t mode_reg = {
        .fields.dev_reset = 1
    };
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay 10 ms to allow reset to complete
    return drv2605_reg_rmw(DRV2605_MODE_REG, mode_reg_mask.raw, mode_reg.raw);
}

/**
 * @brief Initializes the DRV2605 haptic driver to RTP mode
 * 
 * Sets the DRV2605 to RTP mode for real-time playback and
 * sets the data format to unsigned integer.
 * 
 * @return cy_rslt_t Result of the operation
 */
cy_rslt_t drv2605_rtp_init(void)
{
    // Set RTP data format to unsigned integer
    drv2605_ctrl3_reg_t ctrl3_mask = {.raw = 0};
    ctrl3_mask.fields.data_format_rtp = ~0; // Mask to modify ONLY data_format_rtp
    drv2605_ctrl3_reg_t ctrl3 = {
        .fields.data_format_rtp = RTP_DATA_FORMAT,
    };
    cy_rslt_t rslt = drv2605_reg_rmw(DRV2605_CTRL3_REG, ctrl3_mask.raw, ctrl3.raw);
    if (rslt != CY_RSLT_SUCCESS)
    {
        return rslt;
    }

    // Set DRV2605 to RTP mode
    return drv2605_set_mode(DRV2605_MODE_RTP);
}

/**
 * @brief Sets the haptic amplitude for real-time playback
 * 
 * @param amplitude Amplitude value (unsigned 8-bit integer; 0-255)
 * @return cy_rslt_t Result of the operation
 */
cy_rslt_t drv2605_set_amplitude(uint8_t amplitude)
{
    return drv2605_reg_write(DRV2605_RTP_INPUT_REG, amplitude);
}
