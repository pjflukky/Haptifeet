/**
 * @file task_haptics.c
 * @author Isak Keyes
 * @brief Task and CLI handler for controlling the haptics.
 * @date 2025-10-06
 */
#include "task_haptics.h"
#include "drv2605.h"
#include "i2c.h"
#include "task_console.h"
#include "task_eeprom.h"

#define TASK_HAPTICS_STACK_SIZE (configMINIMAL_STACK_SIZE * 20) // TODO how to decide stack size? (INCLUDE_uxTaskGetStackHighWaterMark?)
#define TASK_HAPTICS_PRIORITY (configMAX_PRIORITIES - 5)        // TODO how to decide priority?

/**
 * @brief struct to hold autocalibration save data
 */
typedef struct
{
    drv2605_config_t config;
    uint8_t num_haptics;
    struct __attribute__((packed))
    {
        drv2605_calibration_t calibration;
        uint8_t valid;
    } cals[NUM_HAPTICS];
} haptics_save_data_t;

/**
 * @brief struct for reading/writing haptic data to/from EEPROM
 */
typedef union
{
    haptics_save_data_t save_data;
    uint8_t raw[sizeof(haptics_save_data_t)];
} haptics_eeprom_data_t;

/******************************************************************************/
/* Function Declarations                                                      */
/******************************************************************************/

static void task_haptics(void *param);

static BaseType_t cli_handler_haptics(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString);

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/

/* setup state for each haptic  (0: uninitialized, 1: initialized)*/
static uint8_t haptic_setup_state[NUM_HAPTICS] = {0};

/* amplitude for each haptic (invalid until respective setup state asserted) */
static uint8_t haptic_amplitude[NUM_HAPTICS] = {0};

/* array of queues to hold individual requests for each haptic */
static QueueHandle_t q_haptics_req[NUM_HAPTICS];

/* Task handle for the haptics task */
static TaskHandle_t task_haptics_handle = NULL;

/* The CLI command definition for the haptics command */
static const CLI_Command_Definition_t cmd_haptics =
    {
        "haptics",                                         /* command text */
        "\r\nhaptics <haptic_id> <amplitude (0-255)>\r\n", /* command help text */
        cli_handler_haptics,                               /* The function to run. */
        2                                                  /* The user must enter 2 parameters */
};

/******************************************************************************/
/* Static Function Definitions                                                */
/******************************************************************************/

/**
 * @brief Loads calibration data for a specific haptic from EEPROM
 *
 * @param haptic_id - ID of the haptic (0 to NUM_HAPTICS-1)
 * @param calibration - Pointer to store the loaded calibration data
 * @return 1 if successful, 0 if failed
 */
static uint8_t load_calibration(uint8_t haptic_id, drv2605_calibration_t *const calibration)
{
    static haptics_eeprom_data_t eeprom_data = {0};
    static uint8_t save_config_valid = 1; // flag to indicate if config differs from EEPROM

    // Exit if config data has changed since last save
    if (save_config_valid == 0)
    {
        return 0; // Config has changed, all calibration data invalid
    }

    // Read from EEPROM if not already done
    if (eeprom_data.save_data.num_haptics == 0)
    {
        eeprom_read(eeprom_data.raw, sizeof(haptics_save_data_t));

        // Check if config data matches
        if (eeprom_data.save_data.config.n_erm_lra != DRV_CONFIG.n_erm_lra ||
            eeprom_data.save_data.config.rated_voltage != DRV_CONFIG.rated_voltage ||
            eeprom_data.save_data.config.od_clamp != DRV_CONFIG.od_clamp)
        {
            save_config_valid = 0; // Config has changed, all calibration data invalid
            return 0;
        }
    }

    // Check if calibration data is valid
    if (eeprom_data.save_data.cals[haptic_id].valid != 1)
    {
        return 0; // Invalid calibration data
    }

    // Copy calibration data
    *calibration = eeprom_data.save_data.cals[haptic_id].calibration;
    return 1; // Success
}

/**
 * @brief Saves calibration data for a specific haptic to EEPROM
 *
 * @param haptic_id - ID of the haptic (0 to NUM_HAPTICS-1)
 * @param calibration - Pointer to the calibration data to save
 */
static void save_calibration(uint8_t haptic_id, const drv2605_calibration_t *const calibration)
{
    static haptics_eeprom_data_t eeprom_data = {0};

    // Update config data
    eeprom_data.save_data.config = DRV_CONFIG;
    eeprom_data.save_data.num_haptics = NUM_HAPTICS;

    // Update calibration data for specified haptic
    eeprom_data.save_data.cals[haptic_id].calibration = *calibration;
    eeprom_data.save_data.cals[haptic_id].valid = 1;

    // Write updated data back to EEPROM
    eeprom_write(eeprom_data.raw, sizeof(haptics_save_data_t));
}

/**
 * @brief Sets the MUX select GPIO pins based on the haptic ID
 *
 * @param haptic_id The ID of the haptic to select
 */
static void set_mux_select(uint8_t haptic_id)
{
    cyhal_gpio_write(HAPTIC_SEL_A, (haptic_id >> 0) & 0x01);
    cyhal_gpio_write(HAPTIC_SEL_B, (haptic_id >> 1) & 0x01);
    cyhal_gpio_write(HAPTIC_SEL_C, (haptic_id >> 2) & 0x01);
}

/**
 * @brief Gets the setup state of the specified haptic
 *
 * @param haptic_id The ID number of the haptic to get state of
 * @return uint8_t The setup state (0: uninitialized, 1: initialized)
 */
static uint8_t get_setup_state(uint8_t haptic_id)
{
    return haptic_setup_state[haptic_id];
}

/**
 * @brief Sets the setup state of the specified haptic
 *
 * @param haptic_id The ID number of the haptic to set state of
 * @param state The setup state to set (0: uninitialized, 1: initialized)
 */
static void set_setup_state(uint8_t haptic_id, uint8_t state)
{
    haptic_setup_state[haptic_id] = state;
}

/**
 * @brief Sets the haptic amplitude for the specified haptic ID
 *
 * @param haptic_id The ID of the haptic to set amplitude
 * @param amplitude The amplitude to set (0-255)
 * @return cy_rslt_t Result of the operation
 */
static cy_rslt_t process_haptic_update(uint8_t haptic_id, uint8_t amplitude)
{
    cy_rslt_t rslt;

    set_mux_select(haptic_id);

    // Check if haptic is initialized
    if (get_setup_state(haptic_id) == 0)
    {
        // Reset the device (equivalent to power cycle)
        rslt = drv2605_dev_reset();
        if (rslt != CY_RSLT_SUCCESS)
        {
            task_print_error("DRV2605: Failed to reset device (ID %lu)", rslt);
            return rslt;
        }

        // Calibrate the haptic
        drv2605_calibration_t calibration;
        // check for saved calibration
        if (load_calibration(haptic_id, &calibration))
        {
            // Write loaded calibration to device
            rslt = drv2605_write_calibration(&calibration);
            if (rslt != CY_RSLT_SUCCESS)
            {
                task_print_error("DRV2605: Failed to write prior calibration values (ID %lu)", rslt);
                return rslt;
            }
        }
        else
        {
            task_print_info("No valid calibration found in EEPROM, running autocalibration");
            rslt = drv2605_run_autocal();
            if (rslt != CY_RSLT_SUCCESS)
            {
                task_print_error("DRV2605: Failed to run autocalibration (ID %lu)", rslt);
                return rslt;
            }
            else
            {
                // Read back calibration results
                drv2605_calibration_t calibration;
                rslt = drv2605_read_calibration(&calibration);
                if (rslt == CY_RSLT_SUCCESS)
                {
                    // Save calibration to EEPROM
                    save_calibration(haptic_id, &calibration);
                }
                else
                {
                    task_print_error("DRV2605: Failed to read calibration results (ID %lu)", rslt);
                }
            }
        }

        // Initialize the haptic to RTP mode
        rslt = drv2605_rtp_init();
        if (rslt != CY_RSLT_SUCCESS)
        {
            task_print_error("DRV2605: Failed to initialize RTP mode (ID %lu)", rslt);
            return rslt;
        }
        set_setup_state(haptic_id, 1);
    }
    else if (haptic_amplitude[haptic_id] == amplitude)
    {
        // Amplitude is the same, no need to update
        return CY_RSLT_SUCCESS;
    }

    // Set the amplitude
    rslt = drv2605_set_amplitude(amplitude);
    if (rslt != CY_RSLT_SUCCESS)
    {
        task_print_error("DRV2605: Failed to set amplitude (%lu)", rslt);
        return rslt;
    }
    // Update the local amplitude
    haptic_amplitude[haptic_id] = amplitude;

    return rslt; // == CY_RSLT_SUCCESS
}

/**
 * @brief Requests a haptic amplitude update with queue overwrite behavior
 *
 * @param haptic_id The ID of the haptic (0 to NUM_HAPTICS-1)
 * @param amplitude The amplitude to set (0-255)
 */
void req_haptic_update(uint8_t haptic_id, uint8_t amplitude)
{
    if (haptic_id >= NUM_HAPTICS)
    {
        return; // Invalid haptic ID
    }

    // Try to send to the haptic's queue (non-blocking)
    if (xQueueSend(q_haptics_req[haptic_id], &amplitude, 0) != pdTRUE)
    {
        // Queue is full (has 1 old request), so overwrite it
        uint8_t old_amplitude;
        xQueueReceive(q_haptics_req[haptic_id], &old_amplitude, 0); // Remove old
        xQueueSend(q_haptics_req[haptic_id], &amplitude, 0);        // Add new
    }

    // Notify the haptics task that there's work to do
    if (task_haptics_handle != NULL)
    {
        xTaskNotifyGive(task_haptics_handle);
    }
}

/**
 * @brief Updates all haptics to the same amplitude
 *
 * This function sends amplitude requests to all haptic queues then notifies
 * the haptics task to process all updates.
 *
 * @param amplitude The amplitude to set for all haptics (0-255)
 */
void req_haptic_update_all(uint8_t amplitude)
{
    bool work_queued = false;

    // Update all haptic queues
    for (uint8_t i = 0; i < NUM_HAPTICS; i++)
    {
        // Try to send to the haptic's queue (non-blocking)
        if (xQueueSend(q_haptics_req[i], &amplitude, 0) != pdTRUE)
        {
            // Queue is full, so overwrite by removing old and adding new
            uint8_t old_amplitude;
            xQueueReceive(q_haptics_req[i], &old_amplitude, 0); // Remove old
            xQueueSend(q_haptics_req[i], &amplitude, 0);        // Add new
        }
        work_queued = true;
    }

    // Notify the haptics task only once if any work was queued
    if (work_queued && task_haptics_handle != NULL)
    {
        xTaskNotifyGive(task_haptics_handle);
    }
}

/**
 * @brief CLI handler for haptics command
 *
 * @param pcWriteBuffer Buffer to write response to
 * @param xWriteBufferLen Length of the write buffer
 * @param pcCommandString The command string from user
 * @return BaseType_t pdFALSE (command complete)
 */
static BaseType_t cli_handler_haptics(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString)
{
    const char *pcParameter;
    BaseType_t xParameterStringLength;
    uint8_t haptic_id, amplitude;

    // Get haptic_id parameter
    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing haptic_id parameter\r\n");
        return pdFALSE;
    }
    haptic_id = (uint8_t)atoi(pcParameter);

    // Get amplitude parameter
    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);
    if (pcParameter == NULL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing amplitude parameter\r\n");
        return pdFALSE;
    }
    amplitude = (uint8_t)atoi(pcParameter);

    // Validate parameters
    if (haptic_id >= NUM_HAPTICS)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: haptic_id must be 0-%d\r\n", NUM_HAPTICS - 1);
        return pdFALSE;
    }

    // Set the haptic amplitude
    req_haptic_update(haptic_id, amplitude);
    snprintf(pcWriteBuffer, xWriteBufferLen, "Request sent: haptic %d to amplitude %d\r\n", haptic_id, amplitude);

    return pdFALSE;
}

/******************************************************************************/
/* Public Function Definitions                                                */
/******************************************************************************/

/**
 * @brief Task used to monitor the reception of haptic update requests
 */
static void task_haptics(void *param)
{
    uint8_t amplitude;

    while (1)
    {
        // Wait for notification that an update request has been made
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Loop through each haptic queue to check for pending requests
        for (uint8_t i = 0; i < NUM_HAPTICS; i++)
        {
            // Check if this haptic has a pending request (non-blocking)
            if (xQueueReceive(q_haptics_req[i], &amplitude, 0) == pdTRUE)
            {
                // Process the haptic update
                process_haptic_update(i, amplitude);
            }
        }
    }
}

/**
 * @brief
 * Initializes the haptics task, queue, and CLI command.
 *
 * This function assumes the I2C bus is already initialized.
 *
 */
void task_haptics_init(void)
{
    // Ensure at least one haptic is defined, otherwise code will break
    configASSERT(NUM_HAPTICS >= 1);

    // Initialize the pins that control the MUX select lines
    cy_rslt_t rslt;
    rslt = cyhal_gpio_configure(HAPTIC_SEL_A, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG);
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);
    rslt = cyhal_gpio_configure(HAPTIC_SEL_B, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG);
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);
    rslt = cyhal_gpio_configure(HAPTIC_SEL_C, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG);
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);

    // Create the queues for haptic requests
    for (uint8_t i = 0; i < NUM_HAPTICS; i++)
    {
        q_haptics_req[i] = xQueueCreate(1, sizeof(uint8_t));
        configASSERT(q_haptics_req[i]);
    }

    // TODO calibrate all haptics here?

    // Register the CLI command for haptics
    FreeRTOS_CLIRegisterCommand(&cmd_haptics);

    // Create the task for handling haptic requests
    xTaskCreate(
        task_haptics,
        "Haptics",
        TASK_HAPTICS_STACK_SIZE,
        NULL,
        TASK_HAPTICS_PRIORITY,
        &task_haptics_handle);

    configASSERT(task_haptics_handle != NULL);
}
