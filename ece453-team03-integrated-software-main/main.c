/**
 * @file main.c
 * @author Joe Krachey (jkrachey@wisc.edu)
 * @brief
 * @version 0.1
 * @date 2024-05-14
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "main.h"
#include "source/app_hw/i2c.h"
#include "source/app_hw/spi.h"
#include "source/app_hw/task_console.h"
#include "source/app_hw/task_eeprom.h"
#include "source/app_hw/task_haptics.h"
#include "source/app_hw/task_poten.h"
#include "source/app_hw/task_lidar.h"
#include "source/app_hw/task_core.h"

int main(void)
{
    cy_rslt_t rslt;

    __enable_irq();

    /* Initialize the device and board peripherals */
    rslt = cybsp_init();
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);

    rslt = i2c_init();
    // output error msg if initialization fails
    if (rslt != CY_RSLT_SUCCESS)
    {
        debug_printf("I2C Initialization Failed!\n");
    }
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);
    
    rslt = spi_init();
    if (rslt != CY_RSLT_SUCCESS)
    {
        debug_printf("SPI Initialization Failed!\n");
    }
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);

    task_haptics_init();
    task_eeprom_resources_init(&Semaphore_SPI, &mSPI, EEPROM_CS_N);

    task_console_init();

    task_core_init();

    task_poten_init();

    task_lidar_init();

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    // Unreachable
    for (;;)
    {
    }
}

/* [] END OF FILE */
