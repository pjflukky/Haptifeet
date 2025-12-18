/**
 * @file task_eeprom.c
 * @author Ben Wolf
 * @brief Task for the EEPROM. Provides CLI and helper functions for interacting with the EEPROM.
 * @version 0.1
 * @date 2025-12-05
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "task_eeprom.h"
#include "main.h"
#include "eeprom.h"
#include "portable.h"
#include <stdint.h>

static BaseType_t cli_handler_eeprom_read(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString);

static BaseType_t cli_handler_eeprom_write(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString);

static cyhal_spi_t *eeprom_spi_obj = NULL;
static cyhal_gpio_t eeprom_cs_pin = NC;

QueueHandle_t Queue_EEPROM_Requests;
QueueHandle_t Queue_EEPROM_Responses;

static const CLI_Command_Definition_t xEEPROM_Read =
    {
        "eeprom_read",
        "\r\neeprom_read < address >\r\n",
        cli_handler_eeprom_read,
        1};

static const CLI_Command_Definition_t xEEPROM_Write =
    {
        "eeprom_write",
        "\r\neeprom_write < address > < data >\r\n",
        cli_handler_eeprom_write,
        2};

/**
 * @brief
 * This function will create the EEPROM task for handling read and write requests to the EEPROM.
 * It assumes that you have already created a semaphore for SPI access and initialized
 * the SPI peripheral.  This function does NOT initialize the SPI peripheral OR CS Pin
 * because the SPI peripheral is shared between multiple tasks (e.g. IMU, EEPROM, etc.).
 * @param spi_semaphore
 * @param spi_obj
 * @param cs_pin
 * @return true
 * @return false
 */
bool task_eeprom_resources_init(SemaphoreHandle_t *spi_semaphore, cyhal_spi_t *spi_obj, cyhal_gpio_t cs_pin)
{
    if (spi_semaphore == NULL || spi_obj == NULL)
    {
        return false;
    }

    eeprom_spi_obj = spi_obj;
    eeprom_cs_pin = cs_pin;

    /* Initialize the eeprom driver with the correct spi handle and cs pin */
    eeprom_init(eeprom_spi_obj, eeprom_cs_pin);

    /*Create the EEPROM Requests Queue */
    Queue_EEPROM_Requests = xQueueCreate(5, sizeof(eeprom_message_t));
    Queue_EEPROM_Responses = xQueueCreate(5, sizeof(eeprom_message_t));

    FreeRTOS_CLIRegisterCommand(&xEEPROM_Read);
    FreeRTOS_CLIRegisterCommand(&xEEPROM_Write);

    /* Create the FreeRTOS task for the EEPROM */
    if (xTaskCreate(
            task_eeprom,
            "EEPROM",
            5 * configMINIMAL_STACK_SIZE,
            spi_semaphore,
            tskIDLE_PRIORITY + 1,
            NULL) != pdPASS)
    {
        return false;
    }
    return true;
}

BaseType_t cli_handler_eeprom_read(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString)
{
    eeprom_message_t request;

    BaseType_t xParameterStringLength, xReturn;

    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);

    // Get the address parameter
    const char *pcParameter = FreeRTOS_CLIGetParameter(
        pcCommandString,
        1,
        &xParameterStringLength);
    configASSERT(pcParameter);

    request.command = EEPROM_CMD_READ_DATA;
    request.address = (uint16_t)strtol(pcParameter, NULL, 16);
    request.data = NULL;
    request.length = 1;
    request.response_queue = Queue_EEPROM_Responses;
    xQueueSend(
        Queue_EEPROM_Requests,
        &request,
        portMAX_DELAY);

    while (1)
    {
        if (xQueueReceive(
                Queue_EEPROM_Responses,
                &request,
                portMAX_DELAY) == pdTRUE)
        {
            if (request.command == EEPROM_CMD_READ_DATA)
            {
                sprintf(pcWriteBuffer, "\r\nThe value at 0x%s is 0x%02X\r\n", pcParameter, request.data[0]);
                vPortFree(request.data);
                break;
            }
        }
    }

    xReturn = pdFALSE;
    return xReturn;
}

BaseType_t cli_handler_eeprom_write(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString)
{
    char *pcParameter;
    eeprom_message_t request;
    request.data = pvPortMalloc(1);

    BaseType_t xParameterStringLength, xReturn;

    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);

    // Get the address parameter
    pcParameter = strdup(FreeRTOS_CLIGetParameter(
        pcCommandString,
        1,
        &xParameterStringLength));
    configASSERT(pcParameter);

    const char *address = strtok(pcParameter, " ");
    const char *data = strtok(NULL, " ");

    request.command = EEPROM_CMD_WRITE_DATA;
    request.address = (uint16_t)strtol(address, NULL, 16);
    request.data[0] = (uint8_t)strtol(data, NULL, 16);
    request.length = 1;
    request.response_queue = NULL;

    xQueueSend(
        Queue_EEPROM_Requests,
        &request,
        portMAX_DELAY);

    sprintf(pcWriteBuffer, "\r\nEEPROM: wrote 0x%s to address 0x%s\r\n", data, address);

    xReturn = pdFALSE;
    return xReturn;
}

void eeprom_write(uint8_t *data, uint16_t length)
{
    eeprom_message_t request;

    // 1. Allocate memory on the heap. 
    // The task expects to own this memory and will vPortFree it.
    request.data = pvPortMalloc(length);

    // 2. Check if allocation was successful to prevent null pointer dereference
    if (request.data != NULL)
    {
        request.command = EEPROM_CMD_WRITE_DATA;
        request.address = 0;
        request.length = length;
        request.response_queue = NULL;

        // 3. Copy the data from the source to the new heap buffer
        // We use a loop here to avoid dependency on <string.h> / memcpy
        for (uint16_t i = 0; i < length; i++)
        {
            request.data[i] = data[i];
        }

        xQueueSend(
            Queue_EEPROM_Requests,
            &request,
            portMAX_DELAY);
    }
}

// data needs to be freed after use
void eeprom_read(uint8_t *data, uint16_t length)
{
    eeprom_message_t request;

    request.command = EEPROM_CMD_READ_DATA;
    request.address = 0;
    request.data = NULL;
    request.length = length;
    request.response_queue = Queue_EEPROM_Responses;

    xQueueSend(
        Queue_EEPROM_Requests,
        &request,
        portMAX_DELAY);

    while (1)
    {
        if (xQueueReceive(
                Queue_EEPROM_Responses,
                &request,
                portMAX_DELAY) == pdTRUE)
        {
            if (request.command == EEPROM_CMD_READ_DATA)
            {
                for (uint16_t i = 0; i < length; i++)
                {
                    data[i] = request.data[i];
                }
                vPortFree(request.data);
                break;
            }
        }
    }
}

void task_eeprom(void *arg)
{
    SemaphoreHandle_t *spi_semaphore = NULL;
    eeprom_message_t request;

    if (arg != NULL)
    {
        spi_semaphore = (SemaphoreHandle_t *)arg;
    }
    else
    {
        CY_ASSERT(0);
    }

    while (1)
    {
        // Wait for a request to arrive
        xQueueReceive(
            Queue_EEPROM_Requests,
            &request,
            portMAX_DELAY);

        // Take the SPI semaphore
        xSemaphoreTake(*spi_semaphore, portMAX_DELAY);

        // Process the request
        switch (request.command)
        {
        case EEPROM_CMD_WRITE_DATA:
            for (uint8_t i = 0; i < request.length; i++)
            {
                eeprom_write_byte(request.address + i, request.data[i]);
            }
            // Need to return the data that was written to the FreeRTOS heap
            vPortFree(request.data);
            break;

        case EEPROM_CMD_READ_DATA:
            /* Return the data to the user.  */

            /* If the data pointer is NULL, allocate space to hold the data on the FreeRTOS heap */
            if (request.data == NULL)
            {
                request.data = pvPortMalloc(request.length);
                if (request.data == NULL)
                {
                    CY_ASSERT(0);
                }
            }

            for (uint8_t i = 0; i < request.length; i++)
            {
                request.data[i] = eeprom_read_byte(request.address + i);
            }
            break;

        default:
            // Invalid command
            break;
        }

        // Give the SPI semaphore
        xSemaphoreGive(*spi_semaphore);

        // If a response queue was provided, send the request back as a response
        if (request.command == EEPROM_CMD_READ_DATA && request.response_queue != NULL)
        {
            xQueueSend(
                request.response_queue,
                &request,
                portMAX_DELAY);
        }
    }
}
