/**
 * @file task_eeprom.h
 * @author Ben Wolf
 * @brief Header file for the eeprom task
 * @version 0.1
 * @date 2025-12-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __TASK_EEPROM_H__
#define __TASK_EEPROM_H__
#include "main.h"


typedef enum
{
    EEPROM_CMD_WRITE_DATA,
    EEPROM_CMD_READ_DATA,
} eeprom_command_t;

typedef struct
{
    eeprom_command_t command;
    uint16_t        address;
    uint8_t         *data;
    uint16_t         length;
    QueueHandle_t  response_queue;
} eeprom_message_t;

extern QueueHandle_t Queue_EEPROM_Requests;

bool task_eeprom_resources_init(SemaphoreHandle_t *spi_semaphore, cyhal_spi_t *spi_obj, cyhal_gpio_t cs_pin);
void task_eeprom(void *arg);
void eeprom_write(uint8_t *data, uint16_t length);
void eeprom_read(uint8_t *data, uint16_t length);

#endif /* __TASK_EEPROM_H__ */
