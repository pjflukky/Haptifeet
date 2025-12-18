/**
 * @file task_lidar.c
 * @author Wei Chen Zhang, Ben Wolf
 * @brief Task for the LiDAR sensor. Creates a queue to send LiDAR data over, as well as a CLI and helper functions.
 * @version 0.1
 * @date 2025-12-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "task_lidar.h"

static point_t buf[LIDAR_MAX_POINTS];
static int buf_len = 0;

QueueHandle_t Queue_LIDAR_Requests, Queue_LIDAR_Responses;

static BaseType_t cli_handler_lidar_print(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString);

static const CLI_Command_Definition_t xLIDAR_Print =
    {
        "lidar_print",
        "\r\nlidar_print\r\n",
        cli_handler_lidar_print,
        0,
};


/**
 * @brief Blocking read request for LiDAR scan data
 *
 * @param data point_t array of length <LIDAR_MAX_POINTS>
 */
void lidar_read(point_t * data)
{
    uint8_t request = 1;
    xQueueSend(Queue_LIDAR_Requests, &request, portMAX_DELAY);

    while (1)
    {
        if (xQueueReceive(
                Queue_LIDAR_Responses,
                data,
                portMAX_DELAY) == pdTRUE)
        {
            return;
        }
        vTaskDelay(10);
    }
}

static BaseType_t cli_handler_lidar_print(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString)
{
    // 1. Use static variables to maintain state across multiple calls
    static point_t data[LIDAR_MAX_POINTS];
    static int output_index = -1; 

    (void)pcCommandString;
    (void)xWriteBufferLen;
    configASSERT(pcWriteBuffer);

    // 2. On the very first call (index 0), fetch the LiDAR data
    if (output_index == -1)
    {
        lidar_read(data);
        sprintf(pcWriteBuffer, "\r\n\r\n===== FULL ROTATION =====\r\n\r\n");
        vTaskDelay(5);
        output_index++;
        return pdTRUE;
    }

    // 3. check if we have valid data to print
    if (output_index < LIDAR_MAX_POINTS && data[output_index].valid)
    {
        // Write exactly ONE point to the buffer
        sprintf(pcWriteBuffer, "Point %d: %.2f deg  %.1f mm\r\n", 
                 output_index, 
                 data[output_index].angle, 
                 data[output_index].dist);

        // Move to the next point for the next call
        output_index++;
        vTaskDelay(5);

        // Return pdTRUE: "I am not done, call me again immediately"
        return pdTRUE;
    }
    else
    {
        sprintf(pcWriteBuffer, "\r\n===== END OF ROTATION (%d POINTS) =====\r\n", output_index-1);
        vTaskDelay(5);

        // 4. We hit an invalid point or the end of the array
        // Reset index to 0 so the command works correctly next time
        output_index = -1;

        // Return pdFALSE: "I am finished, stop calling me"
        return pdFALSE;
    }
}

void task_lidar(void *arg)
{
    uint8_t request;

    while (1)
    {
        xQueueReceive(
            Queue_LIDAR_Requests,
            &request,
            portMAX_DELAY);

        if (request == 1)
        {
            collect_one_rotation(buf, &buf_len);
            point_t response_buf[LIDAR_MAX_POINTS];
            for (int i = 0; i < LIDAR_MAX_POINTS; i++)
            {
                response_buf[i] = buf[i];
            }
            xQueueSend(
                Queue_LIDAR_Responses,
                response_buf,
                portMAX_DELAY);
        }
    }
}

void task_lidar_init(void)
{
    lidar_init();

    Queue_LIDAR_Requests = xQueueCreate(1, 1 * sizeof(uint8_t));
    Queue_LIDAR_Responses = xQueueCreate(1, 500 * sizeof(point_t));

    FreeRTOS_CLIRegisterCommand(&xLIDAR_Print);

    xTaskCreate(
        task_lidar,
        "LIDAR",
        20 * configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL);
}
