/**
 * @file task_poten.c
 * @author Pakorn Jantacumma, Ben Wolf
 * @brief Task for potentiometer. Includes CLI and helper functions.
 * @version 0.1
 * @date 2025-12-05
 * 
 */

#include "task_poten.h"

QueueHandle_t poten_queue;  // Add to globals

static BaseType_t cli_handler_poten(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString);

/* The CLI command definition for the poten_read command */
static const CLI_Command_Definition_t xPoten =
    {
        "poten_read",                 /* command text */
        "\r\npoten_read\r\n",         /* command help text */
        cli_handler_poten,            /* The function to run. */
        0                             /* The user can enter 0 parameters */
    };

// main task
void task_poten(void* param){
     /* Suppress warning for unused parameter */
  (void)param;

  /* Repeatedly running part of the task */
  for (;;)
  {
    float poten_ratio = poten_read();
    // Send to queue (don't block if full)
    xQueueOverwrite(poten_queue, &poten_ratio);  // Overwrites old value
    vTaskDelay(50); // read every 50 ms
  }
}

// initialize potentiometer task
void task_poten_init(){
    // Create queue to hold potentiometer values
    poten_queue = xQueueCreate(1, sizeof(float));  // Queue depth of 1

    poten_init();

    /* Register the CLI command */
    FreeRTOS_CLIRegisterCommand(&xPoten);

    /* Create the task that will interact with the potentiometer */
    xTaskCreate(
      task_poten,
      "Task_poten",
      configMINIMAL_STACK_SIZE,
      NULL,
      configMAX_PRIORITIES - 6,
      NULL);
}

/* FreeRTOS CLI Handler for the 'poten_read' command */
static BaseType_t cli_handler_poten(
    char *pcWriteBuffer,
    size_t xWriteBufferLen,
    const char *pcCommandString)
{
  BaseType_t xReturn;

  /* Remove compile time warnings about unused parameters */
  (void)pcCommandString;
  (void)xWriteBufferLen;
  configASSERT(pcWriteBuffer);

  float poten_ratio;
  while (1) {
    if (xQueueReceive(poten_queue, &poten_ratio, portMAX_DELAY) == pdTRUE) {
      break;
    }
  }

  sprintf(pcWriteBuffer, "\nPercent: %f\n\r", poten_ratio);

  /* Indicate that the command has completed */
  xReturn = pdFALSE;
  return xReturn;
}