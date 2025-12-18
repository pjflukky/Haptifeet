/**
 * @file poten.c
 * @author Pakorn Jantacumma
 * @brief Header file for the potentiometer task.
 * @version 0.1
 * @date 2025-12-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __TASK_poten_H__
#define __TASK_poten_H__

#include "main.h"
#include "ece453_pins.h"
#include "poten.h"

extern QueueHandle_t poten_queue;

void task_poten(void* param);
void task_poten_init(void);

#endif