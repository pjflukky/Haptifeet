/**
 * @file task_haptics.h
 * @author Isak Keyes
 * @brief Header file for the haptics task and request functions
 * @date 2025-10-06
 * 
 */
#ifndef __TASK_HAPTICS_H__
#define __TASK_HAPTICS_H__

#include "main.h"

/* Number of haptics connected */
#define NUM_HAPTICS 8

void req_haptic_update(uint8_t haptic_id, uint8_t amplitude);

void req_haptic_update_all(uint8_t amplitude);

void task_haptics_init(void);

#endif /* __TASK_HAPTICS_H__ */
