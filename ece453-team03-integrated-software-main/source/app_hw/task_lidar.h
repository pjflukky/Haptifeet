/**
 * @file task_lidar.hs
 * @author Wei Chen Zhang, Ben Wolf
 * @brief Header file for the LiDAR task.
 * @version 0.1
 * @date 2025-12-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __TASK_LIDAR_H__
#define __TASK_LIDAR_H__

#include "main.h"
#include "lidar.h"

void lidar_read(point_t *data);

void task_lidar_init(void);
void task_lidar(void *args);

#endif