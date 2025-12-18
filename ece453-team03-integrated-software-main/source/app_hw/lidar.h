/**
 * @file lidar.h
 * @brief header file for the lidar source file
 * @version 0.1
 * @author Weichen Zhang, Ben Wolf
 * @date 2025-12-05
 */
#ifndef __LIDAR_H__
#define __LIDAR_H__

#include "main.h"

#define LIDAR_MAX_POINTS 500    // maximum number of points per LiDAR scan
#define LIDAR_MAX_RANGE 12000.0f // maximum distance (mm) of LiDAR sensor

typedef struct
{
    float angle; // in degrees
    float dist;  // in mm
    bool valid;
} point_t;

void lidar_init(void);
bool valid_frame(uint8_t d[]);
void get_valid_frame(uint8_t d[5]);
void collect_one_rotation(point_t *buf, int *buf_len);

#endif