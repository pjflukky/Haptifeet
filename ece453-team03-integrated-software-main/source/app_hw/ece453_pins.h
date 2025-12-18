/**
 * @file ece453_pins.h
 * @author Joe Krachey (jkrachey@wisc.edu)
 * @brief 
 * @version 0.1
 * @date 2025-05-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __ECE453_PINS_H__
#define __ECE453_PINS_H__

/* Include Infineon BSP Libraries */
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

/****************/
/* Final Design */
/****************/

#define HAPTIC_SCL    P9_0
#define HAPTIC_SDA    P9_1
#define HAPTIC_SEL_A  P5_6
#define HAPTIC_SEL_B  P5_5
#define HAPTIC_SEL_C  P5_4

#define LIDAR_RX      P6_4
#define LIDAR_TX      P6_5

#define EEPROM_TX     P10_0
#define EEPROM_RX     P10_1
#define EEPROM_SCLK   P10_2
#define EEPROM_CS_N   P10_3

#define POTEN_S       P10_6
#define POTEN_IO      P10_5

#endif
