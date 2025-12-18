/**
 * @file poten.h
 * @author Ben Wolf
 * @brief Header file for the potentiometer source file.
 * @version 0.1
 * @date 2025-12-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __poten_H__
#define __poten_H__

#include "main.h"
#include "ece453_pins.h"

#define POTEN_MAX 65520.0f

void poten_init(void);
float poten_read(void);

#endif