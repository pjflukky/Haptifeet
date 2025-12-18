/**
 * @file poten.c
 * @author Pakorn Jantacumma
 * @brief Source file for getting data from the potentiometer.
 * @version 0.1
 * @date 2025-12-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "poten.h"

cyhal_adc_channel_t adc_chan;
cyhal_adc_t adc_obj;

void poten_init(void) {
    cy_rslt_t rslt;

    rslt = cyhal_gpio_init(POTEN_S, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    rslt = cyhal_adc_init(&adc_obj, POTEN_IO, NULL);
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);
    rslt = cyhal_adc_channel_init_diff(&adc_chan, &adc_obj, POTEN_IO, CYHAL_ADC_VNEG, NULL);
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);
}

/*
* Reads from the potentiometer and returns a normalized value
*/
float poten_read(void) {
    uint16_t read = cyhal_adc_read_u16(&adc_chan);
    float actual = ((float)read / POTEN_MAX); // normalized between 0 to 1
    return actual;
}
