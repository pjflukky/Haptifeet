/**
 * @file lidar.c
 * @author Wei Chen Zhang, Ben Wolf
 * @brief Source file for getting data from the LiDAR sensor.
 * @version 0.1
 * @date 2025-12-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "lidar.h"
#include "ece453_pins.h"
#include <math.h>

#define LIDAR_BAUD_RATE 460800
#define RX_BUF_SIZE 128

static cyhal_uart_t uart;

static uint8_t hw_rxbuf[RX_BUF_SIZE];

static void send_cmd2(uint8_t c0, uint8_t c1)
{
    cyhal_uart_putc(&uart, c0);
    cyhal_uart_putc(&uart, c1);
}


static void wait_scan_header(void)
{
    uint8_t w[7];
    int f = 0;

    while (f < 7)
        cyhal_uart_getc(&uart, &w[f++], 200);

    for (;;)
    {
        if (w[0]==0xA5 && w[1]==0x5A && w[2]==0x05 &&
            w[3]==0x00 && w[4]==0x00 && w[5]==0x40 && w[6]==0x81)
            return;

        for (int i=0;i<6;i++)
            w[i] = w[i+1];
        cyhal_uart_getc(&uart, &w[6], 200);
    }
}

inline bool valid_frame(uint8_t d[])
{
    uint8_t S = d[0] & 1;
    uint8_t S_n = (d[0] >> 1) & 1;
    uint8_t C = d[1] & 1;
    return (((S ^ S_n) & C) == 1);
}

void get_valid_frame(uint8_t d[5])
{
    for (int i=0;i<5;i++)
        cyhal_uart_getc(&uart, &d[i], 200);

    for (;;)
    {
        if (valid_frame(d))
            return;

        d[0] = d[1];
        d[1] = d[2];
        d[2] = d[3];
        d[3] = d[4];

        cyhal_uart_getc(&uart, &d[4], 200);
    }
}

inline float parse_angle(uint8_t d[])
{
    uint16_t aq6 = ((uint16_t)d[2] << 7) | (d[1] >> 1);
    return (aq6 / 64.0f);
}

inline float parse_dist_mm(uint8_t d[])
{
    uint16_t q2 = ((uint16_t)d[4] << 8) | d[3];
    if (q2 == 0) return -1;
    return q2 / 4.0f;
}

void collect_one_rotation(point_t *buf, int *buf_len)
{
    uint8_t d[5];
    int points = 0;

    for (;;)
    {
        get_valid_frame(d);
        if ((d[0] & 1) == 1)
            break;
    }

    float a = parse_angle(d);
    float dist = parse_dist_mm(d);
    if (dist >= 0)
        buf[points++] = (point_t){.angle = a, .dist = dist, .valid = true};

    uint8_t last_s = 1;

    for (;;)
    {
        get_valid_frame(d);

        uint8_t S = d[0] & 1;

        if (S == 1 && last_s == 0) {
            *buf_len = points;
            buf[points+1] = (point_t){.angle = 0, .dist = 0, .valid = false};
            return;
        }

        float a2 = parse_angle(d);
        float d2 = parse_dist_mm(d);

        if (d2 >= 0 && points < LIDAR_MAX_POINTS)
            buf[points++] = (point_t){.angle = a2, .dist = d2, .valid = true};

        last_s = S;
    }
}

void lidar_init(void)
{
    const cyhal_uart_cfg_t cfg = {
        .data_bits = 8,
        .stop_bits = 1,
        .parity    = CYHAL_UART_PARITY_NONE,
        .rx_buffer = hw_rxbuf,
        .rx_buffer_size = RX_BUF_SIZE
    };

    cyhal_uart_init(&uart,
                    LIDAR_TX,
                    LIDAR_RX,
                    NC, NC,
                    NULL,
                    &cfg);

    cyhal_uart_set_baud(&uart, LIDAR_BAUD_RATE, NULL);

    send_cmd2(0xA5, 0x20);  
    wait_scan_header();
}
