/**
 * @file   ws2812b.c
 * @brief  ESP8266 driver for WS2812B
 * @author Ond艡ej Hru拧ka, (c) 2016
 *
 * MIT License
 */

#include "espressif/esp_common.h" // sdk_os_delay_us
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>

#include "ws2812.h"


/** Set one RGB LED color */
void ws2812_set(uint8_t gpio_num, uint32_t rgb)
{
    ws2812_seq_start();
    ws2812_seq_rgb(gpio_num, rgb);
    ws2812_seq_end();
}


/** Set many RGBs */
void ws2812_set_many(uint8_t gpio_num, uint32_t rgb, size_t count)
{
    size_t i;
    ws2812_seq_start();

    for (i = 0; i < count; i++) {
        //uint32_t rgb = *rgbs++;
        ws2812_seq_rgb(gpio_num, rgb);
    }

    ws2812_seq_end();
}


/** Set one RGB to black (when used as indicator) */
void ws2812_off(uint8_t gpio_num)
{
    ws2812_set(gpio_num, 0x000000);
}


void SEND_WS_0()
{
    uint8_t time;

    //写寄存器函数 (GPIO基地址+GPIO管脚地址，高低电平)
    //time = 3; while(time--) WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + 4, 1<<0 );
    //time = 8; while(time--) WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + 8, 1<<0 );
    time =2 ;
    while(time--)
    {
    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1);
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 0);
        }

    time = 4;
    while(time--)
    {
        GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 0);
        GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1);
        }


}

void SEND_WS_1()
{
    uint8_t time;

    //time = 7; while(time--) WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + 4, 1<<0 );
    //time = 5; while(time--) WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + 8, 1<<0 );

    time = 2;
    while(time--)
    {
        GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1);
        GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 0);
        }
    time = 3;
    while(time--)
    {
        GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 0);
        GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1);
        }

    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1);
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 0);
}


void WS2812OutBuffer( uint8_t * buffer, uint16_t length )
{
    uint16_t i;

    GPIO_OUTPUT_SET(0, 0);

    //WRITE_PERI_REG( PERIPHS_GPIO_BASEADDR + 8, 1<<0 );

    //ets_intr_lock();


    for( i = 0; i < length; i++ )
    {
        uint8_t mask = 0x80;
        uint8_t byte = buffer[i];
        while (mask)
        {
            if( byte & mask ) SEND_WS_1(); else SEND_WS_0();
            mask >>= 1;
        }

    }



}
