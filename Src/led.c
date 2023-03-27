//
// Created by Dmitriy on 25.03.2023.
//


#include "led.h"

extern uint8_t data_buffer_raw[BUFFER_SIZE];
extern data_led data_buffer;
extern volatile int16_t distance_sm;
extern int16_t k;
extern int16_t b;

void write_byte_to_buffer_raw(uint16_t byte_address, uint8_t value)
{
    for (int i = 0; i < 8; ++i) {
        if(value & 1<<(7-i)){
            data_buffer_raw[byte_address + i] = T1H;
        }
        else
        {
            data_buffer_raw[byte_address + i] = T0H;
        }
    }
}

void set_led_value(uint16_t led_num, uint8_t red_light, uint8_t green_light, uint8_t blue_light)
{
    write_byte_to_buffer_raw(DELAY_COUNT_TICK + 24 * (uint16_t) led_num, green_light);
    write_byte_to_buffer_raw(DELAY_COUNT_TICK + 24 * (uint16_t) led_num + 8, red_light);
    write_byte_to_buffer_raw(DELAY_COUNT_TICK + 24 * (uint16_t) led_num + 16, blue_light);

    data_buffer.led[led_num].green = green_light;
    data_buffer.led[led_num].red = red_light;
    data_buffer.led[led_num].blue = blue_light;
}

void set_color_background(void ){
    for (int i = 0; i < COUNT_LED; ++i) {
        set_led_value(i,0,0,5);
    }
}

void up_light_to_max(void ){
    while (1)
    {
        if (distance_sm < DISTANCE_LIGHT_ON_SM)
        {
            for (int i = 0; i < COUNT_LED; ++i) {
                if(data_buffer.led[i].red < 240) data_buffer.led[i].red += 10;
                if(data_buffer.led[i].green < 240) data_buffer.led[i].green += 10;
                if(data_buffer.led[i].blue < 128) data_buffer.led[i].blue += 10;

                set_led_value(i, data_buffer.led[i].red, data_buffer.led[i].green, data_buffer.led[i].blue);
                HAL_Delay(1);
            }
        }
        else
        {
            if (data_buffer.led[0].red > 1 && distance_sm > DISTANCE_LIGHT_ON_SM)
            {
                for (int i = 0; i < COUNT_LED; ++i)
                {
                    if(data_buffer.led[i].red > 1) data_buffer.led[i].red -= 1;
                    if(data_buffer.led[i].green > 1) data_buffer.led[i].green -= 1;
                    if(data_buffer.led[i].blue > 1) data_buffer.led[i].blue -= 1;

                    set_led_value(i, data_buffer.led[i].red, data_buffer.led[i].green, data_buffer.led[i].blue);
                    HAL_Delay(1);
                }
            }
            else
            {
                break;
            }
        }
    }
}

void two_led_to_one(uint16_t position, uint8_t red, uint8_t green, uint8_t blue){
    for (int i = 0; i < COUNT_LED/2; i++) {
        if (i > position)
        {
            set_led_value(i, 0, 0, 0);
            set_led_value(COUNT_LED - i - 1, 0, 0, 0);
        }
        else
        {
            set_led_value(i, red, green, blue);
            set_led_value(COUNT_LED - i - 1, red, green, blue);
        }
    }
}

void run_led(void ){
    uint16_t curr_position_led = k * distance_sm / 10 + b;
    two_led_to_one(curr_position_led, 50, 0, 0);
}

void fill_light(void ){
    for (int i = 0; i < COUNT_LED; ++i) {
        set_led_value(i, 50, 50, 50);
        HAL_Delay(10);
    }
}