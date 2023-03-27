//
// Created by Dmitriy on 25.03.2023.
//

#ifndef AUTOLEDADDRESS_LED_H
#define AUTOLEDADDRESS_LED_H

#include "main.h"

void write_byte_to_buffer_raw(uint16_t byte_address, uint8_t value);
void set_led_value(uint16_t led_num, uint8_t red_light, uint8_t green_light, uint8_t blue_light);
void set_color_background(void );
void up_light_to_max(void );
void two_led_to_one(uint16_t position, uint8_t red, uint8_t green, uint8_t blue);
void run_led(void );
void fill_light(void );

#endif //AUTOLEDADDRESS_LED_H
