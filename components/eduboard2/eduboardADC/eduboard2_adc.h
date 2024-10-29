#pragma once

#include "hal/adc_types.h"

#ifdef CONFIG_ENABLE_AN0
    #define ADC0    0
#endif
#ifdef CONFIG_ENABLE_AN1
    #define ADC0    1
#endif
#ifdef CONFIG_ENABLE_AN2
    #define ADC0    2
#endif
#ifdef CONFIG_ENABLE_AN3
    #define ADC0    3
#endif
#ifdef CONFIG_ENABLE_AN4
    #define ADC0    4
#endif

uint32_t adc_get_raw(uint8_t adc_channel);
uint32_t adc_get_voltage_mv(uint8_t adc_channel);
void adc_set_stream_callback(void* stream_callback_function);
void eduboard_init_adc();