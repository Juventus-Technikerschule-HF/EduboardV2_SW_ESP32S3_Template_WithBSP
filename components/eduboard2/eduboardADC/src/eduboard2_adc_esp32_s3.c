#include "../../eduboard2.h"
#include "../eduboard2_adc.h"

#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"

#define TAG "ADC_Driver"

#define _ADC_UNIT_STR(unit)         #unit
#define ADC_UNIT_STR(unit)          _ADC_UNIT_STR(unit)
#define ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define ADC_GET_DATA(p_data)        ((p_data)->type2.data)

#if defined(ADC_ATTEN_DB_12)
    #define ADC_ATTENUATION ADC_ATTEN_DB_12
#else
    #define ADC_ATTENUATION ADC_ATTEN_DB_11
#endif

SemaphoreHandle_t hADCMutex;

// static int adc_raw;
// static int voltage;
adc_cali_handle_t adc1_cali_handle = NULL;
static int* adc_raw;
static int* voltage;
static uint16_t** adcbuffer;
void (*adcStreamCallbackFunction)() = NULL;

int8_t channelmap_ANX_TO_ADCX[5] = {-1,-1,-1,-1,-1};
int8_t channelmap_ADCX_TO_ANX[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
adc_channel_t* channels;



/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle, uint32_t samplerate_us, uint32_t buffersize)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = buffersize*2,
        .conv_frame_size = buffersize,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    uint32_t sample_freq = (1000000/samplerate_us)*channel_num;
    ESP_LOGI(TAG, "Set ADC Sampling Frequency to %iHz" ,(int)(sample_freq));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = sample_freq,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_ATTENUATION;
        adc_pattern[i].channel = channel[i];
        adc_pattern[i].unit = ADC_UNIT;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

uint8_t initADCChannels() {
    uint8_t activeChannels = 0;
    #ifdef CONFIG_ENABLE_AN0
    channelmap_ADCX_TO_ANX[AN0_CHANNEL] = activeChannels;
    activeChannels++;
    channelmap_ANX_TO_ADCX[AN0] = AN0_CHANNEL;
    #endif
    #ifdef CONFIG_ENABLE_AN1
    channelmap_ADCX_TO_ANX[AN1_CHANNEL] = activeChannels;
    activeChannels++;
    channelmap_ANX_TO_ADCX[AN1] = AN1_CHANNEL;
    #endif
    #ifdef CONFIG_ENABLE_AN2
    channelmap_ADCX_TO_ANX[AN2_CHANNEL] = activeChannels;
    activeChannels++;
    channelmap_ANX_TO_ADCX[AN2] = AN2_CHANNEL;
    #endif
    #ifdef CONFIG_ENABLE_AN3
    channelmap_ADCX_TO_ANX[AN3_CHANNEL] = activeChannels;
    activeChannels++;
    channelmap_ANX_TO_ADCX[AN3] = AN3_CHANNEL;
    #endif
    #ifdef CONFIG_ENABLE_AN4
    channelmap_ADCX_TO_ANX[AN4_CHANNEL] = activeChannels;
    activeChannels++;
    channelmap_ANX_TO_ADCX[AN4] = AN4_CHANNEL;
    #endif
    if(activeChannels > 0) {
        channels = malloc(sizeof(adc_channel_t)*activeChannels);
        #ifdef CONFIG_ENABLE_ADC_STREAMING
        adcbuffer = malloc(sizeof(uint16_t**) * activeChannels);
        for(int i = 0; i < activeChannels; i++) {
            adcbuffer[i] = malloc(sizeof(uint16_t)*CONFIG_ADC_STREAMING_BUFFERSIZE);
        }
        #endif
        adc_raw = malloc(activeChannels * sizeof(int));
        voltage = malloc(activeChannels * sizeof(int));
    } else {
        channels = NULL;
    }
    uint8_t i = 0;
    #ifdef CONFIG_ENABLE_AN0
    channels[i] = AN0_CHANNEL;
    channelmap_ADCX_TO_ANX[AN0_CHANNEL] = i;
    i++;
    #endif
    #ifdef CONFIG_ENABLE_AN1
    channels[i] = AN1_CHANNEL;
    channelmap_ADCX_TO_ANX[AN1_CHANNEL] = i;
    i++;
    #endif
    #ifdef CONFIG_ENABLE_AN2
    channels[i] = AN2_CHANNEL;
    channelmap_ADCX_TO_ANX[AN2_CHANNEL] = i;
    i++;
    #endif
    #ifdef CONFIG_ENABLE_AN3
    channels[i] = AN3_CHANNEL;
    channelmap_ADCX_TO_ANX[AN3_CHANNEL] = i;
    i++;
    #endif
    #ifdef CONFIG_ENABLE_AN4
    channels[i] = AN4_CHANNEL;
    channelmap_ADCX_TO_ANX[AN4_CHANNEL] = i;
    i++;
    #endif
    return activeChannels;
}

void setADCDataValue(uint8_t channel, int value) {
    if(channelmap_ADCX_TO_ANX[channel] == -1) return;
    adc_raw[channelmap_ADCX_TO_ANX[channelmap_ANX_TO_ADCX[channel]]] = value;
    adc_cali_raw_to_voltage(adc1_cali_handle, value, &voltage[channelmap_ADCX_TO_ANX[channelmap_ANX_TO_ADCX[channel]]]);
    // for(int i = 0; i < sizeof(channels); i++) {
    //     if(channels[i] == channel) {
    //         adc_raw[i] = value;
    //         adc_cali_raw_to_voltage(adc1_cali_handle, value, &voltage[i]);
    //         return;
    //     }
    // }
}
int getADCRawDataValue(uint8_t channel) {
    if(channelmap_ANX_TO_ADCX[channel] != -1) {
        // return adc_raw[channelmap_ADCX_TO_ANX]
    }
    // for(int i = 0; i < sizeof(channels); i++) {
    //     if(channels[i] == channel) {
    //         return adc_raw[i];
    //     }
    // }
    return 0;
}
int getADCVoltageValue(uint8_t channel) {
    for(int i = 0; i < sizeof(channels); i++) {
        if(channels[i] == channel) {
            return voltage[i];
        }
    }
    return 0;
}

void adcTask(void * parameter) {
    esp_err_t ret;
    ESP_LOGI(TAG, "init ADC...");
    hADCMutex = xSemaphoreCreateMutex();
    uint32_t ret_num = 0;
    uint8_t numberofchannels = initADCChannels();
    char unit[] = ADC_UNIT_STR(ADC_UNIT);
    adc_calibration_init(ADC_UNIT, ADC_ATTENUATION, &adc1_cali_handle);
#ifdef CONFIG_ENABLE_ADC_STREAMING
    
    uint8_t* result;
    uint32_t buffersize = CONFIG_ADC_STREAMING_BUFFERSIZE*SOC_ADC_DIGI_RESULT_BYTES*numberofchannels;
    result = malloc(buffersize);
    memset(result, 0xcc, buffersize);
    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channels, numberofchannels, &handle, ADC_STREAM_SAMPLERATE_US, buffersize);
    adc_continuous_start(handle);
    uint32_t n[5] = {0,0,0,0,0};
    for(;;) {
        for(int i = 0; i < 5; i++) {
            n[i] = 0;
        }
        ret = adc_continuous_read(handle, result, buffersize, &ret_num, portMAX_DELAY);
        if (ret == ESP_OK) {
            // ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                uint32_t chan_num = ADC_GET_CHANNEL(p);
                uint32_t data = ADC_GET_DATA(p);
                /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
                if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT)) {
                    adcbuffer[channelmap_ADCX_TO_ANX[chan_num]][n[channelmap_ADCX_TO_ANX[chan_num]]++] = data;
                    // n[channelmap_ADCX_TO_ANX[chan_num]]++;

                    // ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, unit, chan_num, data);
                } else {
                    ESP_LOGE(TAG, "Invalid ADC Data");
                    // ESP_LOGW(TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
                }
            }
            // ESP_LOGW(TAG, "channelmap_ANX_TO_ADCX: %i\t %i\t %i\t %i\t %i", (int)channelmap_ANX_TO_ADCX[0], (int)channelmap_ANX_TO_ADCX[1], (int)channelmap_ANX_TO_ADCX[2], (int)channelmap_ANX_TO_ADCX[3], (int)channelmap_ANX_TO_ADCX[4]);
            // ESP_LOGW(TAG, "channelmap_ADCX_TO_ANX: %i\t %i\t %i\t %i\t %i\t %i\t %i\t %i\t %i\t %i", (int)channelmap_ADCX_TO_ANX[0], (int)channelmap_ADCX_TO_ANX[1], (int)channelmap_ADCX_TO_ANX[2], (int)channelmap_ADCX_TO_ANX[3], (int)channelmap_ADCX_TO_ANX[4], (int)channelmap_ADCX_TO_ANX[5], (int)channelmap_ADCX_TO_ANX[6], (int)channelmap_ADCX_TO_ANX[7], (int)channelmap_ADCX_TO_ANX[8], (int)channelmap_ADCX_TO_ANX[9]);
            // ESP_LOGW(TAG, "Data received:          n0:%i - n1:%i - n2:%i - n3:%i - n4:%i", (int)n[0], (int)n[1], (int)n[2], (int)n[3], (int)n[4]);
            if(adcStreamCallbackFunction != NULL) {
                (*adcStreamCallbackFunction)();
            }
            if(n[0] < CONFIG_ADC_STREAMING_BUFFERSIZE) {
                ESP_LOGE(TAG, "Not enough Data received!");
            }
            // vTaskDelay(1);
        }
    }
#else
    uint8_t result[SOC_ADC_DIGI_RESULT_BYTES];
    memset(result, 0xcc, 1*SOC_ADC_DIGI_RESULT_BYTES);
    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channels, numberofchannels, &handle, 1000, 1); //Updaterate: 10ms
    adc_continuous_start(handle);
    for(;;) {
        ret = adc_continuous_read(handle, result, 1*SOC_ADC_DIGI_RESULT_BYTES, &ret_num, 0);
        if (ret == ESP_OK) {
            // ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                uint32_t chan_num = ADC_GET_CHANNEL(p);
                uint32_t data = ADC_GET_DATA(p);
                /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
                if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT)) {
                    // ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, unit, chan_num, data);
                    xSemaphoreTake(hADCMutex, portMAX_DELAY);
                    setADCDataValue(chan_num, data);
                    xSemaphoreGive(hADCMutex);
                }
            }
            vTaskDelay(1);
        }
    }
#endif

//     //-------------ADC1 Init---------------//
//     adc_oneshot_unit_handle_t adc1_handle;
//     adc_oneshot_unit_init_cfg_t init_config1 = {
//         .unit_id = ADC_UNIT_1,
//     };
//     ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    
//     //-------------ADC1 Config---------------//
//     adc_oneshot_chan_cfg_t config = {
//         .bitwidth = ADC_BITWIDTH_DEFAULT,
//         .atten = ADC_ATTENUATION
//     };
//     ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, AN0_CHANNEL, &config));
//     //-------------ADC1 Calibration Init---------------//
//     // adc_cali_handle_t adc1_cali_handle = NULL;
// #ifdef ADC_ATTEN_DB_12
//     bool do_calibration1 = adc_calibration_init(ADC_UNIT_1, ADC_ATTENUATION, &adc1_cali_handle);
// #else
//     bool do_calibration1 = adc_calibration_init(ADC_UNIT_1, ADC_ATTENUATION, &adc1_cali_handle);
// #endif
//     ESP_LOGI(TAG, "ADC init done");
//     for(;;) {
//         xSemaphoreTake(hADCMutex, portMAX_DELAY);
//         ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, AN0_CHANNEL, &adc_raw));

// #ifdef CONFIG_ADC_DEBUG
//         ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, AN0_CHANNEL, adc_raw);
// #endif
//         if (do_calibration1) {
//             ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage));
// #ifdef CONFIG_ADC_DEBUG
//             ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, AN0_CHANNEL, voltage);
// #endif
//         }
//         xSemaphoreGive(hADCMutex);
//         vTaskDelay(pdMS_TO_TICKS(50));
//     }
}
uint32_t adc_get_raw(uint8_t adc_channel) {
    uint32_t returnValue = 0;
    xSemaphoreTake(hADCMutex, portMAX_DELAY);
    // returnValue = getADCRawDataValue(adc_channel);
    xSemaphoreGive(hADCMutex);
    return returnValue;
}
uint32_t adc_get_voltage_mv(uint8_t adc_channel) {
    uint32_t returnValue = 0;
    xSemaphoreTake(hADCMutex, portMAX_DELAY);
    returnValue = getADCVoltageValue(adc_channel);
    xSemaphoreGive(hADCMutex);
    return returnValue;
}
uint32_t adc_get_buffer(uint8_t adc_channel, uint16_t* buffer) {
    if(channelmap_ANX_TO_ADCX[adc_channel] == -1) return 0;
    for(uint32_t i = 0; i < CONFIG_ADC_STREAMING_BUFFERSIZE; i++) {
        buffer[i] = adcbuffer[channelmap_ADCX_TO_ANX[channelmap_ANX_TO_ADCX[adc_channel]]][i];
    }
    return CONFIG_ADC_STREAMING_BUFFERSIZE;
}

void adc_set_stream_callback(void* stream_callback_function) {
#ifdef CONFIG_ENABLE_ADC_STREAMING
    adcStreamCallbackFunction = stream_callback_function;
#endif
}

void eduboard_init_adc() {
    xTaskCreate(adcTask, "adcTask", 4 * 2048, NULL, 5, NULL);
}