/**
Radiostyrd is a remote controlled toy car.
Copyright (C) 2020  Andreas Dahlberg

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
**/

//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

///////////////////////////////////////////////////////////////////////////////
//INCLUDES
///////////////////////////////////////////////////////////////////////////////

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "monitor.h"
#include "Filter.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

///////////////////////////////////////////////////////////////////////////////
//DEFINES
///////////////////////////////////////////////////////////////////////////////

#define NUMBER_OF_CHANNELS 2
#define ADC_ATTEN ADC_ATTEN_DB_2_5

#define IDLE_CURRENT 7  // Idle current in mA

///////////////////////////////////////////////////////////////////////////////
//FUNCTION PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

static void init_channels(void);
static void setup_adc_calibration(void);
static void measurement_task(void * parameter);
static uint32_t get_total_current(void);
static uint32_t get_motor_current(void);

///////////////////////////////////////////////////////////////////////////////
//TYPES
///////////////////////////////////////////////////////////////////////////////

struct channel_config_t
{
    adc1_channel_t channel;
    adc_atten_t attenuation;
    adc_bits_width_t width;
    esp_adc_cal_characteristics_t *calibration_data_p;
};

struct pid_t
{
    int32_t p;
    int32_t i;
    float cv;
    struct pid_parameters_t parameters;
};

struct monitor_t
{
    struct channel_config_t channels[NUMBER_OF_CHANNELS];
    struct filter_t filter;
    struct pid_t pid;
    uint32_t stall_current;
    uint32_t stall_counter;
    uint32_t stall_counter_max;
};

///////////////////////////////////////////////////////////////////////////////
//VARIABLES
///////////////////////////////////////////////////////////////////////////////

static struct monitor_t monitor = {
    .channels = {
        {
            .channel = ADC1_CHANNEL_6,
            .attenuation = ADC_ATTEN_DB_2_5,
            .width = ADC_WIDTH_BIT_12
        },
        {
            .channel = ADC1_CHANNEL_7,
            .attenuation = ADC_ATTEN_DB_11,
            .width = ADC_WIDTH_BIT_12
        },
    },
    .pid = {
        .p = 0,
        .i = 0,
        .cv = 0,
        .parameters = {
            .kp = 0.07,
            .ki = 0.09,
            .kd = 0.0,
            .sp = 800,
            .imax = 700
        }
    },
    .stall_current = 420,
    /**
     * With a sampling frequency of 100 Hz, 'stall_counter_max' gives a time of
     * ('stall_counter_max' * 10) ms before stall is decided.
     */
    .stall_counter_max = 5
};

static const char* TAG = "monitor";

///////////////////////////////////////////////////////////////////////////////
//FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

void monitor_init(void)
{
    init_channels();
    setup_adc_calibration();

    TaskHandle_t xHandle = NULL;
    xTaskCreate(measurement_task, "Measurement", 2048, NULL, tskIDLE_PRIORITY, &xHandle);

    ESP_LOGI(TAG, "Initialized");
}

uint32_t monitor_get_battery_voltage(void)
{
    struct channel_config_t *config_p = &monitor.channels[1];

    const size_t number_of_samples = 8;
    uint32_t adc_reading = 0;

    for (size_t i = 0; i < number_of_samples; ++i)
    {
        adc_reading += adc1_get_raw((adc1_channel_t)config_p->channel);
    }

    adc_reading /= number_of_samples;

    return esp_adc_cal_raw_to_voltage(adc_reading, config_p->calibration_data_p) * 2;
}

uint32_t monitor_get_motor_current(void)
{
    return Filter_Output(&monitor.filter);
}

float monitor_get_suggested_duty(void)
{
    return monitor.pid.cv;
}

void monitor_set_pid_parameters(const struct pid_parameters_t *parameters)
{
    assert(parameters != NULL);

    ESP_LOGI(TAG, "PID(kp=%.2f, ki=%.2f, kd=%.2f, sp=%u)", parameters->kp, parameters->ki, parameters->kd, parameters->sp);

    //TODO: Lock?
    monitor.pid.parameters = *parameters;
}

struct pid_parameters_t monitor_get_pid_parameters(void)
{
    return monitor.pid.parameters;
}

bool monitor_is_motor_stalled(void)
{
    return monitor.stall_counter >= monitor.stall_counter_max;
}

///////////////////////////////////////////////////////////////////////////////
//LOCAL FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

static uint32_t get_total_current(void)
{
    struct channel_config_t *config_p = &monitor.channels[0];

    const size_t number_of_samples = 64;
    uint32_t adc_reading = 0;

    for (size_t i = 0; i < number_of_samples; ++i)
    {
        adc_reading += adc1_get_raw((adc1_channel_t)config_p->channel);
    }

    adc_reading /= number_of_samples;

    return esp_adc_cal_raw_to_voltage(adc_reading, config_p->calibration_data_p);
}

static uint32_t get_motor_current(void)
{
    uint32_t total_current = get_total_current();

    if (total_current < IDLE_CURRENT)
    {
        ESP_LOGW(TAG, "Low idle current detected (%u mA)", total_current);
        return 0;
    }

    return total_current - IDLE_CURRENT;
}

static float apply_cv_limits(float cv)
{
    if (cv > 100.0)
    {
        cv = 100.0;
    }
    else if (cv < 0.0)
    {
        cv = 0.0;
    }

    return cv;
}

static void check_for_stall(void)
{
    uint32_t current = Filter_Output(&monitor.filter);
    if (current > monitor.stall_current)
    {

        if (monitor.stall_counter < monitor.stall_counter_max)
        {
            monitor.stall_counter += 1;
        }
    }
    else
    {
        const uint32_t step = 2;
        if (monitor.stall_counter >= step)
        {
            monitor.stall_counter -= step;
        }
        else
        {
            monitor.stall_counter = 0;
        }
    }
}

static void measurement_task(void * parameter) {

    Filter_Init(&monitor.filter, get_motor_current(), FILTER_ALPHA(0.1));

    while (1)
    {
        uint32_t pv = get_motor_current();
        Filter_Process(&monitor.filter, pv);

        check_for_stall();

        // TODO: Use filtered current!
        monitor.pid.p = monitor.pid.parameters.sp - pv;
        monitor.pid.i = monitor.pid.i + monitor.pid.p;
        if (monitor.pid.i > monitor.pid.parameters.imax)
        {
            monitor.pid.i = monitor.pid.parameters.imax;
        }

        float cv  = (monitor.pid.p * monitor.pid.parameters.kp) + (monitor.pid.i * monitor.pid.parameters.ki);
        cv = apply_cv_limits(cv);

        monitor.pid.cv = cv;

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void init_channels(void)
{
    for (size_t i = 0; i < NUMBER_OF_CHANNELS; ++i)
    {
        const struct channel_config_t *channel_config_p = &monitor.channels[i];

        adc1_config_width(channel_config_p->width);
        adc1_config_channel_atten(channel_config_p->channel, channel_config_p->attenuation);
    }
}

static void setup_adc_calibration(void)
{
    for (size_t i = 0; i < sizeof(monitor.channels) / sizeof(monitor.channels[0]); ++i)
    {
        const uint32_t default_vref = 1100;
        struct channel_config_t *config_p = &monitor.channels[i];

        config_p->calibration_data_p = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_value_t val_type  = esp_adc_cal_characterize(ADC_UNIT_1, config_p->attenuation, config_p->width, default_vref, config_p->calibration_data_p);

        if (val_type != ESP_ADC_CAL_VAL_EFUSE_VREF) {
            ESP_LOGW(TAG, "Not using EFUSE VREF(%u)", val_type);
        }
    }
}

