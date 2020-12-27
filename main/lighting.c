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

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "lighting.h"
#include "utils.h"

///////////////////////////////////////////////////////////////////////////////
//DEFINES
///////////////////////////////////////////////////////////////////////////////

#define GPIO_HEADLIGHT_PWM 33

#define GPIO_INDICATOR_R_PWM 15
#define GPIO_INDICATOR_G_PWM 32
#define GPIO_INDICATOR_B_PWM 14

#define PWM_MAX_VALUE 4096
#define DEFAULT_HEADLIGHTS_FADE_TIME_MS 200
#define DEFAULT_INDICATOR_FADE_TIME_MS 750

#define HEADLIGHTS_CHANNEL LEDC_CHANNEL_3
#define INDICATOR_R_CHANNEL LEDC_CHANNEL_4
#define INDICATOR_G_CHANNEL LEDC_CHANNEL_5
#define INDICATOR_B_CHANNEL LEDC_CHANNEL_6

///////////////////////////////////////////////////////////////////////////////
//FUNCTION PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

static void init_headlight_pwm(void);
static void init_indicator_pwm(void);
static uint32_t brightness_to_duty(float brightness);
static void fade_headlights(float brightness);
static void fade_led(ledc_channel_t channel, float brightness, uint32_t fade_time_ms, ledc_fade_mode_t mode);
static void reset_indicator();
static void set_indicator_color(enum color_t color, float brightness);
static void led_steady_task(void *parameter);
static void led_pulse_task(void * parameter);
static void delete_active_task(void);
static inline bool has_active_task(void);

///////////////////////////////////////////////////////////////////////////////
//TYPES
///////////////////////////////////////////////////////////////////////////////

struct headlights_t
{
  float brightness;
  float low_beam_brightness;
  float high_beam_brightness;
  uint32_t fade_time_ms;
  bool on;
};

struct indicator_t
{
  float brightness;
  uint32_t fade_time_ms;
  enum color_t color;
  enum mode_t mode;
  TaskHandle_t task_handle;
};

struct lighting_t {
  struct headlights_t headlights;
  struct indicator_t indicator;
};

///////////////////////////////////////////////////////////////////////////////
//VARIABLES
///////////////////////////////////////////////////////////////////////////////

static const char* TAG = "lighting";

static struct lighting_t lighting;

///////////////////////////////////////////////////////////////////////////////
//FUNCTIONs
///////////////////////////////////////////////////////////////////////////////
void lighting_init(void)
{
  lighting = (typeof(lighting)) {0};
  lighting.headlights.fade_time_ms = DEFAULT_HEADLIGHTS_FADE_TIME_MS;
  lighting.headlights.low_beam_brightness = 0.15;
  lighting.headlights.high_beam_brightness = 1.0;
  lighting.headlights.brightness = lighting.headlights.low_beam_brightness;

  lighting.indicator.brightness = 0.2;
  lighting.indicator.fade_time_ms = DEFAULT_INDICATOR_FADE_TIME_MS;
  lighting.indicator.color = INDICATOR_BLUE;
  lighting.indicator.mode = INDICATOR_OFF;

  init_headlight_pwm();
  init_indicator_pwm();

  ledc_fade_func_install(0);

  lighting_headlight_on();

  ESP_LOGI(TAG, "Initialized");
}

void lighting_set_headlight_limits(float low_beam, float high_beam)
{
  ESP_LOGD(TAG, "%s(%.2f, %.2f)", __FUNCTION__, low_beam, high_beam);

  lighting.headlights.low_beam_brightness = low_beam;
  lighting.headlights.high_beam_brightness = high_beam;
}

void lighting_headlight_on(void)
{
  ESP_LOGD(TAG, "%s()", __FUNCTION__);

  lighting.headlights.on = true;
  fade_headlights(lighting.headlights.brightness);
}

void lighting_headlight_off(void)
{
  ESP_LOGD(TAG, "%s()", __FUNCTION__);

  lighting.headlights.on = false;
  fade_headlights(0);
}

void lighting_headlight_toggle(void)
{
  ESP_LOGD(TAG, "%s()", __FUNCTION__);

  if (lighting.headlights.on)
  {
    lighting_headlight_off();
  }
  else
  {
    lighting_headlight_on();
  }
}

void lighting_headlight_toggle_high_low_beam(void)
{
  ESP_LOGD(TAG, "%s()", __FUNCTION__);
  if (lighting.headlights.on)
  {
    if (lighting.headlights.brightness == lighting.headlights.low_beam_brightness)
    {
      lighting_headlight_high_beam();
    }
    else
    {
      lighting_headlight_low_beam();
    }
  }
}

void lighting_headlight_low_beam(void)
{
  ESP_LOGD(TAG, "%s()", __FUNCTION__);

  lighting.headlights.on = true;
  lighting.headlights.brightness = lighting.headlights.low_beam_brightness;
  fade_headlights(lighting.headlights.low_beam_brightness);
}

void lighting_headlight_high_beam(void)
{
  ESP_LOGD(TAG, "%s()", __FUNCTION__);

  lighting.headlights.on = true;
  lighting.headlights.brightness = lighting.headlights.high_beam_brightness;
  fade_headlights(lighting.headlights.high_beam_brightness);
}

void lighting_set_headlight_brightness(float brightness)
{
  ESP_LOGD(TAG, "%s(%.2f)", __FUNCTION__, brightness);

  lighting.headlights.brightness = brightness;
  fade_headlights(brightness);
}

void lighting_set_indicator_color(enum color_t color)
{
  ESP_LOGD(TAG, "%s(%u)", __FUNCTION__, (uint32_t)color);
  lighting.indicator.color = color;
}

void lighting_set_indicator_mode(enum mode_t mode)
{
  ESP_LOGD(TAG, "%s(%u)", __FUNCTION__, (uint32_t)mode);
  //if (mode != lighting.indicator.mode)
  //{
  lighting.indicator.mode = mode;

  if (has_active_task())
  {
    delete_active_task();
  }

  switch (mode)
  {
  case INDICATOR_OFF:
    ESP_LOGD(TAG, "%s Changed to OFF",  __FUNCTION__);
    reset_indicator();
    break;

  case INDICATOR_STEADY:
    ESP_LOGD(TAG, "%s Changed to STEADY",  __FUNCTION__);
    xTaskCreate(led_steady_task, "Steady LED Task", 2048, NULL, tskIDLE_PRIORITY, &lighting.indicator.task_handle);
    break;

  case INDICATOR_PULSE:
    ESP_LOGD(TAG, "%s Changed to PULSE",  __FUNCTION__);
    xTaskCreate(led_pulse_task, "Fade LED Task", 2048, NULL, tskIDLE_PRIORITY, &lighting.indicator.task_handle);
    break;

  default:
    ESP_LOGW(TAG, "Unknown mode(%u)", mode);
    break;
  }
  //}
}

void lighting_set_indicator_brightness(float brightness)
{
  ESP_LOGD(TAG, "%s(%.2f)", __FUNCTION__, brightness);
  if (brightness < 0.0)
  {
    brightness = 0.0;
  }
  else if (brightness > 1.0)
  {
    brightness = 1.0;
  }

  lighting.indicator.brightness = brightness;
}

///////////////////////////////////////////////////////////////////////////////
//LOCAL FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

static void init_headlight_pwm(void)
{
  ledc_channel_config_t pwm_channel = {

    .channel    = LEDC_CHANNEL_3,
    .duty       = 0,
    .gpio_num   = GPIO_HEADLIGHT_PWM,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .hpoint     = 0,
    .timer_sel  = LEDC_TIMER_1
  };
  ledc_channel_config(&pwm_channel);
}

static void init_indicator_pwm(void)
{
  const ledc_channel_config_t channels[] = {
    {

      .channel    = INDICATOR_R_CHANNEL,
      .duty       = PWM_MAX_VALUE,
      .gpio_num   = GPIO_INDICATOR_R_PWM,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .hpoint     = 0,
      .timer_sel  = LEDC_TIMER_1
    },
    {
      .channel    = INDICATOR_G_CHANNEL,
      .duty       = PWM_MAX_VALUE,
      .gpio_num   = GPIO_INDICATOR_G_PWM,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .hpoint     = 0,
      .timer_sel  = LEDC_TIMER_1
    },
    {
      .channel    = INDICATOR_B_CHANNEL,
      .duty       = PWM_MAX_VALUE,
      .gpio_num   = GPIO_INDICATOR_B_PWM,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .hpoint     = 0,
      .timer_sel  = LEDC_TIMER_1
    }
  };

  for (size_t i = 0; i < ElementsIn(channels); ++i)
  {
    ledc_channel_config(&channels[i]);
  }
}

static uint32_t brightness_to_duty(float brightness)
{
  return brightness * PWM_MAX_VALUE;
}

static void fade_headlights(float brightness)
{
  uint32_t duty = brightness_to_duty(brightness);

  ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, HEADLIGHTS_CHANNEL, duty, lighting.headlights.fade_time_ms);
  ledc_fade_start(LEDC_LOW_SPEED_MODE, HEADLIGHTS_CHANNEL, LEDC_FADE_NO_WAIT);
}

static void fade_led(ledc_channel_t channel, float brightness, uint32_t fade_time_ms, ledc_fade_mode_t mode)
{
  const uint32_t duty = brightness_to_duty(brightness);

  ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, channel, duty, fade_time_ms);
  ledc_fade_start(LEDC_LOW_SPEED_MODE, channel, mode);
}

static void reset_indicator()
{
  const ledc_channel_t channels[] = {INDICATOR_R_CHANNEL, INDICATOR_G_CHANNEL, INDICATOR_B_CHANNEL};

  for (size_t i = 0; i < ElementsIn(channels); ++i)
  {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channels[i], PWM_MAX_VALUE);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channels[i]);
  }
}

static void set_indicator_color(enum color_t color, float brightness)
{
  assert(brightness >= 0.0);
  assert(brightness <= 1.0);
  const float inverted_brightness = 1.0 - brightness;

  switch (color)
  {
  case INDICATOR_RED:
    fade_led(INDICATOR_R_CHANNEL, inverted_brightness, lighting.indicator.fade_time_ms, LEDC_FADE_WAIT_DONE);
    break;

  case INDICATOR_GREEN:
    fade_led(INDICATOR_G_CHANNEL, inverted_brightness, lighting.indicator.fade_time_ms, LEDC_FADE_WAIT_DONE);
    break;

  case INDICATOR_BLUE:
    fade_led(INDICATOR_B_CHANNEL, inverted_brightness, lighting.indicator.fade_time_ms, LEDC_FADE_WAIT_DONE);
    break;

  case INDICATOR_YELLOW:
    fade_led(INDICATOR_R_CHANNEL, inverted_brightness, lighting.indicator.fade_time_ms, LEDC_FADE_NO_WAIT);
    fade_led(INDICATOR_G_CHANNEL, inverted_brightness, lighting.indicator.fade_time_ms, LEDC_FADE_WAIT_DONE);
    break;

  case INDICATOR_PURPLE:
    fade_led(INDICATOR_R_CHANNEL, inverted_brightness, lighting.indicator.fade_time_ms, LEDC_FADE_NO_WAIT);
    fade_led(INDICATOR_B_CHANNEL, inverted_brightness, lighting.indicator.fade_time_ms, LEDC_FADE_WAIT_DONE);
    break;

  default:
    ESP_LOGW(TAG, "Unknown color: 0x%02X", color);
    break;
  }
}

static void led_steady_task(void *parameter)
{
  ESP_LOGD(TAG, "%s started",  __FUNCTION__);

  reset_indicator();

  set_indicator_color(lighting.indicator.color, lighting.indicator.brightness);
  delete_active_task();
}

static void led_pulse_task(void *parameter)
{
  ESP_LOGD(TAG, "%s started",  __FUNCTION__);

  reset_indicator();

  while (1)
  {
    const enum color_t color = lighting.indicator.color;
    set_indicator_color(color, lighting.indicator.brightness);
    set_indicator_color(color, 0.0);
    vTaskDelay(350 / portTICK_PERIOD_MS);
  }
}

static void delete_active_task(void)
{
  char *task_name_p = pcTaskGetTaskName(lighting.indicator.task_handle);
  ESP_LOGD(TAG, "%s: Delete ""%s""", __FUNCTION__, task_name_p);

  TaskHandle_t task_handle = lighting.indicator.task_handle;
  lighting.indicator.task_handle = NULL;

  vTaskDelete(task_handle);
}

static inline bool has_active_task(void)
{
  return lighting.indicator.task_handle != NULL;
}
