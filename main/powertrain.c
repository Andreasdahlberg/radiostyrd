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
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm.h"
#include "driver/ledc.h"
#include "powertrain.h"

///////////////////////////////////////////////////////////////////////////////
//DEFINES
///////////////////////////////////////////////////////////////////////////////

#define GPIO_PWM0A_OUT 4
#define GPIO_PWM0B_OUT 17
#define GPIO_ENABLE_OUT_1_2 25
#define GPIO_PWM_1 27
#define GPIO_PWM_2 22
#define GPIO_ENABLE_OUT_3_4 23
#define LEDC_LS_CH0_GPIO       (13)

#define PWM_MAX_VALUE 4096
#define ENABLE_DEBUG_LED 1

///////////////////////////////////////////////////////////////////////////////
//FUNCTION PROTOTYPES
///////////////////////////////////////////////////////////////////////////////
static void init_led(void);
static void init_brake_pwm(void);
static void init_mcpwm(void);
static void init_gpio(void);


static void motor_forward(float duty);
static void motor_reverse(float duty);
static void motor_enable(void);
static void motor_disable(void);
static inline void motor_brake(void);
static void disable_front_steering(void);
static void enable_front_steering(void);
static float speed_to_duty(float speed);
static uint32_t steering_to_duty(float value);
static inline bool is_forward(void);
static inline void set_debug_led(float speed);

///////////////////////////////////////////////////////////////////////////////
//TYPES
///////////////////////////////////////////////////////////////////////////////

struct parameter_t {
  float current;
  float max;
  float min;
};

struct powertrain_t {
  struct parameter_t speed;
  struct parameter_t steer;
  bool invert_steering;
  bool braking;
};

///////////////////////////////////////////////////////////////////////////////
//VARIABLES
///////////////////////////////////////////////////////////////////////////////

static const char* TAG = "powertrain";

static struct powertrain_t powertrain_status;

///////////////////////////////////////////////////////////////////////////////
//FUNCTIONs
///////////////////////////////////////////////////////////////////////////////
void powertrain_init(void)
{
  powertrain_status = (typeof(powertrain_status)) {0};
  powertrain_status.speed.max = 75.0;
  powertrain_status.speed.min = 35.0;
  powertrain_status.steer.max = 0.7;
  powertrain_status.steer.min = 0.0;

  init_led();
  init_gpio();
  init_mcpwm();
  init_brake_pwm();

  ESP_LOGI(TAG, "Initialized");
}

enum powertrain_direction_t powertrain_get_direction(void)
{
  if (powertrain_status.speed.current > 0)
  {
    return POWERTRAIN_FORWARD;
  }
  else if (powertrain_status.speed.current < 0)
  {
    return POWERTRAIN_REVERSE;
  }
  else
  {
    return POWERTRAIN_UNDEFINED;
  }
}

void powertrain_steer(float value)
{
  powertrain_status.steer.current = value;
  if (powertrain_status.braking)
  {
    return;
  }

  enum powertrain_direction_t direction = powertrain_get_direction();
  if (direction == POWERTRAIN_REVERSE)
  {
    powertrain_status.invert_steering = true;
  }
  else if (direction == POWERTRAIN_FORWARD)
  {
    powertrain_status.invert_steering = false;
  }

  bool isLeft = (bool)(value > 0);
  if (powertrain_status.invert_steering)
  {
    isLeft = !isLeft;
  }

  if (value)
  {
    const uint32_t duty = steering_to_duty(value);

    ESP_LOGV(TAG, "%s(%.2f) %u Duty value: %u", __FUNCTION__, value, (uint32_t)powertrain_status.invert_steering, duty);

    if (isLeft)
    {
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
    else
    {
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    }
    enable_front_steering();
  }
  else
  {
    disable_front_steering();
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
  }
}

void powertrain_set_speed(float speed)
{
  powertrain_status.speed.current = speed;
  if (!powertrain_status.braking)
  {
    // Make sure that the car changes steer direction if the moving direction is changed.
    powertrain_steer(powertrain_status.steer.current);

    if (speed)
    {
      if (is_forward())
      {
        motor_forward(speed_to_duty(speed));
      }
      else
      {
        motor_reverse(speed_to_duty(speed));
      }
      motor_enable();
    }
    else
    {
      motor_disable();
    }

    set_debug_led(speed);
  }
}

float powertrain_get_speed(void)
{
  return powertrain_status.speed.current;
}

void powertrain_engage_brakes(void)
{
  char *task_name_p = pcTaskGetTaskName(NULL);
  ESP_LOGI(TAG, "%s() from %s", __FUNCTION__, task_name_p);

  powertrain_status.braking = true;

  // Rear brakes
  motor_brake();

  // Front brakes
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, PWM_MAX_VALUE);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, PWM_MAX_VALUE);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
  enable_front_steering();
}

void powertrain_disengage_brakes(void)
{
  char *task_name_p = pcTaskGetTaskName(NULL);
  ESP_LOGI(TAG, "%s() from %s", __FUNCTION__, task_name_p);

  powertrain_status.braking = false;
  powertrain_set_speed(powertrain_status.speed.current);
  powertrain_steer(powertrain_status.steer.current);
}

void powertrain_set_max_speed(float speed)
{
  ESP_LOGI(TAG, "%s(%.2f)", __FUNCTION__, speed);
  const float max_limit = 100.0;

  if (speed > max_limit)
  {
    speed = max_limit;
  }

  if (speed < powertrain_status.speed.min)
  {
    speed = powertrain_status.speed.min;
  }

  powertrain_status.speed.max = speed;

  // Set the current speed again so that the new max limit is applied.
  powertrain_set_speed(powertrain_status.speed.current);
}

float powertrain_get_max_speed(void)
{
  return powertrain_status.speed.max;
}

void powertrain_set_max_steer(float value)
{
  ESP_LOGI(TAG, "%s(%.2f)", __FUNCTION__, value);
  const float max_limit = 1.0;

  if (value > max_limit)
  {
    value = max_limit;
  }

  if (value < powertrain_status.steer.min)
  {
    value = powertrain_status.steer.min;
  }

  powertrain_status.steer.max = value;

  // Set the current steer value again so that the new max limit is applied.
  powertrain_steer(powertrain_status.steer.current);

}

float powertrain_get_max_steer(void)
{
  return powertrain_status.steer.max;
}

///////////////////////////////////////////////////////////////////////////////
//LOCAL FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

static void init_led(void)
{
  ledc_timer_config_t ledc_timer = {
    .duty_resolution = LEDC_TIMER_12_BIT, // resolution of PWM duty
    .freq_hz = 15000,                      // frequency of PWM signal
    .speed_mode = LEDC_LOW_SPEED_MODE,      // timer mode
    .timer_num = LEDC_TIMER_1,            // timer index
    .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {

    .channel    = LEDC_CHANNEL_0,
    .duty       = 0,
    .gpio_num   = LEDC_LS_CH0_GPIO,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .hpoint     = 0,
    .timer_sel  = LEDC_TIMER_1
  };
  ledc_channel_config(&ledc_channel);
}


static void init_brake_pwm(void)
{
  ledc_channel_config_t pwm_1_channel = {

    .channel    = LEDC_CHANNEL_1,
    .duty       = 0,
    .gpio_num   = GPIO_PWM_1,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .hpoint     = 0,
    .timer_sel  = LEDC_TIMER_1
  };
  ledc_channel_config(&pwm_1_channel);


  ledc_channel_config_t pwm_2_channel = {

    .channel    = LEDC_CHANNEL_2,
    .duty       = 0,
    .gpio_num   = GPIO_PWM_2,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .hpoint     = 0,
    .timer_sel  = LEDC_TIMER_1
  };
  ledc_channel_config(&pwm_2_channel);
}

static void init_mcpwm(void)
{
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);

  const mcpwm_config_t pwm_config = {
    .frequency = 20000,
    .cmpr_a = 0.0,
    .cmpr_b = 0.0,
    .counter_mode = MCPWM_UP_COUNTER,
    .duty_mode = MCPWM_DUTY_MODE_0
  };

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

static void init_gpio(void)
{
  const uint64_t pin_mask = (1 << GPIO_ENABLE_OUT_1_2) |
                            (1 << GPIO_ENABLE_OUT_3_4) |
                            (1 << GPIO_PWM_1) |
                            (1 << GPIO_PWM_2);

  const gpio_config_t config = {
    .pin_bit_mask = pin_mask,
    .mode = GPIO_MODE_OUTPUT,
    .intr_type = GPIO_INTR_DISABLE
  };

  gpio_config(&config);
  gpio_set_level(GPIO_ENABLE_OUT_1_2, 0);
  gpio_set_level(GPIO_ENABLE_OUT_3_4, 0);
}

static void motor_forward(float duty)
{
  char *task_name_p = pcTaskGetTaskName(NULL);
  ESP_LOGV(TAG, "%s(%.2f) from %s", __FUNCTION__, duty, task_name_p);

  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, duty);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
}

static void motor_reverse(float duty)
{
  char *task_name_p = pcTaskGetTaskName(NULL);
  ESP_LOGV(TAG, "%s(%.2f) from %s", __FUNCTION__, duty, task_name_p);

  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, duty);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, MCPWM_DUTY_MODE_0);
}

static inline void motor_enable(void)
{
  gpio_set_level(GPIO_ENABLE_OUT_1_2, 1);
}

static inline void motor_disable(void)
{
  gpio_set_level(GPIO_ENABLE_OUT_1_2, 0);

  // TODO: Is this needed?
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B);
}

static inline void motor_brake(void)
{
  mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);
  mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B);
  motor_enable();
}

static inline void disable_front_steering(void)
{
  gpio_set_level(GPIO_ENABLE_OUT_3_4, 0);
}

static inline void enable_front_steering(void)
{
  gpio_set_level(GPIO_ENABLE_OUT_3_4, 1);
}

static float speed_to_duty(float speed)
{
  float result = 0.0;

  if (speed)
  {
    result = powertrain_status.speed.min + fabsf(speed) * (powertrain_status.speed.max - powertrain_status.speed.min);
  }

  return result;
}

static uint32_t steering_to_duty(float value)
{
  float limited_value = powertrain_status.steer.min + fabsf(value) * (powertrain_status.steer.max - powertrain_status.steer.min);

  return limited_value * PWM_MAX_VALUE;
}

static inline bool is_forward(void)
{
  /*TODO: Fix this, use powertrain_get_direction() */
  return powertrain_status.speed.current >= 0;
}

static inline void set_debug_led(float speed)
{
#if ENABLE_DEBUG_LED
    const uint32_t duty = fabsf(speed) * PWM_MAX_VALUE;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
#endif
}
