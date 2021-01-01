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

#if 0
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#endif

///////////////////////////////////////////////////////////////////////////////
//INCLUDES
///////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "driver/mcpwm.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "nvs_flash.h"
#include "ps3.h"

#include "wifi.h"
#include "debugserver.h"
#include "powertrain.h"
#include "monitor.h"
#include "lighting.h"
#include "speedcontroller.h"

///////////////////////////////////////////////////////////////////////////////
//DEFINES
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//TYPES
///////////////////////////////////////////////////////////////////////////////

enum config_mode_t {CONFIG_DISABLED, CONFIG_SPEED_STEERING, CONFIG_CONTROLLER, CONFIG_PID_1, CONFIG_PID_2, CONFIG_MAX};

struct module_t
{
  enum config_mode_t config_mode;
  bool limit_current;
  bool notify_stall;
  size_t input_id;
};

struct controller_t
{
  struct
  {
    int8_t left;
    int8_t right;
  } deadzone;

  int8_t prev_left_x_value;
  int8_t prev_right_y_value;

  enum ps3_status_battery battery_status;
  size_t input_id;
};

///////////////////////////////////////////////////////////////////////////////
//FUNCTION PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

static void controller_init(void);
static void controller_set_left_deadzone(int8_t);
static int8_t controller_get_left_deadzone(void);
static void controller_set_right_deadzone(int8_t);
static int8_t controller_get_right_deadzone(void);

static void handle_speedcontroller_callback(size_t id, float value);
static float value_to_percent(int8_t value, int8_t min);
static void adjust_pid_p(float modifier_value);
static void adjust_pid_i(float modifier_value);
static void adjust_pid_d(float modifier_value);
static void adjust_pid_sp(int32_t modifier_value);
static void adjust_max_speed(float modifier_value);
static void adjust_max_steering(float modifier_value);
static void adjust_left_deadzone(int8_t value);
static void adjust_right_deadzone(int8_t value);
static void handle_button_down_up(void);
static void handle_button_down_down(void);
static void handle_button_down_right(void);
static void handle_button_down_left(void);
static void handle_button_down_select(void);
static void start_wifi_debugging(void);
static void handle_button_down_cross(void);
static void handle_button_down_circle(void);
static const char* get_battery_status_text(enum ps3_status_battery status);
static void controller_event_cb(ps3_t ps3, ps3_event_t event);
static void check_battery_voltage(void);
static void init_nvs(void);
static bool is_same_sign(int8_t a, int8_t b);

///////////////////////////////////////////////////////////////////////////////
//VARIABLES
///////////////////////////////////////////////////////////////////////////////

static const char* TAG = "main";
static struct module_t module;
static struct controller_t controller;

///////////////////////////////////////////////////////////////////////////////
//FUNCTIONs
///////////////////////////////////////////////////////////////////////////////

void app_main(void)
{
  module = (typeof(module)) {
    .config_mode = CONFIG_DISABLED,
     .limit_current = true,
      .notify_stall = true
  };

  controller = (typeof(controller)) {
    .deadzone = {.left = 10, .right = 14},
  };

  /* Initialize NVS since it is used to store PHY calibration data */
  init_nvs();

  debugserver_init();
  powertrain_init();
  lighting_init();
  speedcontroller_init();
  monitor_init();
  controller_init();

  module.input_id = speedcontroller_register_input("Current limit");
  speedcontroller_register_listener(handle_speedcontroller_callback);

  ESP_LOGI(TAG, "Waiting for controller connection...");
  lighting_set_indicator_color(INDICATOR_BLUE);
  lighting_set_indicator_mode(INDICATOR_PULSE);
  while (!ps3IsConnected()) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  lighting_set_indicator_mode(INDICATOR_OFF);
  ps3SetRumble(10, 150, 10, 150);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  ps3SetLed(1);

  uint32_t counter = 0;
  while (1)
  {
    vTaskDelay(10 / portTICK_PERIOD_MS);

    if (module.notify_stall && monitor_is_motor_stalled())
    {
      if (!((counter) % 50))
      {

        ESP_LOGW(TAG, "Motor stalled(%u mA)", monitor_get_motor_current());
        ps3SetRumble(10, 100, 10, 100);

      }
      ++counter;
    }
    else
    {
      counter = 0;
    }

    if (module.limit_current)
    {
      float suggested_speed = monitor_get_suggested_duty() / 100;
      speedcontroller_input(module.input_id, suggested_speed);

      ESP_LOGV(TAG, "%u", monitor_get_motor_current());
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
//FUNCTIONs
///////////////////////////////////////////////////////////////////////////////

static void controller_init(void)
{
  const uint8_t mac_address[8] = {0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57};
  ps3SetBluetoothMacAddress(mac_address);
  ps3SetEventCallback(controller_event_cb);
  ps3Init();

  controller.input_id = speedcontroller_register_input("PS3 Controller");
}

static void controller_set_left_deadzone(int8_t value)
{
  if (value < 0)
  {
    value = 0;
  }

  ESP_LOGI(TAG, "%s(%i)", __FUNCTION__, value);
  controller.deadzone.left = value;
}

static int8_t controller_get_left_deadzone(void)
{
  return controller.deadzone.left;
}

static void controller_set_right_deadzone(int8_t value)
{
  if (value < 0)
  {
    value = 0;
  }

  ESP_LOGI(TAG, "%s(%i)", __FUNCTION__, value);
  controller.deadzone.right = value;
}

static int8_t controller_get_right_deadzone(void)
{
  return controller.deadzone.right;
}

static void handle_speedcontroller_callback(size_t id, float value)
{
  static float prev_value = 100.0;

  if (prev_value != value)
  {
    prev_value = value;

    const enum speedcontroller_direction_t direction = speedcontroller_get_direction();
    const float speed = (direction == SPEEDCONTROLLER_FORWARD) ? value : value * -1.0;
    powertrain_set_speed(speed);
  }
}

static float value_to_percent(int8_t value, int8_t min)
{
  value += value > 0 ? -1 * min : min;
  int8_t max = value > 0 ? INT8_MAX - min : abs(INT8_MIN) - min;
  return -1 * ((float)value / (float)max);
}

static void adjust_pid_p(float modifier_value)
{
  struct pid_parameters_t parameters = monitor_get_pid_parameters();
  parameters.kp += modifier_value;
  if (parameters.kp < 0.0)
  {
    parameters.kp = 0.0;
  }

  monitor_set_pid_parameters(&parameters);
}

static void adjust_pid_i(float modifier_value)
{
  struct pid_parameters_t parameters = monitor_get_pid_parameters();
  parameters.ki += modifier_value;
  if (parameters.ki < 0.0)
  {
    parameters.ki = 0.0;
  }

  monitor_set_pid_parameters(&parameters);
}

static void adjust_pid_d(float modifier_value)
{
  struct pid_parameters_t parameters = monitor_get_pid_parameters();
  parameters.kd += modifier_value;
  if (parameters.kd < 0.0)
  {
    parameters.kd = 0.0;
  }

  monitor_set_pid_parameters(&parameters);
}

static void adjust_pid_sp(int32_t modifier_value)
{
  struct pid_parameters_t parameters = monitor_get_pid_parameters();
  parameters.sp += modifier_value;
  monitor_set_pid_parameters(&parameters);
}

static void adjust_max_speed(float modifier_value)
{
  float max_speed = powertrain_get_max_speed();
  powertrain_set_max_speed(max_speed + modifier_value);
}

static void adjust_max_steering(float modifier_value)
{
  float max_steer = powertrain_get_max_steer();
  powertrain_set_max_steer(max_steer + (modifier_value / 100));
}

static void adjust_left_deadzone(int8_t value)
{
  int8_t deadzone = controller_get_left_deadzone();
  controller_set_left_deadzone(deadzone + value);
}

static void adjust_right_deadzone(int8_t value)
{
  int8_t deadzone = controller_get_right_deadzone();
  controller_set_right_deadzone(deadzone + value);
}

static void handle_button_down_up(void)
{
  switch (module.config_mode)
  {
  case CONFIG_DISABLED:
    ESP_LOGI(TAG, "Configuration disabled");
    break;

  case CONFIG_SPEED_STEERING:
    adjust_max_speed(5.0);
    break;

  case CONFIG_CONTROLLER:
    adjust_left_deadzone(1);
    break;

  case CONFIG_PID_1:
    adjust_pid_p(0.01);
    break;

  case CONFIG_PID_2:
    adjust_pid_d(0.01);
    break;

  default:
    ESP_LOGW(TAG, "Unknown config mode(%u)", (uint32_t)module.config_mode);
    break;
  }
}

static void handle_button_down_down(void)
{
  switch (module.config_mode)
  {
  case CONFIG_DISABLED:
    ESP_LOGI(TAG, "Configuration disabled");
    break;

  case CONFIG_SPEED_STEERING:
    adjust_max_speed(-5.0);
    break;

  case CONFIG_CONTROLLER:
    adjust_left_deadzone(-1);
    break;

  case CONFIG_PID_1:
    adjust_pid_p(-0.01);
    break;

  case CONFIG_PID_2:
    adjust_pid_d(-0.01);
    break;

  default:
    ESP_LOGW(TAG, "Unknown config mode(%u)", (uint32_t)module.config_mode);
    break;
  }
}

static void handle_button_down_right(void)
{
  switch (module.config_mode)
  {
  case CONFIG_DISABLED:
    ESP_LOGI(TAG, "Configuration disabled");
    break;

  case CONFIG_SPEED_STEERING:
    adjust_max_steering(5.0);
    break;

  case CONFIG_CONTROLLER:
    adjust_right_deadzone(1);
    break;

  case CONFIG_PID_1:
    adjust_pid_i(0.01);
    break;

  case CONFIG_PID_2:
    adjust_pid_sp(50);
    break;

  default:
    ESP_LOGW(TAG, "Unknown config mode(%u)", (uint32_t)module.config_mode);
    break;
  }
}

static void handle_button_down_left(void)
{
  switch (module.config_mode)
  {
  case CONFIG_DISABLED:
    ESP_LOGI(TAG, "Configuration disabled");
    break;

  case CONFIG_SPEED_STEERING:
    adjust_max_steering(-5.0);
    break;

  case CONFIG_CONTROLLER:
    adjust_right_deadzone(-1);
    break;

  case CONFIG_PID_1:
    adjust_pid_i(-0.01);
    break;

  case CONFIG_PID_2:
    adjust_pid_sp(-50);
    break;

  default:
    ESP_LOGW(TAG, "Unknown config mode(%u)", (uint32_t)module.config_mode);
    break;
  }
}

static void handle_button_down_select(void)
{
  module.config_mode += 1;
  if (module.config_mode >= CONFIG_MAX)
  {
    module.config_mode = 0;
  }

  switch (module.config_mode)
  {
  case CONFIG_DISABLED:
    ESP_LOGI(TAG, "Disabled configuration");
    if (!module.limit_current)
    {
      lighting_set_indicator_color(INDICATOR_RED);
      lighting_set_indicator_mode(INDICATOR_STEADY);
    }
    else
    {
      lighting_set_indicator_mode(INDICATOR_OFF);
    }
    break;

  case CONFIG_SPEED_STEERING:
    ESP_LOGI(TAG, "Selected speed/steering adjustment mode");
    lighting_set_indicator_color(INDICATOR_GREEN);
    lighting_set_indicator_mode(INDICATOR_STEADY);
    break;

  case CONFIG_CONTROLLER:
    ESP_LOGI(TAG, "Selected controller configuration mode");
    lighting_set_indicator_color(INDICATOR_YELLOW);
    lighting_set_indicator_mode(INDICATOR_STEADY);
    break;

  case CONFIG_PID_1:
    ESP_LOGI(TAG, "Selected PID 1 tuning mode");
    lighting_set_indicator_color(INDICATOR_BLUE);
    lighting_set_indicator_mode(INDICATOR_STEADY);
    break;

  case CONFIG_PID_2:
    ESP_LOGI(TAG, "Selected PID 2 tuning mode");
    lighting_set_indicator_color(INDICATOR_PURPLE);
    lighting_set_indicator_mode(INDICATOR_STEADY);
    break;

  default:
    ESP_LOGW(TAG, "Selected unknown mode(%u)", (uint32_t)module.config_mode);
    lighting_set_indicator_mode(INDICATOR_OFF);
    break;
  }
}

static void start_wifi_debugging(void)
{
  lighting_set_indicator_color(INDICATOR_YELLOW);
  lighting_set_indicator_mode(INDICATOR_STEADY);
  if (!wifi_is_connected())
  {
    wifi_init();
  }

  if (debugserver_is_redirected())
  {
    debugserver_restore_log();
  }
  else
  {
    debugserver_redirect_log();
  }
  debugserver_start();

  lighting_set_indicator_mode(INDICATOR_OFF);
}

static void handle_button_down_cross(void)
{
  if (module.config_mode == CONFIG_PID_1 || module.config_mode == CONFIG_PID_2)
  {
    module.limit_current = !module.limit_current;

    if (!module.limit_current)
    {
      ESP_LOGI(TAG, "Disabled motor current limit");
    }
    else
    {
      ESP_LOGI(TAG, "Enabled motor current limit");
    }
  }
  else
  {
    ESP_LOGI(TAG, "Motor current limit can only be changed in PID1/2 tuning mode.");
  }
}

static void handle_button_down_circle(void)
{
  if (module.config_mode != CONFIG_DISABLED)
  {
    module.notify_stall = !module.notify_stall;

    if (!module.notify_stall)
    {
      ESP_LOGI(TAG, "Disabled motor stall notification");
    }
    else
    {
      ESP_LOGI(TAG, "Enabled motor stall notification");
    }
  }
  else
  {
    ESP_LOGI(TAG, "Motor stall notification can't be changed when configuration is disabled.");
  }
}

static const char* get_battery_status_text(enum ps3_status_battery status)
{
  switch (status)
  {
  case ps3_status_battery_shutdown:
    return "shutdown";

  case ps3_status_battery_dying:
    return "dying";

  case ps3_status_battery_low:
    return "low";

  case ps3_status_battery_high:
    return "high";

  case ps3_status_battery_full:
    return "full";

  case ps3_status_battery_charging:
    return "charging";

  default:
    return "unknown";
  }
}

static void controller_event_cb(ps3_t ps3, ps3_event_t event)
{
  if (ps3.status.battery != controller.battery_status)
  {
    controller.battery_status = ps3.status.battery;
    const char *status_text_p = get_battery_status_text(controller.battery_status);
    ESP_LOGI(TAG, "Controller battery is %s", status_text_p);
  }

  if (event.button_down.triangle)
  {
    lighting_headlight_toggle();
  }

  if (event.button_down.circle)
  {
    handle_button_down_circle();
  }

  if (event.button_down.square)
  {
    lighting_headlight_toggle_high_low_beam();
  }

  if (event.button_down.cross)
  {
    handle_button_down_cross();
  }

  if (event.button_down.r1)
  {
    ps3SetRumble(8, 100, 8, 100);
    powertrain_engage_brakes();
  }
  else if (event.button_up.r1)
  {
    powertrain_disengage_brakes();
  }

  if (event.button_down.up)
  {
    handle_button_down_up();
  }

  if (event.button_down.down)
  {
    handle_button_down_down();
  }

  if (event.button_down.right)
  {
    handle_button_down_right();
  }

  if (event.button_down.left)
  {
    handle_button_down_left();
  }

  if (event.analog_changed.stick.lx)
  {
    const int8_t left_x_value = ps3.analog.stick.lx;

    /* Skip the first value after a direction change to reduce the risk of joystick backlash. */
    if (left_x_value == 0 || is_same_sign(controller.prev_left_x_value, left_x_value))
    {
      const int8_t deadzone = controller_get_left_deadzone();

      if (abs(left_x_value) > deadzone)
      {
        float percent = value_to_percent(left_x_value, deadzone);
        powertrain_steer(percent);
      }
      else
      {
        powertrain_steer(0);
      }
    }
    controller.prev_left_x_value = left_x_value;
  }

  if (event.analog_changed.stick.ry)
  {
    const int8_t right_y_value = ps3.analog.stick.ry;

    /* Skip the first value after a direction change to reduce the risk of joystick backlash. */
    if (right_y_value == 0 || is_same_sign(controller.prev_right_y_value, right_y_value))
    {
      const int8_t deadzone = controller_get_right_deadzone();

      if (abs(right_y_value) >= deadzone)
      {
        const float percent = value_to_percent(right_y_value, deadzone);
        const enum speedcontroller_direction_t direction = percent >= 0 ? SPEEDCONTROLLER_FORWARD : SPEEDCONTROLLER_REVERSE;

        speedcontroller_set_direction(direction);
        speedcontroller_input(controller.input_id, percent);
      }
      else
      {
        speedcontroller_input(controller.input_id, 0.0);
      }
    }
    controller.prev_right_y_value = right_y_value;
  }

  if (event.button_down.select && ps3.button.start)
  {
    start_wifi_debugging();
  }
  else if (event.button_down.select)
  {
    handle_button_down_select();
  }

  if (ps3.button.start && event.button_down.ps)
  {
    ESP_LOGI(TAG, "Restart...");
    esp_restart();
  }
}

static void check_battery_voltage(void)
{
  uint32_t voltage = monitor_get_battery_voltage();

  printf("VBAT: %u mV\r\n", voltage);
}

static void init_nvs(void)
{
  esp_err_t ret;

  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
}

static bool is_same_sign(int8_t a, int8_t b)
{
  return (a ^ b) >= 0;
}
