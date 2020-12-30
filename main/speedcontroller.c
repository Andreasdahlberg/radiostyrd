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

#include <assert.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "utils.h"
#include "speedcontroller.h"

///////////////////////////////////////////////////////////////////////////////
//DEFINES
///////////////////////////////////////////////////////////////////////////////

#define MAX_NUMBER_OF_INPUTS 3
#define MAX_NUMBER_OF_LISTENERS 2
#define MAX_INPUT_VALUE 1.0
#define MAX_QUEUE_ITEMS 64
#define CORE_ID 1

///////////////////////////////////////////////////////////////////////////////
//TYPES
///////////////////////////////////////////////////////////////////////////////

struct input_t {
  float value;
  const char *name_p;
};

struct input_message_t
{
  size_t id;
  float value;
};

struct speedcontroller_t {
  struct input_t inputs[MAX_NUMBER_OF_INPUTS];
  size_t number_of_inputs;
  size_t active_input_id;
  speedcontroller_callback listeners[MAX_NUMBER_OF_LISTENERS];
  size_t number_of_listeners;
  enum speedcontroller_direction_t direction;
  QueueHandle_t message_queue;
  SemaphoreHandle_t mutex;
};

///////////////////////////////////////////////////////////////////////////////
//FUNCTION PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

static void controller_task(void * parameter);
static void handle_input_message(const struct input_message_t *message_p);
static inline size_t get_active_id(void);
static inline const char* get_active_name(void);
static inline float get_output_value(void);
static void select_active_input(void);
static void notify_listeners(void);

///////////////////////////////////////////////////////////////////////////////
//VARIABLES
///////////////////////////////////////////////////////////////////////////////

static const char* TAG = "speedcontroller";
static struct speedcontroller_t speedcontroller;

///////////////////////////////////////////////////////////////////////////////
//FUNCTIONs
///////////////////////////////////////////////////////////////////////////////

void speedcontroller_init(void)
{
  speedcontroller = (typeof(speedcontroller)) {0};
  speedcontroller.mutex = xSemaphoreCreateMutex();
  speedcontroller.message_queue = xQueueCreate(MAX_QUEUE_ITEMS, sizeof(struct input_message_t));
  assert(speedcontroller.message_queue != NULL);

  TaskHandle_t xHandle = NULL;
  xTaskCreatePinnedToCore(controller_task, "Controller", 2048, NULL, tskIDLE_PRIORITY, &xHandle, CORE_ID);
  assert(xHandle != NULL);

  ESP_LOGI(TAG, "Initialized");
}

size_t speedcontroller_register_input(const char *name_p)
{
  xSemaphoreTake(speedcontroller.mutex, portMAX_DELAY);

  assert(name_p != NULL);
  assert(speedcontroller.number_of_inputs < ElementsIn(speedcontroller.inputs));

  const size_t input_id = speedcontroller.number_of_inputs;
  speedcontroller.inputs[input_id].name_p = name_p;
  speedcontroller.inputs[input_id].value = 0.0;
  ++speedcontroller.number_of_inputs;

  xSemaphoreGive(speedcontroller.mutex);

  ESP_LOGI(TAG, "Registered input '%s' with ID %zu", name_p, input_id);
  return input_id;
}

void speedcontroller_register_listener(speedcontroller_callback listener)
{
  xSemaphoreTake(speedcontroller.mutex, portMAX_DELAY);

  assert(listener != NULL);
  assert(speedcontroller.number_of_listeners < ElementsIn(speedcontroller.listeners));

  const size_t index = speedcontroller.number_of_listeners;
  speedcontroller.listeners[index] = listener;
  ++speedcontroller.number_of_listeners;

  xSemaphoreGive(speedcontroller.mutex);

  ESP_LOGI(TAG, "Registered listener with callback 0x%x", (uint32_t)listener);
}

void speedcontroller_input(size_t id, float value)
{
  assert(id < ElementsIn(speedcontroller.inputs));

  const struct input_message_t message = {.id = id, .value = value};

  if (xQueueSendToBack(speedcontroller.message_queue, &message, 100 / portTICK_PERIOD_MS) != pdTRUE)
  {
    ESP_LOGE(TAG, "Failed to set input '%zu' to '%f'", id, value);
  }
}

void speedcontroller_set_direction(enum speedcontroller_direction_t direction)
{
  assert(direction < SPEEDCONTROLLER_MAX);
  speedcontroller.direction = direction;
}

enum speedcontroller_direction_t speedcontroller_get_direction(void)
{
  return speedcontroller.direction;
}

const char *speedcontroller_get_name(size_t id)
{
  assert(id < ElementsIn(speedcontroller.inputs));

  return speedcontroller.inputs[id].name_p;
}

///////////////////////////////////////////////////////////////////////////////
//LOCAL FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

static void controller_task(void * parameter)
{
  while (1)
  {
    struct input_message_t message;
    xQueueReceive(speedcontroller.message_queue, &message, portMAX_DELAY);

    ESP_LOGV(TAG, "New input: %s(%zu) -> %f", speedcontroller_get_name(message.id), message.id, message.value);
    handle_input_message(&message);
  }
}

static void handle_input_message(const struct input_message_t *message_p)
{
  /* Sign can be removed since the direction is set separately. */
  speedcontroller.inputs[message_p->id].value = fabsf(message_p->value);
  select_active_input();
  notify_listeners();
}

static inline size_t get_active_id(void)
{
  return speedcontroller.active_input_id;
}

static inline const char* get_active_name(void)
{
  return speedcontroller.inputs[get_active_id()].name_p;
}

static inline float get_output_value(void)
{
  return speedcontroller.inputs[get_active_id()].value;
}

static void select_active_input(void)
{
  float value = MAX_INPUT_VALUE;
  size_t active_input_id = 0;

  for (size_t i = 0; i < speedcontroller.number_of_inputs; ++i)
  {
    if (speedcontroller.inputs[i].value < value)
    {
      value = speedcontroller.inputs[i].value;
      active_input_id = i;
    }
  }

  speedcontroller.active_input_id = active_input_id;

  ESP_LOGV(TAG, "Selected input '%s' with ID %zu", get_active_name(), get_active_id());
}

static void notify_listeners(void)
{
  for (size_t i = 0; i < speedcontroller.number_of_listeners; ++i)
  {
    assert(speedcontroller.listeners[i] != NULL);
    speedcontroller.listeners[i](get_active_id(), get_output_value());
  }
}
