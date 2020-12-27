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

///////////////////////////////////////////////////////////////////////////////
//INCLUDES
///////////////////////////////////////////////////////////////////////////////

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "wifi.h"

///////////////////////////////////////////////////////////////////////////////
//DEFINES
///////////////////////////////////////////////////////////////////////////////

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

///////////////////////////////////////////////////////////////////////////////
//FUNCTION PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void wifi_event_handler(int32_t event_id, void* event_data);
static void ip_event_handler(int32_t event_id, void* event_data);

///////////////////////////////////////////////////////////////////////////////
//TYPES
///////////////////////////////////////////////////////////////////////////////

struct wifi_t {
  EventGroupHandle_t event_group;
  uint32_t connection_retry_counter;
  bool connected;
};

///////////////////////////////////////////////////////////////////////////////
//VARIABLES
///////////////////////////////////////////////////////////////////////////////

static const char* TAG = "wifi";
static struct wifi_t wifi;

///////////////////////////////////////////////////////////////////////////////
//FUNCTIONs
///////////////////////////////////////////////////////////////////////////////

void wifi_init(void)
{
  wifi = (typeof(wifi)) {0};
  wifi.event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

  wifi_config_t wifi_config = {
    .sta = {
      .ssid = CONFIG_ESP_WIFI_SSID,
      .password = CONFIG_ESP_WIFI_PASSWORD,
      .threshold.authmode = WIFI_AUTH_WPA2_PSK,
      .pmf_cfg = {
        .capable = true,
        .required = false
      },
    },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
  ESP_ERROR_CHECK(esp_wifi_start() );

  ESP_LOGI(TAG, "wifi_init_sta finished.");

  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
   * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
  EventBits_t bits = xEventGroupWaitBits(wifi.event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE,
                                         pdFALSE,
                                         portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
   * happened. */
  if (bits & WIFI_CONNECTED_BIT)
  {
    ESP_LOGI(TAG, "Connected to SSID:%s", CONFIG_ESP_WIFI_SSID);
    wifi.connected = true;
  } else if (bits & WIFI_FAIL_BIT)
  {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s", CONFIG_ESP_WIFI_SSID);
  } else
  {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }

  ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
  ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
  vEventGroupDelete(wifi.event_group);
}

bool wifi_is_connected(void)
{
  return wifi.connected;
}

///////////////////////////////////////////////////////////////////////////////
//LOCAL FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  if (event_base == WIFI_EVENT)
  {
    wifi_event_handler(event_id, event_data);
  }
  else if (event_base == IP_EVENT) {
    ip_event_handler(event_id, event_data);
  }
  else
  {
    ESP_LOGW(TAG, "Unknown event base: 0x%x", (uint32_t)event_base);
  }
}

static void wifi_event_handler(int32_t event_id, void* event_data)
{
  switch (event_id)
  {
  case WIFI_EVENT_STA_START:
    esp_wifi_connect();
    break;

  case WIFI_EVENT_STA_DISCONNECTED:
    if (wifi.connection_retry_counter < CONFIG_ESP_MAXIMUM_RETRY)
    {
      ESP_LOGI(TAG, "Retry to connect to the AP");
      esp_wifi_connect();
      ++wifi.connection_retry_counter;
    }
    else
    {
      ESP_LOGW(TAG, "Connecting to the AP failed");
      xEventGroupSetBits(wifi.event_group, WIFI_FAIL_BIT);
    };
    break;

  default:
    ESP_LOGW(TAG, "Event not handled: %i", event_id);
    break;
  }
}

static void ip_event_handler(int32_t event_id, void* event_data)
{
  ip_event_got_ip_t* event_p = NULL;

  switch (event_id)
  {
  case IP_EVENT_STA_GOT_IP:
    event_p = (ip_event_got_ip_t*) event_data;
    ESP_LOGI(TAG, "Got ip:" IPSTR, IP2STR(&event_p->ip_info.ip));
    wifi.connection_retry_counter = 0;
    xEventGroupSetBits(wifi.event_group, WIFI_CONNECTED_BIT);
    break;

  default:
    ESP_LOGW(TAG, "Event not handled: %i", event_id);
    break;
  }
}
