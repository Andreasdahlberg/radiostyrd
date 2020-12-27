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
#include "freertos/ringbuf.h"

#include "esp_log.h"

#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "debugserver.h"

///////////////////////////////////////////////////////////////////////////////
//DEFINES
///////////////////////////////////////////////////////////////////////////////

#define TCP_SOCKET_PORT 3333

///////////////////////////////////////////////////////////////////////////////
//FUNCTION PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

static int log_to_server(const char *format_p, va_list args);
static void server_task(void *pvParameters);

///////////////////////////////////////////////////////////////////////////////
//TYPES
///////////////////////////////////////////////////////////////////////////////

struct debugserver_t
{
  RingbufHandle_t tx_buffer_handle;
  bool initialized;
  bool started;
  bool local_echo;
  vprintf_like_t local_log_func;
};

///////////////////////////////////////////////////////////////////////////////
//VARIABLES
///////////////////////////////////////////////////////////////////////////////

static const char* TAG = "debugserver";
static struct debugserver_t debugserver;

///////////////////////////////////////////////////////////////////////////////
//FUNCTIONs
///////////////////////////////////////////////////////////////////////////////

void debugserver_init(void)
{
  debugserver = (typeof(debugserver)) {0};
  debugserver.tx_buffer_handle = xRingbufferCreate(4096, RINGBUF_TYPE_BYTEBUF);

  debugserver.local_log_func = NULL;

  if (debugserver.tx_buffer_handle == NULL)
  {
    ESP_LOGE(TAG, "Failed to create Tx-buffer");
  }
  else
  {
    debugserver.initialized = true;
    ESP_LOGI(TAG, "Debugserver initialized");
  }
}

void debugserver_start(void)
{
  if (debugserver.initialized && !debugserver.started)
  {
    xTaskCreate(server_task, "debug_server", 4096, NULL, 5, NULL);
    debugserver.started = true;
    ESP_LOGI(TAG, "Debugserver started");
  }
}

void debugserver_send(char *text_p, size_t size)
{
  UBaseType_t result = xRingbufferSend(debugserver.tx_buffer_handle, text_p, size, 0);
  if (result != pdTRUE) {
    printf("%s: Failed to send: %s", TAG, text_p);
  }
}

void debugserver_redirect_log(void)
{
  ESP_LOGI(TAG, "Redirecting log output to debug server");
  debugserver.local_log_func = esp_log_set_vprintf(&log_to_server);
}

void debugserver_restore_log(void)
{
  if (debugserver.local_log_func != NULL)
  {
    ESP_LOGI(TAG, "Redirecting log output to serial");
    esp_log_set_vprintf(debugserver.local_log_func);

    debugserver.local_log_func = NULL;
  }
}

bool debugserver_is_redirected(void)
{
  return debugserver.local_log_func != NULL;
}

///////////////////////////////////////////////////////////////////////////////
//LOCAL FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

static int log_to_server(const char *format_p, va_list args)
{
  char text[128];
  vsnprintf(text, sizeof(text), format_p, args);

  debugserver_send(text, strnlen(text, sizeof(text)));

  if (debugserver.local_echo && debugserver.local_log_func != NULL)
  {
    debugserver.local_log_func(format_p, args);
  }

  return strnlen(text, sizeof(text));
}

static void send_from_buffer(const int sock)
{
  while (1)
  {
    size_t size;
    char *text_p = (char *)xRingbufferReceive(debugserver.tx_buffer_handle, &size, pdMS_TO_TICKS(1000));

    if (text_p != NULL)
    {
      ssize_t to_write = size;
      while (to_write > 0)
      {
        size_t offset = size - to_write;
        ssize_t written = send(sock, text_p + offset, to_write, 0);
        if (written < 0)
        {
          ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
          vRingbufferReturnItem(debugserver.tx_buffer_handle, text_p);
          return;
        }
        to_write -= written;
      }

      vRingbufferReturnItem(debugserver.tx_buffer_handle, text_p);
    }
    //taskYIELD();
  }
}

static void server_task(void *pvParameters)
{
  char addr_str[128];
  int addr_family;
  int ip_protocol;

  struct sockaddr_in dest_addr;
  dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(TCP_SOCKET_PORT);
  addr_family = AF_INET;
  ip_protocol = IPPROTO_IP;
  inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

  int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
  if (listen_sock < 0)
  {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    vTaskDelete(NULL);
    return;
  }
  ESP_LOGI(TAG, "Socket created");

  int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (err != 0)
  {
    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    vTaskDelete(NULL);
    return;
  }
  ESP_LOGI(TAG, "Socket bound, port %d", TCP_SOCKET_PORT);

  err = listen(listen_sock, 1);
  if (err != 0)
  {
    ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
    vTaskDelete(NULL);
    return;
  }

  while (1)
  {

    ESP_LOGI(TAG, "Socket listening");

    struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
    unsigned int addr_len = sizeof(source_addr);
    int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
    if (sock < 0)
    {
      ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
      break;
    }

    // Convert ip address to string
    if (source_addr.sin6_family == PF_INET)
    {
      inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
    } else
    {
      ESP_LOGE(TAG, "IPv6 not supported");
    }
    ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

    send_from_buffer(sock);

    shutdown(sock, 0);
    closesocket(sock);
  }
}
