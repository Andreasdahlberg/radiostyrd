#ifndef WIFI_H_
#define WIFI_H_

///////////////////////////////////////////////////////////////////////////////
//INCLUDES
///////////////////////////////////////////////////////////////////////////////

#include <stdbool.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
//FUNCTION PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

/**
 * Initialize WiFi and connect to a predefined AP.
 */
void wifi_init(void);

/**
 * Check if connected to an AP.
 *
 * @return True if connected, otherwise false.
 */
bool wifi_is_connected(void);

#endif
