#ifndef DEBUGSERVER_H_
#define DEBUGSERVER_H_

///////////////////////////////////////////////////////////////////////////////
//INCLUDES
///////////////////////////////////////////////////////////////////////////////

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

///////////////////////////////////////////////////////////////////////////////
//FUNCTION PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

/**
 * Initialize the debug server.
 */
void debugserver_init(void);

/**
 * Start the debug server.
 */
void debugserver_start(void);

void debugserver_send(char *text_p, size_t size);

void debugserver_redirect_log(void);

void debugserver_restore_log(void);

bool debugserver_is_redirected(void);

#endif
