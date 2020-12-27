#ifndef SPEEDCONTROLLER_H_
#define SPEEDCONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

///////////////////////////////////////////////////////////////////////////////
//TYPES
///////////////////////////////////////////////////////////////////////////////

typedef void (*speedcontroller_callback)(size_t id, float value);

///////////////////////////////////////////////////////////////////////////////
//FUNCTION PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

/**
 * Initialize the speedcontroller.
 */
void speedcontroller_init(void);

/**
 * Register a new speed input to the controller.
 *
 * @param name_p Name of the input, only used for debugging.
 *
 * @return ID of the input.
 */
size_t speedcontroller_register_input(const char *name_p);

/**
 * Register a listener.
 *
 * The listener will be notified when the output changes.
 *
 * @param listener Callback function to the listener.
 */
void speedcontroller_register_listener(speedcontroller_callback listener);

/**
 * Get the speed selected by the controller.
 *
 * @return Speed
 */
float speedcontroller_get_output(void);

/**
 * Set value of the specified input.
 *
 * @param id ID of the input, see 'speedcontroller_register_input()'.
 * @param value New value of the input.
 */
void speedcontroller_input(size_t id, float value);

#endif
