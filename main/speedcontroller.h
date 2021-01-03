#ifndef SPEEDCONTROLLER_H_
#define SPEEDCONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

///////////////////////////////////////////////////////////////////////////////
//TYPES
///////////////////////////////////////////////////////////////////////////////

enum speedcontroller_message_type_t {SPEEDCONTROLLER_SPEED, SPEEDCONTROLLER_STEERING, SPEEDCONTROLLER_BRAKE};

/**
 * Called when the output is changed.
 *
 * @param type Type of the changed output.
 * @param id ID of the input selected as output.
 * @param value Output value
 */
typedef void (*speedcontroller_callback)( enum speedcontroller_message_type_t type, size_t id, float value);
enum speedcontroller_direction_t {SPEEDCONTROLLER_FORWARD, SPEEDCONTROLLER_REVERSE, SPEEDCONTROLLER_MAX};

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
 * Set value of the specified input.
 *
 * @param id ID of the input, see 'speedcontroller_register_input()'.
 * @param value New value of the input.
 */
void speedcontroller_input(size_t id, float value);

/**
 * Set the steering direction.
 *
 * @param value Negative value for right, positive for left and zero for center. [-1.0, 1.0]
 */
void speedcontroller_steer(float value);

/**
 * Engage/disengage the brakes.
 *
 * Both the front brakes and motor braking is used.
 *
 * @param value Engage/disengage.
 */
void speedcontroller_brake(bool value);

/**
 * Set the direction.
 *
 * @param direction
 */
void speedcontroller_set_direction(enum speedcontroller_direction_t direction);

/**
 * Get the direction.
 *
 * @return The direction, SPEEDCONTROLLER_FORWARD or SPEEDCONTROLLER_REVERSE.
 */
enum speedcontroller_direction_t speedcontroller_get_direction(void);

/**
 * Get the name of the specified input.
 *
 * @param id ID of input.
 *
 * @return Name of input.
 */
const char *speedcontroller_get_name(size_t id);

#endif
