#ifndef POWERTRAIN_H_
#define POWERTRAIN_H_

#include <stdbool.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
//TYPES
///////////////////////////////////////////////////////////////////////////////

enum powertrain_direction_t {POWERTRAIN_UNDEFINED, POWERTRAIN_FORWARD, POWERTRAIN_REVERSE, POWERTRAIN_MAX};

///////////////////////////////////////////////////////////////////////////////
//FUNCTION PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

/**
 * Initialize all powertrain hardware hardware.
 */
void powertrain_init(void);
void powertrain_set_speed(float speed);

/**
 * Get the current speed.
 *
 * @return Speed [0.0, 1.0]
 */
float powertrain_get_speed(void);

void powertrain_steer(float value);

bool powertrain_is_forward(void);

/**
 * Get the current direction.
 *
 * The powertrain direction is based on the set speed so it's not guaranteed to match with
 * the direction of the vehicle.
 *
 * @return The current direction, POWERTRAIN_UNDEFINED if speed is set to 0.
 */
enum powertrain_direction_t powertrain_get_direction(void);

void powertrain_engage_brakes(void);
void powertrain_disengage_brakes(void);

void powertrain_set_max_speed(float max_speed);
float powertrain_get_max_speed(void);

void powertrain_set_max_steer(float value);
float powertrain_get_max_steer(void);


void powertrain_set_headlight_brightness(float brightness);

#endif
