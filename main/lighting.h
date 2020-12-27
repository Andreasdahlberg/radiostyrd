#ifndef LIGHTING_H_
#define LIGHTING_H_

///////////////////////////////////////////////////////////////////////////////
//INCLUDES
///////////////////////////////////////////////////////////////////////////////

#include <stdbool.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
//TYPES
///////////////////////////////////////////////////////////////////////////////

enum color_t {INDICATOR_RED, INDICATOR_GREEN, INDICATOR_BLUE, INDICATOR_YELLOW, INDICATOR_PURPLE};
enum mode_t {INDICATOR_OFF, INDICATOR_STEADY, INDICATOR_PULSE};

///////////////////////////////////////////////////////////////////////////////
//FUNCTION PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

/**
 * Initialize all lighting hardware.
 */
void lighting_init(void);
void lighting_set_headlight_limits(float low_beam, float high_beam);

void lighting_headlight_on(void);
void lighting_headlight_off(void);
void lighting_headlight_toggle(void);
void lighting_headlight_toggle_high_low_beam(void);
void lighting_headlight_low_beam(void);
void lighting_headlight_high_beam(void);
void lighting_set_headlight_brightness(float brightness);

void lighting_set_indicator_color(enum color_t color);
void lighting_set_indicator_mode(enum mode_t mode);

#endif
