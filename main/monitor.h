#ifndef MONITOR_H_
#define MONITOR_H_

#include <stdint.h>
#include <stdbool.h>

///////////////////////////////////////////////////////////////////////////////
//TYPES
///////////////////////////////////////////////////////////////////////////////

struct pid_parameters_t
{
    float kp;
    float ki;
    float kd;
    uint32_t sp;
    int32_t imax;
};

///////////////////////////////////////////////////////////////////////////////
//FUNCTION PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

/**
 * Initialize the monitor module.
 */
void monitor_init(void);

/**
 * Get the battery voltage.
 *
 * @return  Battery voltage in millivolt.
 */
uint32_t monitor_get_battery_voltage(void);

/**
 * Get the motor current.
 *
 * @return Motor current in milliampere.
 */
uint32_t monitor_get_motor_current(void);


float monitor_get_suggested_duty(void);


void monitor_set_pid_parameters(const struct pid_parameters_t *parameters);

struct pid_parameters_t monitor_get_pid_parameters(void);

bool monitor_is_motor_stalled(void);

#endif
