
#ifndef FILTER_H_
#define FILTER_H_

//////////////////////////////////////////////////////////////////////////
//INCLUDES
//////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

//////////////////////////////////////////////////////////////////////////
//DEFINES
//////////////////////////////////////////////////////////////////////////

#define FILTER_ALPHA(a) ((a) * UINT32_MAX)

//////////////////////////////////////////////////////////////////////////
//TYPE DEFINITIONS
//////////////////////////////////////////////////////////////////////////

struct filter_t
{
    uint32_t value;
    uint32_t alpha;
};

//////////////////////////////////////////////////////////////////////////
//FUNCTION PROTOTYPES
//////////////////////////////////////////////////////////////////////////

/**
 * Initialize the exponential moving average(EMA) filter.
 *
 * @param self_p        Pointer to filter struct.
 * @param initial_value Initial value of the filter, e.g. the first sample.
 * @param alpha         Smoothing factor, a higher value discounts older
 *                      observations faster. Use `FILTER_ALPHA()` to convert
 *                      a floating point value to an integer at compile time.
 */
void Filter_Init(struct filter_t *self_p, uint32_t initial_value, uint32_t alpha);

/**
 * Process the supplied sample.
 *
 * @param self_p Pointer to filter struct.
 * @param sample Sample to process.
 */
void Filter_Process(struct filter_t *self_p, uint32_t sample);

/**
 * Get the filter output value.
 *
 * @param self_p Pointer to filter struct.
 *
 * @return The filter output value.
 */
uint32_t Filter_Output(const struct filter_t *self_p);

/**
 * Check if the supplied filter is initialized.
 *
 * @param self_p Pointer to filter struct.
 *
 * @return True if initialized, otherwise false.
 */
bool Filter_IsInitialized(const struct filter_t *self_p);

#endif
