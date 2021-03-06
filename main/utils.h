
#ifndef UTILS_H_
#define UTILS_H_

//////////////////////////////////////////////////////////////////////////
//INCLUDES
//////////////////////////////////////////////////////////////////////////

#include <stddef.h>

//////////////////////////////////////////////////////////////////////////
//DEFINES
//////////////////////////////////////////////////////////////////////////

#define ElementsIn(array) \
    ({ \
        _Static_assert \
        ( \
            ! __builtin_types_compatible_p(__typeof__(array), __typeof__(&array[0])), \
            "ElementsIn: "  # array " is not an array" \
        ); \
        sizeof(array) / sizeof((array)[0]); \
    })

#endif
