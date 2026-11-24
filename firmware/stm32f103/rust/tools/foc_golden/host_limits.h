/* Host shim for the Simulink-generated controller.
 *
 * The generated source rejects LP64 hosts because its `#if` checks assume a
 * 32-bit `long`. Supplying the expected limits before the source includes
 * <limits.h> keeps the check satisfied without modifying the pinned reference.
 */
#ifndef FOC_GOLDEN_HOST_LIMITS_H
#define FOC_GOLDEN_HOST_LIMITS_H

#include <limits.h>

#undef UCHAR_MAX
#undef SCHAR_MAX
#undef SHRT_MAX
#undef INT_MAX
#undef LONG_MAX
#undef ULONG_MAX

#define UCHAR_MAX 0xFFU
#define SCHAR_MAX 0x7F
#define SHRT_MAX 0x7FFF
#define INT_MAX 0x7FFFFFFF
#define LONG_MAX 0x7FFFFFFFL
#define ULONG_MAX 0xFFFFFFFFUL

#endif
