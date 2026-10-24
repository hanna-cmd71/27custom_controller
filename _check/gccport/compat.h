/*
 * _check/gccport/compat.h
 *
 * Syntax-check-only pre-include (passed with -include by _check/syn.ps1).
 * Not part of the Keil/armcc build.
 *
 * Why it exists: newlib's <sys/cdefs.h> defines
 *     #define __packed __attribute__((__packed__))
 * which collides with this project's ARMCC-style names:
 *     attribute_typedef.h : #define __packed__ __attribute__((packed))
 *     remote_control.h    : } __packed__ rc;
 * so the two expand into a nested __attribute__ and gcc errors out.
 * Pulling cdefs.h in first and re-defining both names keeps gcc happy.
 */
#ifndef _CHECK_GCC_COMPAT_H
#define _CHECK_GCC_COMPAT_H

#include <sys/cdefs.h>

#undef __packed
#undef __packed__

#define __packed    __attribute__((packed))
#define __packed__  __attribute__((packed))

#endif /* _CHECK_GCC_COMPAT_H */
