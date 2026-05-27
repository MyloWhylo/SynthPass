#ifndef _FUNCONFIG_H
#define _FUNCONFIG_H

#define FUNCONF_USE_DEBUGPRINTF     0 // can only have one printf
// Composite CDC+MSD device drives USB via FUSB_USER_HANDLERS (see usb_config.h),
// which is mutually exclusive with USBPRINTF. printf is routed to the CDC
// endpoint through our own _write() in SynthPass.c instead.
#define FUNCONF_USE_USBPRINTF       0
#define FUNCONF_USE_HSI             0 // CH5xx does not have HSI
#define CLK_SOURCE_CH5XX            CLK_SOURCE_PLL_60MHz // default so not really needed
#define FUNCONF_SYSTEM_CORE_CLOCK   60 * 1000 * 1000     // keep in line with CLK_SOURCE_CH5XX
#define FUNCONF_DEBUG_HARDFAULT     0
#define FUNCONF_USE_CLK_SEC         0

#endif