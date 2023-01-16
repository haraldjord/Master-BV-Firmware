#ifndef _TICK_H
#define _TICK_H

#include "nrfx_systick.h"

nrfx_systick_state_t ticktime;

#define tick_get() nrfx_systick_get(&ticktime)

#define  

#endif