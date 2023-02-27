#ifndef _MOCKMISSION_H
#define _MOCKMISSION_H

#include "main.h"
#include "PID.h"
#include "mission.h"
#include "timers.h"

void testglobal(void);

void mockmissionInit(void);

void preparemockmisison(void);

void runmockmission(void);

void CalcPressureAndDepth_mock(void);

#endif