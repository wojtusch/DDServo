/*
 ******************************************************************************
 * File    		measurementLookup.h
 * Author  		wojtusch@sim.tu-darmstadt.de
 * Brief   		Header for measurement lookup tables.
 * Peripherals
 ******************************************************************************
 */

#ifndef __MEASUREMENT_LOOKUP_H
#define __MEASUREMENT_LOOKUP_H

// Includes
#include "stm32f10x.h"
#include "fixpoint.h"

// Global variables
extern const fxp16_t voltageScaling[4096];
extern const fxp16_t controllerTemperatureScaling[4096];
extern const fxp16_t motorTemperatureScaling[4096];

#endif
