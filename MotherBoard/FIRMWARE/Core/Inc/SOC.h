/*
 * SOC.h
 *
 *  Created on: Apr 24, 2025
 *      Author: Jacob
 */

#ifndef INC_SOC_H_
#define INC_SOC_H_

#include <stdint.h>

typedef struct {
    uint32_t voltage;  // Battery voltage in millivolts
    uint8_t soc;       // State of Charge percentage (0-100%)
} BatterySOCEntry;

// Declare the lookup table (defined in the .c file)
extern const BatterySOCEntry batterySOCTable[];

// Number of entries in the SOC lookup table
#define SOC_TABLE_SIZE (sizeof(batterySOCTable) / sizeof(BatterySOCEntry))

// Function to get estimated SOC from battery voltage (in millivolts)
uint8_t getBatterySOC(uint32_t voltage_mv);

#endif /* INC_SOC_H_*/
