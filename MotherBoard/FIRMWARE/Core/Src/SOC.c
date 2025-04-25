/*
 * SOC.c
 *
 *  Created on: Apr 24, 2025
 *      Author: Jacob
 */

#include <SOC.h>

// Lookup table with voltage to SOC mapping
// Voltages represented in millivolts (e.g., 4200 means 4.2V)
const BatterySOCEntry batterySOCTable[] = {
    {4200, 100}, // Full charge
    {4100, 95},  // Estimated from graph
    {4000, 90},  // Estimated from graph
    {3900, 80},  // ~600mAh discharged
    {3800, 70},  // ~900mAh discharged
    {3700, 60},  // ~1200mAh discharged
    {3600, 50},  // ~1500mAh discharged
    {3500, 40},  // ~1800mAh discharged
    {3400, 30},  // ~2100mAh discharged
    {3300, 20},  // ~2400mAh discharged
    {3200, 15},  // Estimated from graph
    {3000, 10},  // Steep voltage drop begins
    {2800, 5},   // Approaching cutoff
    {2500, 0}    // Discharge cutoff voltage
};

// Function to get estimated SOC from battery voltage (in millivolts)
uint8_t getBatterySOC(uint32_t voltage_mv) {
    // Handle edge cases
    if (voltage_mv >= batterySOCTable[0].voltage) {
        return batterySOCTable[0].soc;
    }
    if (voltage_mv <= batterySOCTable[SOC_TABLE_SIZE-1].voltage) {
        return batterySOCTable[SOC_TABLE_SIZE-1].soc;
    }

    // Find the appropriate voltage range and interpolate
    for (uint8_t i = 0; i < SOC_TABLE_SIZE-1; i++) {
        if (voltage_mv <= batterySOCTable[i].voltage &&
            voltage_mv > batterySOCTable[i+1].voltage) {
            // Linear interpolation using integer math
            uint32_t voltageDiff = batterySOCTable[i].voltage - batterySOCTable[i+1].voltage;
            uint8_t socDiff = batterySOCTable[i].soc - batterySOCTable[i+1].soc;

            // Scale up for better precision in integer division
            uint32_t ratio = ((batterySOCTable[i].voltage - voltage_mv) * 100) / voltageDiff;

            return batterySOCTable[i].soc - ((ratio * socDiff) / 100);
        }
    }

    // Fallback (should not reach here)
    return 0;
}
