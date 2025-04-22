/**
  ******************************************************************************
  * @file           : bq79600.h
  * @brief          : BQ79600/BQ79616 function declarations
  ******************************************************************************
  */

#ifndef BQ79600_H
#define BQ79600_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include <stdbool.h>

/* Constants -----------------------------------------------------------------*/
// Timing constants (in microseconds)
#define BQ79600_WAKE_PING_TIME_US     2750  // 2.75ms for WAKE ping low time
#define BQ79600_WAKE_SETUP_TIME_US    3500  // 3.5ms wait after WAKE ping
#define BQ79600_WAKE_TONE_TIME_US     1600  // 1.6ms for WAKE tone duration
#define BQ79600_ACTIVE_MODE_TIME_US   10000 // 10ms to enter ACTIVE mode

// Command types
#define CMD_SINGLE_DEV_READ   0x80
#define CMD_SINGLE_DEV_WRITE  0x90
#define CMD_STACK_READ        0xA0
#define CMD_STACK_WRITE       0xB0
#define CMD_BROADCAST_READ    0xC0
#define CMD_BROADCAST_WRITE   0xD0
#define CMD_BROADCAST_WRITE_R 0xE0

// Register addresses
#define REG_DIR0_ADDR         0x0306
#define REG_DIR1_ADDR         0x0307
#define REG_COMM_CTRL         0x0308
#define REG_CONTROL1          0x0309
#define REG_CONTROL2          0x030A
#define REG_DEV_CONF1         0x2001

//Constants
// USER DEFINES
#define TOTALBOARDS 2     // boards in stack, including base device
#define ACTIVECHANNELS 14 // channels to activate (16 for BQ79616, 14 FOR BQ79614, etc)
#define BRIDGEDEVICE 1    //
#define MAXBYTES (16 * 2) // maximum number of bytes to be read from the devices (for array creation)
#define RESPONSE_HEADER_SIZE 7 // SPI response frame head size

#define FRMWRT_SGL_R 0x00     // single device READ
#define FRMWRT_SGL_W 0x10     // single device WRITE
#define FRMWRT_STK_R 0x20     // stack READ
#define FRMWRT_STK_W 0x30     // stack WRITE
#define FRMWRT_ALL_R 0x40     // broadcast READ
#define FRMWRT_ALL_W 0x50     // broadcast WRITE
#define FRMWRT_REV_ALL_W 0x60 // broadcast WRITE reverse direction

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief  Temporarily disable SPI1 to control MOSI pin as GPIO
 * @retval None
 */
void SPI1_DisableForGPIO(void);

/**
 * @brief  Restore SPI1 configuration for normal operation
 * @retval None
 */
void SPI1_RestoreFromGPIO(void);

HAL_StatusTypeDef SpiAutoAddress(uint8_t numStackedDevices);
uint16_t SpiCRC16(uint8_t *pBuf, int nLen);
HAL_StatusTypeDef SpiWrite(int nLen);
HAL_StatusTypeDef SpiRead(int nLen, int rLen);
HAL_StatusTypeDef SpiClear();
HAL_StatusTypeDef stackVoltageRead(int returnLen);
HAL_StatusTypeDef spiWriteReg(uint8_t devAddr, uint16_t regAddr, uint8_t data[], uint8_t sendLen, uint8_t packetType);
uint32_t convert_adc_to_voltage(uint8_t high_byte, uint8_t low_byte);
/**
 * @brief  Performs the BQ79600-Q1 wakeup sequence
 * @param  num_stacked_devices: Number of stacked BQ79616-Q1 devices
 * @param  need_double_wake: Set to true if device was previously shut down using SHUTDOWN ping
 * @retval HAL status
 */
HAL_StatusTypeDef BQ79600_WakeUp(uint8_t num_stacked_devices, bool need_double_wake);

#ifdef __cplusplus
}
#endif

#endif /* BQ79600_H */
