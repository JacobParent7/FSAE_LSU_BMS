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
#include "main.h"
#include <stdbool.h>
#include "datatypes.h"

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
#define ACTIVECHANNELS 16 // channels to activate
#define BRIDGEDEVICE 1    //
#define MAXBYTES (16 * 2) // maximum number of bytes to be read from the devices (for array creation)
#define BAUDRATE 1000000  // device + uC baudrate

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

HAL_StatusTypeDef SpiAutoAddress(uint8_t num_stacked_devices);

int SpiWriteReg(BYTE bID, uint16_t wAddr, uint64_t dwData, BYTE bLen, BYTE bWriteType);
int SpiWriteFrame(uint16_t bID, uint16_t wAddr, uint16_t *pData, uint16_t bLen, uint8_t bWriteType);
int SpiReadReg(BYTE bID, uint16_t wAddr, uint16_t *pData, BYTE bLen, uint32_t dwTimeOut, BYTE bWriteType);

uint16_t SpiCRC16(uint8_t *pBuf, int nLen);

HAL_StatusTypeDef SpiWrite(int nLen);
HAL_StatusTypeDef SpiRead(int nLen, int rLen);
HAL_StatusTypeDef SpiClear();
/**
 * @brief  Performs the BQ79600-Q1 wakeup sequence
 * @param  num_stacked_devices: Number of stacked BQ79616-Q1 devices
 * @param  need_double_wake: Set to true if device was previously shut down using SHUTDOWN ping
 * @retval HAL status
 */
HAL_StatusTypeDef BQ79600_WakeUp(uint8_t num_stacked_devices, bool need_double_wake);

/**
 * @brief  Performs the BQ79600/BQ79616 auto-addressing sequence
 * @param  num_stacked_devices: Number of stacked BQ79616-Q1 devices
 * @retval HAL status
 */
HAL_StatusTypeDef BQ79600_AutoAddressing(uint8_t num_stacked_devices);

/**
 * @brief Initialize BQ79600 and perform auto-addressing for connected BQ79616 devices
 * @param num_stacked_devices: Number of stacked BQ79616-Q1 devices
 * @param need_double_wake: Set to true if device was previously shut down using SHUTDOWN ping
 * @retval HAL status
 */
HAL_StatusTypeDef BQ79600_Init(uint8_t num_stacked_devices, bool need_double_wake);

/**
 * @brief Initialize BQ79600 and perform auto-addressing (simplified version with one parameter)
 * @param num_stacked_devices: Number of stacked BQ79616-Q1 devices
 * @retval HAL status
 */
HAL_StatusTypeDef BQ79600_Init_Simple(uint8_t num_stacked_devices);

/**
 * @brief Send a command to the BQ79616 device via the BQ79600 bridge
 * @param cmd_type: Type of command (Single Device Write, Broadcast Write, etc.)
 * @param device_addr: Target device address (ignored for broadcast/stack commands)
 * @param reg_addr: Register address (16-bit)
 * @param data: Data to write (up to 8 bytes)
 * @param data_len: Length of data (1-8 bytes)
 * @retval HAL status
 */
HAL_StatusTypeDef BQ79600_SendCommand(uint8_t cmd_type, uint8_t device_addr,
                                      uint16_t reg_addr, uint8_t *data,
                                      uint8_t data_len);

/**
 * @brief Read data from a BQ79616 device via the BQ79600 bridge
 * @param cmd_type: Type of command (Single Device Read, Stack Read, etc.)
 * @param device_addr: Target device address (ignored for broadcast/stack commands)
 * @param reg_addr: Register address (16-bit)
 * @param num_bytes: Number of bytes to read (1-128)
 * @param rx_data: Buffer to store received data
 * @retval HAL status
 */
HAL_StatusTypeDef BQ79600_ReadData(uint8_t cmd_type, uint8_t device_addr,
                                   uint16_t reg_addr, uint8_t num_bytes,
                                   uint8_t *rx_data);

/**
 * @brief Send a SPI Comm Clear signal to reset the communication
 * @retval HAL status
 */
HAL_StatusTypeDef BQ79600_CommClear(void);

/**
 * @brief Read cell voltages from the BQ79616 device
 * @param device_addr: Target device address
 * @param cell_data: Buffer to store cell voltage data (at least 32 bytes)
 * @retval HAL status
 */
HAL_StatusTypeDef BQ79600_ReadCellVoltages(uint8_t device_addr, uint8_t *cell_data);

/**
 * @brief Set communication timeout values to prevent unexpected timeouts
 * @param short_timeout: Short timeout value (0-7)
 * @param long_timeout: Long timeout value (0-7)
 * @param long_timeout_action: Action on long timeout (0=SLEEP, 1=SHUTDOWN)
 * @retval HAL status
 */
HAL_StatusTypeDef BQ79600_SetCommTimeout(uint8_t short_timeout, uint8_t long_timeout, uint8_t long_timeout_action);

/**
 * @brief Read fault status registers to diagnose communication issues
 * @param fault_summary: Pointer to store FAULT_SUMMARY register
 * @param fault_comm1: Pointer to store FAULT_COMM1 register
 * @param fault_comm2: Pointer to store FAULT_COMM2 register
 * @retval HAL status
 */
HAL_StatusTypeDef BQ79600_ReadFaultStatus(uint8_t *fault_summary, uint8_t *fault_comm1, uint8_t *fault_comm2);

/**
 * @brief Reset communication fault registers
 * @retval HAL status
 */
HAL_StatusTypeDef BQ79600_ResetCommFaults(void);

/**
 * @brief Perform a soft reset of the BQ79600
 * @retval HAL status
 */
HAL_StatusTypeDef BQ79600_SoftReset(void);

/**
 * @brief Configure device settings for optimal operation
 * @retval HAL status
 */
HAL_StatusTypeDef BQ79600_ConfigureDevice(void);

void BQ79600_DebugSPI(void);
void BQ79600_TestSpiPins(void);
void BQ79600_BasicSpiTest(void);


#ifdef __cplusplus
}
#endif

#endif /* BQ79600_H */
