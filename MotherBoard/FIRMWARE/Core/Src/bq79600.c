/**
  ******************************************************************************
  * @file           : bq79600.c
  * @brief          : BQ79600/BQ79616 function implementations
  ******************************************************************************
  */

#include <bq79600.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>


// GLOBAL VARIABLES (use these to avoid stack overflows by creating too many function variables)
// avoid creating variables/arrays in functions, or you will run out of stack space quickly
uint16_t crc = 0;
HAL_StatusTypeDef status;
uint32_t timeout;

// SpiWriteFrame
uint8_t tx_data[8];
uint8_t rx_data[128];
float voltStackRead[ACTIVECHANNELS];

int M = 0; // expected total response bytes
int i = 0; // number of groups of 128 bytes
int K = 0; // number of bytes remaining in the last group of 128

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim4;

extern void Delay_us(uint32_t us);

// CRC16 TABLE
// ITU_T polynomial: x^16 + x^15 + x^2 + 1
const uint16_t crc16_table[256] = {0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301,
                                   0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1,
                                   0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81,
                                   0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
                                   0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00,
                                   0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1,
                                   0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380,
                                   0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180, 0xF141,
                                   0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501,
                                   0x35C0, 0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0,
                                   0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881,
                                   0x3840, 0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
                                   0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401,
                                   0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1,
                                   0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0, 0x6180,
                                   0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740,
                                   0xA501, 0x65C0, 0x6480, 0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01,
                                   0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1,
                                   0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80,
                                   0xBA41, 0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
                                   0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200,
                                   0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1,
                                   0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780,
                                   0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0, 0x5D80, 0x9D41,
                                   0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901,
                                   0x59C0, 0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1,
                                   0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80,
                                   0x8C41, 0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
                                   0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

/**
 * @brief  Calculates CRC-16-IBM for BQ79600 communication
 * @param  data: Buffer containing the data
 * @param  length: Length of data
 * @retval uint16_t: Calculated CRC value
 */
uint16_t SpiCRC16(uint8_t* pBuf, int sendLen)
{
    uint16_t wCRC = 0xFFFF;
    int i;

    for (i = 0; i < sendLen; i++)
    {
        wCRC ^= (uint16_t)(pBuf[i] & 0x00FF);
        wCRC = crc16_table[wCRC & 0x00FF] ^ (wCRC >> 8);
    }

    //printf("CRC16 calculated: 0x%04X\n", wCRC);

    return wCRC;
}

/**
 * @brief  Temporarily disable SPI1 to control MOSI pin as GPIO
 * @retval None
 */
void SPI1_DisableForGPIO(void)
{
  // Disable SPI1
  __HAL_SPI_DISABLE(&hspi1);

  // Configure MOSI pin (PA7) as GPIO output
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_7;        // PA7 is SPI1_MOSI on most STM32F4 boards
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief  Restore SPI1 configuration for normal operation
 * @retval None
 */
void SPI1_RestoreFromGPIO(void)
{
  // Re-initialize SPI1 pins to their original function
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // SCK pin (PA5)
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // MISO pin (PA6)
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // MOSI pin (PA7)
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Re-enable SPI1
  __HAL_SPI_ENABLE(&hspi1);
}

/*
 *
 * @brief  Performs the BQ79600-Q1 wakeup sequence
 * @param  num_stacked_devices: Number of stacked BQ79616-Q1 devices
 * @param  need_double_wake: Set to true if device was previously shut down using SHUTDOWN ping
 * @retval HAL status
*/
HAL_StatusTypeDef BQ79600_WakeUp(uint8_t num_stacked_devices, bool need_double_wake)
{

  // 1. Send WAKE ping - begin by disabling SPI to control MOSI directly
  SPI1_DisableForGPIO();

  // Configure NSS pin (PA4) as GPIO output
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_4;  // NSS pin
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // If device was shut down with SHUTDOWN ping, we need two WAKE pings
  if (need_double_wake) {
    // First WAKE ping
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // Hold nCS low
    Delay_us(2);  // Wait 2us

    // Pull MOSI low for 2.75ms (tHLD_WAKE)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    Delay_us(BQ79600_WAKE_PING_TIME_US);  // 2.75ms
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

    Delay_us(2);  // Wait 2us
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // Bring nCS back high

    // Wait for first wake ping to process (3.5ms)
    Delay_us(BQ79600_WAKE_SETUP_TIME_US);  // 3.5ms
  }

  // Send (second) WAKE ping
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // Hold nCS low
  Delay_us(2);  // Wait 2us

  // Pull MOSI low for 2.75ms (tHLD_WAKE)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
  Delay_us(BQ79600_WAKE_PING_TIME_US);  // 2.75ms
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

  Delay_us(2);  // Wait 2us
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // Bring nCS back high

  // Restore SPI configuration
  SPI1_RestoreFromGPIO();

  // 2. Wait for tSU(WAKE_SHUT) to allow BQ79600-Q1 to enter ACTIVE mode (3.5ms)
  HAL_Delay(4);

  // 4. Send a single device write to set CONTROL1[SEND_WAKE]=1, which wakes up all stacked devices
  tx_data[0] = 0x90;  // Single device 1 byte write
  tx_data[1] = 0x00;  // Device address
  tx_data[2] = 0x03;  // MSB register address
  tx_data[3] = 0x09;  // LSB register address
  tx_data[4] = 0x20;  // 00100000 (enable SEND_WAKE)

  SpiWrite(5);

  return HAL_OK;
}

/**
 * @brief  Auto address the bq79600s. Set all devices to stack mode, set the highest device as top of stack, synchronize the DLL.
 * @param  None
 * @retval None
 */
HAL_StatusTypeDef SpiAutoAddress(uint8_t num_stacked_devices)
{
	//SYNC DLL
	tx_data[0] = 0xB0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x43;
	tx_data[3] = 0x00;
	SpiWrite(4);

	tx_data[0] = 0xB0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x44;
	tx_data[3] = 0x00;
	SpiWrite(4);

	tx_data[0] = 0xB0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x45;
	tx_data[3] = 0x00;
	SpiWrite(4);

	tx_data[0] = 0xB0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x46;
	tx_data[3] = 0x00;
	SpiWrite(4);

	tx_data[0] = 0xB0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x47;
	tx_data[3] = 0x00;
	SpiWrite(4);

	tx_data[0] = 0xB0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x48;
	tx_data[3] = 0x00;
	SpiWrite(4);

	tx_data[0] = 0xB0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x49;
	tx_data[3] = 0x00;
	SpiWrite(4);

	tx_data[0] = 0xB0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x4A;
	tx_data[3] = 0x00;
	SpiWrite(4);

	//Enable auto-addressing mode
	tx_data[0] = 0xD0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x09;
	tx_data[3] = 0x01;
	SpiWrite(4);

	//Set device addresses
	tx_data[0] = 0xD0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x06;
	tx_data[3] = 0x00;
	SpiWrite(4);

	tx_data[0] = 0xD0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x06;
	tx_data[3] = 0x01;
	SpiWrite(4);

	//set all stacked devices as stack
	tx_data[0] = 0xD0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x08;
	tx_data[3] = 0x02;
	SpiWrite(4);

	//set top device to be top of stack
	tx_data[0] = 0x90;
	tx_data[1] = 0x01;
	tx_data[2] = 0x03;
	tx_data[3] = 0x08;
	tx_data[4] = 0x03;
	SpiWrite(5);

	//SYNC DLL
	tx_data[0] = 0xA0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x43;
	tx_data[3] = 0x00;
	SpiRead(4,2);

	tx_data[0] = 0xA0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x44;
	tx_data[3] = 0x00;
	SpiRead(4,2);

	tx_data[0] = 0xA0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x45;
	tx_data[3] = 0x00;
	SpiRead(4,2);

	tx_data[0] = 0xA0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x46;
	tx_data[3] = 0x00;
	SpiRead(4,2);

	tx_data[0] = 0xA0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x47;
	tx_data[3] = 0x00;
	SpiRead(4,2);

	tx_data[0] = 0xA0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x48;
	tx_data[3] = 0x00;
	SpiRead(4,2);

	tx_data[0] = 0xA0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x49;
	tx_data[3] = 0x00;
	SpiRead(4,2);

	tx_data[0] = 0xA0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x4A;
	tx_data[3] = 0x00;
	SpiRead(4,2);


	//Read from 0x0306
	tx_data[0] = 0x80;
	tx_data[1] = 0x00;
	tx_data[2] = 0x03;
	tx_data[3] = 0x06;
	tx_data[4] = 0x00;
	SpiRead(5,7);
	printf("ADDR_0: 0x%02X\n", rx_data[4]);

	tx_data[0] = 0x80;
	tx_data[1] = 0x01;
	tx_data[2] = 0x03;
	tx_data[3] = 0x06;
	tx_data[4] = 0x00;
	SpiRead(5,7);
	printf("ADDR_1: 0x%02X\n", rx_data[4]);

	tx_data[0] = 0x80;
	tx_data[1] = 0x00;
	tx_data[2] = 0x20;
	tx_data[3] = 0x01;
	tx_data[4] = 0x00;
	SpiRead(5,7);
	printf("DEV_CONF: 0x%02X\n", rx_data[4]);


	if(status != HAL_OK) {
		return status;
	}

    return HAL_OK;
}

HAL_StatusTypeDef SpiWrite(int sendLen)
{
	  crc = SpiCRC16(tx_data, sendLen);
	  tx_data[sendLen] = crc & 0xFF;
	  tx_data[sendLen + 1] = (crc >> 8) & 0xFF;

	  //Check if SPI_READY is high, with timeout
	  timeout = HAL_GetTick() + 100;  // 100ms timeout
	  while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) != GPIO_PIN_SET) {
	    if (HAL_GetTick() >= timeout) {
	      return HAL_TIMEOUT;
	    }
	    Delay_us(100);
	  }

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	  Delay_us(0.5); //t9

	  // Send the command
	  status = HAL_SPI_Transmit(&hspi1, tx_data, sendLen + 2, 100);

	  // Pull nCS high
	  Delay_us(0.5); //t10
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	  Delay_us(1);

	  if (status != HAL_OK) {
	    return status;
	  }

	  return HAL_OK;
}

HAL_StatusTypeDef SpiRead(int sendLen, int returnLen){

	SpiWrite(sendLen);

	for(int i = 0; i <= 7; i++) {
		tx_data[i] = 0xFF;
	}

	timeout = HAL_GetTick() + 1;  // 1ms timeout
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) != GPIO_PIN_SET) {
		if (HAL_GetTick() >= timeout) {
			SpiClear();
			return HAL_TIMEOUT;
		}
		Delay_us(500);
	}

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    Delay_us(0.5); //t9

    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, returnLen, HAL_MAX_DELAY);

    Delay_us(0.5); //t10

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    Delay_us(1);

    if (status != HAL_OK) {
    	return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef SpiClear(){

	tx_data[0] = 0x00;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // Hold nCS low
	Delay_us(0.5);
	status = HAL_SPI_Transmit(&hspi1, tx_data, 1, 100);
	Delay_us(0.5);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // Pull nCS high

	if (status != HAL_OK) {
		    return status;
		  }

	return HAL_OK;
}

HAL_StatusTypeDef stackVoltageRead(){

	/*
	B0 00 03 0A A6 13 //Step 1 (16 active cells)
	B0 03 0D 06 52 76 //Step 2 (set continuous run and start ADC)
	delay [192us + (5us x TOTALBOARDS)] //Step 3 (delay)
	A0 05 68 1F 5C 2D //Step 4 (read ADC measurements)
	*/

	tx_data[0] = 0xB0;
	tx_data[1] = 0x00;
	tx_data[2] = 0x03;
	tx_data[3] = 0x0A;
	status = SpiWrite(4);

	if (status != HAL_OK) {
		return status;
	}

	tx_data[0] = 0xB0;
	tx_data[1] = 0x03;
	tx_data[2] = 0x0D;
	tx_data[3] = 0x06;
	status = SpiWrite(4);

	if (status != HAL_OK) {
		return status;
	}

	Delay_us(192 + (5 * TOTALBOARDS));

	tx_data[0] = 0xA0;
	tx_data[1] = 0x05;
	tx_data[2] = 0x68;
	tx_data[3] = 0x0F;

	/*
	status = SpiRead(4,ACTIVECHANNELS * 2);
	for(int i = 0; i <= (ACTIVECHANNELS * 2) - 1; i++){
		printf("Cell %d: %d\r\n", i, rx_data[i]);
	}

	//convert readings to voltages
	for(int i = 0; i <= ACTIVECHANNELS; i++){
		voltStackRead[i] = ((float)rx_data[RESPONSE_HEADER_SIZE + i]) * 190.7;
	}

	printf("---------------");
	for(int i = 0; i <= ACTIVECHANNELS - 1; i++){
		printf("Cell %d: %f \r\n", i, voltStackRead[i]);
	}
	printf("---------------");
	*/
	if (status != HAL_OK) {
	    	return status;
	    }

	return HAL_OK;

}

HAL_StatusTypeDef spiWriteFrame(uint8_t devAddr, uint16_t regAddr, uint8_t* data, uint8_t dataSize, uint8_t packetType){
	tx_data[0] = packetType;
	tx_data[1] = devAddr;
	tx_data[2] = (regAddr >> 8) & 0xFF;
	tx_data[3] = regAddr & 0xFF;
	for(int i = 0; i <= (dataSize - 1); i++){
		tx_data[i + 4] = data[i];
	}
	crc = SpiCRC16(tx_data, dataSize + 4);
	//tx_data[sendLen] = crc & 0xFF;
	//tx_data[sendLen + 1] = (crc >> 8) & 0xFF;

	return HAL_OK;
}

