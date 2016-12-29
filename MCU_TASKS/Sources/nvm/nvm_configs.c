/*
 * configs.c
 *
 *  Created on: Mar 20, 2015
 *      Author: esmail
 */
#include <mqx.h>
#include <bsp.h>
#include "nvm.h"
#include "nvm_configs.h"
#include "FTFx_KX_flash_config.h"

//#include "Events.h"
//#include "mqx_tasks.h"

//#include "comm.h"
//#include "motor.h"
//#include "main.h"
//#include <cstddef>

/***************************************************************************//**
 * Definitions
 ******************************************************************************/


/***************************************************************************//**
 * Global Variables
 ******************************************************************************/
SettingsStruct nvData = 
{
		.serialNum = "TT12345678",
		.MotorConfigs = 
		{
				.countsPerRev = 76160,
				.intsPerSec = 1000, // interrupt every 1 ms
				.totalRevTime = 10, // 10 seconds for 1 revolution
				.maxTimeSlices = 10000, // INTS_PER_SEC * TOTAL_REV_TIME
				.calibratePwm = 30000,
				.rotKp = 150,
				.rotKi = .25,
				.rotKd = 0,
				.tiltKp = 150,
				.tiltKi = .25,
				.tiltKd = 0,
				.ramp = 1.5, // radians/second/second
				.slew = 0.7, // radians/second
				.maxCurrent = 20000,
				.maxPower = 5.0,
				.maxTilt = -180,
				.minTilt = 0,
				.hardStopToHome = 2.0, // the hardstop is at -2 degrees
				.maxCount = 0xFFFF, //maximum count for the quadrature decoder counters
				.maxPWM = 38000, //maximum pwm before over current error is issued
				.sigmaErrCap = 28000 // to keep the 'I' term from gaining too much momentum
		},
		.MotorRunTimeStats = 
		{
				.isHardstopCalibrated = 0,
				.isOptoCalibrated = 0
		},
		.SystemStats = 
		{
				.totalRotationAngle[0] = 0,
				.totalRotationAngle[1] = 0,
				.totalRotationAngle[2] = 0,
				.totalRotationAngle[3] = 0,
				.totalTiltAngle[0] = 0,
				.totalTiltAngle[1] = 0,
				.totalTiltAngle[2] = 0,
				.totalTiltAngle[3] = 0
		},
		.CrashData = 
		{
				.reasonForCrash = 0
		},
		.EEPROMVersion = EEPROM_STRUC_VERSION
};

//SettingsStruct * pNVData = &nvData;
void * pNVData = &nvData;

/***************************************************************************//**
 * Prototypes
 ******************************************************************************/

/***************************************************************************//**
 * Code
 ******************************************************************************/

void nvm_config_init(void)
{
	 readNVMVar(&nvData.EEPROMVersion, sizeof(nvData.EEPROMVersion));
	/* if invalid struct in eeprom, restore defaults */
	if ( nvData.EEPROMVersion != EEPROM_STRUC_VERSION)
	{
		writeFullNVMStruct(1);
	}

	readFullNVMStruct();	
}

/*! @fn void writeSettingsStruct(settingsStruct * data)
    @brief write settingStruct into NonVolatileEEEPROM 
    @return success = 0, failure -1
*/

int writeFullNVMStruct(uint8_t skipSerial)
{
	int ret = 0;
	if (skipSerial)
	{
		ret = writeBlockToEEPROM(MOTOR_CONFIG_OFFSET, &nvData.MotorConfigs, sizeof(nvData) - MOTOR_CONFIG_OFFSET);
	}
	else
	{
		ret = writeBlockToEEPROM(0, &nvData, sizeof(nvData));
	}
	return ret;
}

/*! @fn void readSettingsStruct(settingsStruct * data)
    @brief read settingStruct from NonVolatileEEEPROM
    @return success = 0, failure -1
*/
int readFullNVMStruct()
{
	return readBlockFromEEPROM(0, &nvData, sizeof(nvData));
}

/*! @fn void writeSettingsStructVal(settingsStruct * data)
    @brief write settingStruct into NonVolatileEEEPROM 
    @return success = 0, failure -1
*/
//void read8bitFromFlexRAM(uint32_t addrOffset, uint8_t * data)
int writeNVMVar(void * pData, uint16_t size)
{
	uint32_t offset = (&pData - &pNVData) * sizeof(uint8_t);
	return writeBlockToEEPROM(offset , pData, size);
}

/*! @fn void readSettingsStruct(settingsStruct * data)
    @brief read settingStruct from NonVolatileEEEPROM
    @return success = 0, failure -1
*/
int readNVMVar(void * pData, uint16_t size)
{
	uint32_t offset = (&pData - &(pNVData)) * sizeof(uint8_t);
	//uint32_t offset = (&pData - (&pData)) * sizeof(uint8_t);
	return readBlockFromEEPROM(offset , pData, size);
}

void getSerialNumNVM(char * serial)
{
	readBlockFromEEPROM(SERIAL_OFFSET, serial, 10);
}

void setSerialNumNVM(char * serial)
{
	writeBlockToEEPROM(SERIAL_OFFSET, serial, 10);
}

void flexRamTest(void)
{
	//uint32_t writeData32 = 0x12345678;
	//uint16_t writeData16 = 0xABCD;
	//uint16_t readData16 = 0;
	//uint32_t readData32 = 0;
	//write16bitToFlexRAM(0, &writeData16);
	//read16bitFromFlexRAM(0, &readData16);
	//write32bitToFlexRAM(0, &writeData32);
	//read32bitFromFlexRAM(0, &readData32);
	uint8_t status;
	status = writeFullNVMStruct(0);
	status = readFullNVMStruct();
	nvData.MotorConfigs.countsPerRev = 1234;
	status = writeNVMVar(&nvData.MotorConfigs.countsPerRev, sizeof(nvData.MotorConfigs.countsPerRev));
	nvData.MotorConfigs.countsPerRev= 9999;
	status = readNVMVar(&nvData.MotorConfigs.countsPerRev, sizeof(nvData.MotorConfigs.countsPerRev));
}

