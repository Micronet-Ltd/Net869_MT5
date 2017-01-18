/*
 * configs.c
 *
 *  Created on: Jan 17, 2016
 *      Author: esmail
 */
#include <mqx.h>
#include <bsp.h>
#include "nvm.h"
#include "nvm_configs.h"
#include "FTFx_KX_flash_config.h"

/***************************************************************************//**
 * Definitions
 ******************************************************************************/


/***************************************************************************//**
 * Global Variables
 ******************************************************************************/
settings_struct_t nv_data =
{

		.NVM_struct_version = NVM_STRUC_VERSION, //offset = 0

		.mcu_manufacturing_data =
		{
				.device_mfg_date = "123456",
				.serial_num = "1234567890",
				.customer_id = "123456",
				.odm_id = "123456",
				.ems_id = "123456",
		},
		.mcu_board_configs =
		{
				.board_version = 1,
				.wiggle_sensor_available = 1,
				.accel_on_mcu = 1,
				.GPIOs_available = 0xFFFF,
				.speakers_available = 1,
				.protocols_available = 0xFFFF,
		},
		.mcu_runtime_configs =
		{
				.reboot_reason = 0,
		},
		.mcu_user_configs =
		{
				.ignition_threshold = 12000, //TODO: use correct value from define
				.wiggle_cnt_threshold = 20, //TODO: use correct value from define
				//.gpinput_thresholds, //TODO: define this better, needs to be a structure
		},
		.mcu_stats =
		{
				.reboot_count = 0,
		},
		.crash_data =
		{
				.reason_for_crash = 0,
		},
		.nvm_end_of_struct = "STOP",
};

//SettingsStruct * pNVData = &nvData;
void * p_nv_data = &nv_data;

/***************************************************************************//**
 * Prototypes
 ******************************************************************************/

/***************************************************************************//**
 * Code
 ******************************************************************************/

void nvm_config_init(void)
{
	uint16_t default_NVM_struct_version = nv_data.NVM_struct_version;
	read_NVM_var(&nv_data.NVM_struct_version, sizeof(nv_data.NVM_struct_version));
	/* if invalid struct in eeprom, restore defaults */
	if ( nv_data.NVM_struct_version != NVM_STRUC_VERSION)
	{
		nv_data.NVM_struct_version = default_NVM_struct_version;
		write_full_NVM_struct(0);
	}

	read_full_NVM_struct();	
}

/*! @fn void writeSettingsStruct(settingsStruct * data)
    @brief write settingStruct into NonVolatileEEEPROM 
    @return success = 0, failure -1
*/

int write_full_NVM_struct(uint8_t skip_version)
{
	int ret = 0;
	if (skip_version)
	{
		ret = write_block_to_EEPROM(MCU_MANUFACTURING_DATA_OFFSET, &nv_data.mcu_manufacturing_data, sizeof(nv_data) - MCU_MANUFACTURING_DATA_OFFSET);
	}
	else
	{
		ret = write_block_to_EEPROM(0, &nv_data, sizeof(nv_data));
	}
	return ret;
}

/*! @fn void readSettingsStruct(settingsStruct * data)
    @brief read settingStruct from NonVolatileEEEPROM
    @return success = 0, failure -1
*/
int read_full_NVM_struct()
{
	return read_block_from_EEPROM(0, &nv_data, sizeof(nv_data));
}

/*! @fn void writeSettingsStructVal(settingsStruct * data)
    @brief write settingStruct into NonVolatileEEEPROM 
    @return success = 0, failure -1
*/
//void read8bitFromFlexRAM(uint32_t addrOffset, uint8_t * data)
int write_NVM_var(void * pData, uint16_t size)
{
	uint32_t offset = ((uint8_t *)pData - (uint8_t *)p_nv_data) * sizeof(uint8_t);
	return write_block_to_EEPROM(offset , pData, size);
}

/*! @fn void readSettingsStruct(settingsStruct * data)
    @brief read settingStruct from NonVolatileEEEPROM
    @return success = 0, failure -1
*/
int read_NVM_var(void * pData, uint16_t size)
{
	uint32_t offset = ((uint8_t *)pData - (uint8_t *)p_nv_data) * sizeof(uint8_t);
	return read_block_from_EEPROM(offset , pData, size);
}

void get_serial_num_NVM(char * serial)
{
	read_NVM_var(serial, sizeof(nv_data.mcu_manufacturing_data.serial_num));
	//read_block_from_EEPROM(SERIAL_OFFSET, serial, 10);
}

void set_serial_num_NVM(char * serial)
{
	write_NVM_var(serial, sizeof(nv_data.mcu_manufacturing_data.serial_num));
	//write_block_to_EEPROM(SERIAL_OFFSET, serial, 10);
}

void flex_ram_test(void)
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
	status = write_full_NVM_struct(0);
	status = read_full_NVM_struct();
	nv_data.mcu_user_configs.ignition_threshold = 8000;
	status = write_NVM_var(&nv_data.mcu_user_configs.ignition_threshold, sizeof(nv_data.mcu_user_configs.ignition_threshold));
	nv_data.mcu_user_configs.ignition_threshold = 9999;
	status = read_NVM_var(&nv_data.mcu_user_configs.ignition_threshold, sizeof(&nv_data.mcu_user_configs.ignition_threshold));
}
