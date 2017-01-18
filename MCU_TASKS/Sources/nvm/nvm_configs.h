#ifndef NVMCONFIGS_H_
#define NVMCONFIGS_H_

/* Change this version every time the eeprom struc is changed and
 * inform the OS developers of a structure version change
 */
#define NVM_STRUC_VERSION			2

/* These offsets should not be changed without extensive testing
 * as you can overwrite data from previous FW if there is an offset change
 */
#define NVM_STRUC_VERSION_OFFSET		0
#define MCU_MANUFACTURING_DATA_OFFSET	2	 /* 254 bytes available */
#define MCU_BOARD_CONFIGS_OFFSET		256  /* 256 bytes available */
#define MCU_RUNTIME_CONFIGS_OFFSET		512  /* 256 bytes available */
#define MCU_USER_CONFIGS_OFFSET			768  /* 512 bytes available */
#define MCU_STATS_OFFSET				1280 /* 128 bytes available */
#define CRASH_DATA_OFFSET				1408 /* 128 bytes available */
#define CPU_BOARD_CONFIGS_OFFSET		1536 /* 508 bytes available */
#define NVM_END_OF_STRUCT_OFFSET		2044 /* 4 bytes available */
#define NVM_SIZE						2048

/* mcu_manufacturing_data_t size = 6+10+6+6+6 = 34 bytes */
typedef struct
{
	char device_mfg_date[6];
	char serial_num[10];
	char customer_id[6];
	char odm_id[6];
	char ems_id[6];
} mcu_manufacturing_data_t;

/* mcu_board_configs_t size = 2+2+2+2+2+2 = 12 bytes */
typedef struct
{
	uint16_t board_version;
	uint16_t wiggle_sensor_available; /* 0: not present on the MCU, 1: present on the MCU */
	uint16_t accel_on_mcu; 			  /* 0: accelerometer not present on MCU, 1: accelerometer present on the MCU */
	uint16_t GPIOs_available;		  /* bits 0-8 represent inputs, bits 9-16 represent outputs */
	uint16_t speakers_available; 	  /* bits 0: left speaker, bits 1: right speaker */
    uint16_t protocols_available; 	  /* bits 0: CAN1, 1: CAN2, 2: J1708, 3:  SWC, 4: 1 wire, RS232, RS422, …. */

//    uint16_t camera_available;  	  /* each bit 0-8 represents a camera */
//    struct s_battery battery_type;  /* battery present, capacity = 2700mA, type = chargeable/not chargeable */
//	  uint16_t microphone; 			  /* bits: 0: internal, 1: external */
//	  uint16_t leds_available; 		  /* number of LEDs available for the customer to control */
//    uint16_t usb_host_available; 	  /* 0: usb_host not available, 1: usb_host available */
} mcu_board_configs_t;

/* mcu_runtime_configs_t size = 2 bytes */
typedef struct  //offset = 16 + 240 = 256 (Max size = 64 bytes)
{
	uint16_t reboot_reason;
} mcu_runtime_configs_t;

/* mcu_user_configs_t size = 2+2+2 = 6 bytes */
typedef struct  //offset = 16 (Max size = 240 bytes)
{
	uint16_t ignition_threshold;
	uint16_t wiggle_cnt_threshold;
	uint16_t gpinput_thresholds; //TODO: define this better, needs to be a structure
} mcu_user_configs_t;

/* mcu_stats_t size = 4 bytes */
typedef struct				//offset = 256 + 64 = 320 (Max size = 64 bytes)
{
	uint32_t reboot_count;
} mcu_stats_t;

/* crash_data_t size = 4 bytes */
typedef struct crash_data	//offset = 320 + 64 = 384 (Max size = 128 bytes)
{
	uint32_t reason_for_crash; //TODO : Define this better
} crash_data_t;

/* cpu_board_configs_t size = 2+2+2+2+2+2+2+2 = 16 bytes */
//TODO: This structure should be managed by the OS and the MCU does not need to know what data is in here.
typedef struct
{
	uint16_t modem_type;			   /* 0: No Modem, 1: LTE,   */
	uint16_t wifi_type;
	uint16_t light_sensor;			   /*  0: no light sensor, 1 : light sensor present */
	uint16_t lcd_type; 				  /* in inches, 0 represents no LCD */
	uint16_t bluetooth;				   /* bit 0: internal, bit 1: external, other bits used for class , might want to make this a structure */
	uint16_t redbend_support; 		   /*0: no support, 1: has support */
	uint16_t communitake_support; 	   /*0: no support, 1: has support */
	uint16_t micronet_app_licenses;    /* stores whether micronet app licenses have been purchased, might want this to be a structure */
} cpu_board_configs_t;



//NVM_SIZE bytes allocated
typedef struct
{
	uint16_t NVM_struct_version; //offset = 0

	mcu_manufacturing_data_t mcu_manufacturing_data;
	char padding_mcu_manufacturing_data[MCU_BOARD_CONFIGS_OFFSET - MCU_MANUFACTURING_DATA_OFFSET - sizeof(mcu_manufacturing_data_t)];

	mcu_board_configs_t mcu_board_configs;
	char padding_mcu_board_configs[MCU_RUNTIME_CONFIGS_OFFSET - MCU_BOARD_CONFIGS_OFFSET - sizeof(mcu_board_configs_t)];

	mcu_runtime_configs_t mcu_runtime_configs;
	char padding_mcu_runtime_configs[MCU_USER_CONFIGS_OFFSET - MCU_RUNTIME_CONFIGS_OFFSET - sizeof(mcu_runtime_configs_t)];

	mcu_user_configs_t mcu_user_configs;
	char padding_mcu_user_configs[MCU_STATS_OFFSET - MCU_USER_CONFIGS_OFFSET - sizeof(mcu_user_configs_t)];

	mcu_stats_t mcu_stats;
	char padding_mcu_stats[CRASH_DATA_OFFSET - MCU_STATS_OFFSET - sizeof(mcu_stats_t)];

	crash_data_t crash_data;
	char padding_crash_data[CPU_BOARD_CONFIGS_OFFSET - CRASH_DATA_OFFSET - sizeof(crash_data_t)];

	cpu_board_configs_t cpu_board_configs;
	char padding_cpu_board_configs[NVM_END_OF_STRUCT_OFFSET - CPU_BOARD_CONFIGS_OFFSET - sizeof(cpu_board_configs_t)];

	char nvm_end_of_struct[4];
} settings_struct_t;

void nvm_config_init();
int write_full_NVM_struct(uint8_t skip_version);
int read_full_NVM_struct(void);
int write_NVM_var(void * pData, uint16_t size);
int read_NVM_var(void * pData, uint16_t size);
void get_serial_num_NVM(char * serial);
void set_serial_num_NVM(char * serial);
void flex_ram_test(void);
int write_settings_struct(settings_struct_t * data);
int read_settings_struct(settings_struct_t * data);

#endif /* NVMCONFIGS_H_ */
