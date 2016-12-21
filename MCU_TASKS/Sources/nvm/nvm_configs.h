#ifndef NVMCONFIGS_H_
#define NVMCONFIGS_H_

/* Change this version every time the eeprom struc is changed and
 * inform the python developers of a structure version change
 */
#define EEPROM_STRUC_VERSION	1

#define SERIAL_OFFSET 					0
#define MOTOR_CONFIG_OFFSET 			16
#define MOTOR_RUNTIMESTATS_OFFSET 		256
#define SYSTEM_STATS_OFFSET 			320
#define CRASH_DATA_OFFSET 				384
#define EEPROM_STRUC_VER_OFFSET			510
#define NVM_SIZE						512

typedef struct motorConfigs  //offset = 16 (Max size = 240 bytes)
{
	float rotKp;
	float rotKi;
	float rotKd;
	float tiltKp;
	float tiltKi;
	float tiltKd;
	float ramp;							// radians/second
	float slew;							// radians/second
	float maxPower;
	float maxTilt;
	float minTilt;
	float hardStopToHome;		// the hardstop is at -2 degrees
	uint32_t countsPerRev;  // 32 quad counts * 2 pulses * 2 edges * 595 gear ratio
	uint16_t intsPerSec;		// interrupt every 1 ms
	uint16_t totalRevTime;	// 10 seconds for 1 revolution
	uint16_t maxTimeSlices;	// INTS_PER_SEC * TOTAL_REV_TIME
	uint16_t calibratePwm;
	uint16_t maxCount; 			//maximum count for the quadrature decoder counters
	uint16_t maxPWM; 				//maximum pwm before over current error is issued
	uint16_t sigmaErrCap; // to keep the 'I' term from gaining too much momentum
	uint16_t maxCurrent;
} MotorConfigs_t;

typedef struct motorRunTimeStats  //offset = 16 + 240 = 256 (Max size = 64 bytes)
{
	uint16_t isHardstopCalibrated;	//True or false
	uint16_t isOptoCalibrated; 			//True or false
} MotorRunTimeStats_t;

typedef struct systemStats				//offset = 256 + 64 = 320 (Max size = 64 bytes)
{
	uint32_t totalRotationAngle[4];
	uint32_t totalTiltAngle[4];
} SystemStats_t;

typedef struct crashData	//offset = 320 + 64 = 384 (Max size = 128 bytes)
{
	uint32_t reasonForCrash; //TODO : Define this better
} CrashData_t;

//512 bytes allocated
typedef struct settingsStruct
{
	char serialNum[10]; //offset = 0
	char serialPadding[MOTOR_CONFIG_OFFSET - SERIAL_OFFSET - 10];
	
	MotorConfigs_t MotorConfigs;
	char motorConfigsPadding[MOTOR_RUNTIMESTATS_OFFSET - MOTOR_CONFIG_OFFSET - sizeof(MotorConfigs_t)];
	
	MotorRunTimeStats_t MotorRunTimeStats;
	char motorRunTimeStatsPadding[SYSTEM_STATS_OFFSET - MOTOR_RUNTIMESTATS_OFFSET - sizeof(MotorRunTimeStats_t)];
	
	SystemStats_t SystemStats;
	char systemStatsPadding[CRASH_DATA_OFFSET - SYSTEM_STATS_OFFSET - sizeof(SystemStats_t)];
	
	CrashData_t CrashData;
	char crashDataPadding[EEPROM_STRUC_VER_OFFSET - CRASH_DATA_OFFSET - sizeof(CrashData_t)];
	uint16_t EEPROMVersion;
} SettingsStruct;

void getSerialNumNVM(char * serial);
void setSerialNumNVM(char * serial);
int writeFullNVMStruct(uint8_t skipSerial);
int readFullNVMStruct(void);
int writeNVMVar(void * pData, uint16_t size);
int readNVMVar(void * pData, uint16_t size);
void getSerialNumNVM(char * serial);
void setSerialNumNVM(char * serial);
void flexRamTest(void);
int writeSettingsStruct(SettingsStruct * data);
int readSettingsStruct(SettingsStruct * data);

#endif /* NVMCONFIGS_H_ */
