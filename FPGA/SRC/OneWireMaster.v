/************************************************************************************************************************************
* 1-Wire Master is a general implementation of 1-Wire protocol.                                                                     *
* The module expects to get 3 types of commands:                                                                                    *
*                                                                                                                                   *
*     RESET command - 1 single clock pulse command that causes the module to generate a reset pulse on the 1-Wire bus.              *
*                     following the Reset command, the module samples the bus status to detect if there is at least one device that *
*                     is present on the bus.                                                                                        *
*                                                                                                                                   *
*     Write command - 1 single clock pulse command that causes the module to read 8 bits (1 byte) from the serial input.            *
*                     The module Tx signal is connected to open drain transistor which causses the module to output the oposite     *
*                     value of the input bit. Each bit is sent in a defined time-slot, starts with 'start' (falling) pulse.         *
*                     At the end of each time-slot there is a recovery period letting the slave device detect next start pulse.     *
*                     During the recovery period, a shift pulse is generated for next bit preparation.                              *
*                                                                                                                                   *
*     Read command  - 1 single clock pulse command that causes the module to read 8 bits (1 byte) from the 1-wire bus.              *
*                     The module Rx signal is connected to open drain transistor which causses the module to get the oposite value  *
*                     of the input bit. Each bit is read in the midle of the time-slot after 'start' (falling) pulse is generated.  *
*                     At the end of each time-slot there is a recovery period letting the slave device detect next start pulse.     *
*                     During the recovery period, the input bit is writtent to RX register in register module                       *
*                                                                                                                                   *
* At the end of each command a single done pulse and ready signal are generted for IRQ and status reports.                          *
************************************************************************************************************************************/
`timescale 1 ns / 1 ps

module OneWireMaster (
	// data from/to HOST
	input      dataToSend,
	output reg dataRecieved  = 0,
	
	// control signals
	input      enable           ,
	input      startResetPulse  ,
	input      startDataWrite   ,
	input      startDataRead    ,
	output reg presentStatus = 0,
	output reg shift         = 0,
	output reg ready         = 0,
	output reg done          = 0,
	
	
	// 1 Wire signals
	input      oneWireRx        ,
	output reg oneWireTx     = 0,
	
	input      rst              ,
	input      clk              
);

parameter	CLK_FRQ_MHZ   =   24 ;

localparam 
TIME_1uSec               =	 (1 * CLK_FRQ_MHZ - 2),
TIMESTAMP_START_PULSE    =	  12,
TIMESTAMP_SAMPLE_RX      =	  20,
TIMESTAMP_TIMESLOT       =	  55,
TIMESTAMP_RECOVERY       =	  60,
TIMESTAMP_RESET          =	 480,
TIMESTAMP_SAMPLE_PRESENT =	 580,
TIMESTAMP_RESET_DONE     =	 720;


reg  [31:0] freeTimerCnt     = 0;
reg  [10:0] microsecCnt      = 0;
reg         microSecTick     = 0;
reg  [6:0]  bitCnt           = 0;

reg         resetPulse       = 0;
reg         resetPulseDone   = 0;

reg         presentSample    = 0;
reg         presentPulse     = 0;
reg         presentPulseDone = 0;

reg         startPulse       = 0;
reg         startPulseDone   = 0;
reg         timeslot         = 0;
reg         newTimeSlot      = 0;
reg         timeslotDone     = 0;
reg         recoveryTime     = 0;
reg         recoveryDone     = 0;
reg         write_cycle      = 0;
reg         read_cycle       = 0;
reg         dataSample       = 0;
reg         dataTx           = 0;

reg         enabled_sampled  = 0;

wire lastDataBitFlag = bitCnt[3];

/*********************************************************** TIME COUNTER ***********************************************************
* FreeTimerCnt is counter that starts when a new command is generated (reset, write, read) and last till command is fully executed. *
* FreeTimerCnt is being used to generate pulse every 1 uSec.                                                                        *
*                                                                                                                                   *
* MicrosecCnt is counter that counts the amount of micro-seconds have passed since the begining of the command.                     *
*                                                                                                                                   *
* Bit counter counts the number of bits that were sent or rceived.                                                                  *
************************************************************************************************************************************/
always @(posedge clk or posedge rst)
begin
	if      (rst)							freeTimerCnt    <= 0;
	else if (microSecTick | ready | !enable)			freeTimerCnt    <= 0;
	else 								freeTimerCnt    <= freeTimerCnt + 1;
end

always @(posedge clk or posedge rst)
begin
	if (rst)
		{microsecCnt, microSecTick, bitCnt} <= 0;
	else begin
		if (ready | !enable | recoveryDone)
			microsecCnt <= 0;
		else if (microSecTick)
			microsecCnt <= microsecCnt + 1;

		if (ready | !enable)
			bitCnt <= 0;
		else if (shift)
			bitCnt <= bitCnt + 1;
		
		microSecTick  <= freeTimerCnt == TIME_1uSec;
	end
end

/************************************************************** RESET ***************************************************************
* Reset pulse starts with the assertion of 'start reset pulse' and last TIMESTAMP_RESET_DONE micro-seconds.                         *
* At the first TIMESTAMP_RESET uSec, the 1-wire bus is forced to be LOW. After that the bus returns to its default value (3-state). *
* When micro second counter reaches to TIMESTAMP_SAMPLE_PRESENT, the module samples the 1-wire bus.                                 *
*      If the bus is LOW,  then there is at least one device on the bus.                                                            *
*      If the bus is HIGH, no device is present on the bus.                                                                         *
* When micro second counter reaches TIMESTAMP_RESET_DONE, the reset pulse terminates.                                               *
************************************************************************************************************************************/
always @(posedge clk or posedge rst)
begin
	if (rst)
		{resetPulseDone, presentPulseDone, presentSample, resetPulse, presentPulse, presentStatus } <= 0;
	else if (done | !enable)
		{resetPulseDone, presentPulseDone, presentSample, resetPulse, presentPulse} <= 0;
	else begin
		resetPulseDone   <= microSecTick & (microsecCnt == TIMESTAMP_RESET          );
		presentPulseDone <= microSecTick & (microsecCnt == TIMESTAMP_RESET_DONE     );
		presentSample    <= microSecTick & (microsecCnt == TIMESTAMP_SAMPLE_PRESENT );
	
		resetPulse       <= resetPulseDone   ? 1'b0      : startResetPulse ? 1'b1    : resetPulse   ;		// reset pulse starts with START generation till counter reaches 480us
		presentPulse     <= presentPulseDone ? 1'b0      : resetPulseDone  ? 1'b1    : presentPulse ;		// present pulse starts with when reset pulse done and last till counter reaches 720us
		presentStatus    <= presentSample    ? oneWireRx : presentStatus;									// sample present of devices on bus
	end
end


/************************************************************ TIME SLOT *************************************************************
* The time slot state starts whenever a 'start write pulse' or a 'start read pulse' is asserted.                                    *
* Each time slot last 61 uSec as following:                                                                                         *
*          0 uSec                to TIMESTAMP_START_PULSE - Start of time slot - the module forces the bus to be LOW                *
*          TIMESTAMP_START_PULSE to TIMESTAMP_TIMESLOT    - Write cycles : the module outputs the bit value                         *
*                                                         - Read  cycles : the module puts the bus in 3-state                       *
*          TIMESTAMP_TIMESLOT    to TIMESTAMP_RECOVERY    - The module puts the bus in 3-state (recovery period)                    *
*          TIMESTAMP_SAMPLE_RX                            - The bus is sampled for reading                                          *
*                                                                                                                                   *
* The module Rx input  is connected to open-drain transistor drain. Thefore the Rx input  is the negative value of the Rx signal.   *
* The module Tx output is inverted in another process                                                                               *
************************************************************************************************************************************/
always @(posedge clk or posedge rst)
begin
	if (rst)
		{newTimeSlot, timeslot, timeslotDone, startPulse, startPulseDone, 
		 recoveryTime, recoveryDone, write_cycle, read_cycle, dataSample} <= 0;
	else if (done | !enable)
		{newTimeSlot, timeslot, timeslotDone, startPulse, startPulseDone, 
		 recoveryTime, recoveryDone, write_cycle, read_cycle, dataSample} <= 0;
	else begin
		// timeslot starts with START generation till counter reaches TIMESTAMP_RECOVERY of last bit
		newTimeSlot    <= startDataWrite | startDataRead | (recoveryDone &~ lastDataBitFlag);
		
		// time slot starts every new time slots till counter reaches TIMESTAMP_TIMESLOT		
		timeslot       <= timeslotDone ? 1'b0 : newTimeSlot  ? 1'b1 : timeslot    ;
		timeslotDone   <= timeslot     & microSecTick & (microsecCnt == TIMESTAMP_TIMESLOT    );
		
		// startPulse starts every new time slots till counter reaches TIMESTAMP_START_PULSE		
		startPulse     <= startPulseDone ? 1'b0 : newTimeSlot  ? 1'b1 : startPulse  ;
		startPulseDone <= timeslot     & microSecTick & (microsecCnt == TIMESTAMP_START_PULSE );		
		
		// recovery period is the time between the time slot is completed and new time slot can be generated
		recoveryTime   <= recoveryDone ? 1'b0 : timeslotDone ? 1'b1 : recoveryTime;
		recoveryDone   <= recoveryTime & microSecTick & (microsecCnt == TIMESTAMP_RECOVERY    );

		if (startDataWrite &~ startResetPulse &~ startDataRead)
			write_cycle <= 1;

		if (startDataRead  &~ startResetPulse &~ startDataWrite)
			read_cycle  <= 1;
		
		dataSample     <= timeslot & microSecTick & (microsecCnt == TIMESTAMP_SAMPLE_RX   );
	end
end
		
		
/************************************************************************************************************************************
* General signals:                                                                                                                  *
*    done            - Single clock pulse generated when last bit is recovered or reset and present pulse are completed             *
*    oneWireTx       - The data that is sent to the open drain transistor. notice it needs to be the oposite value of the data      *
*    ready           - bus status indicator - high when bus is in IDLE                                                              *
*    enabled_sampled - enable value with 1 clock delay                                                                              *
************************************************************************************************************************************/
always @(posedge clk or posedge rst)
begin
	if (rst)
		{enabled_sampled, done, ready, shift, dataTx, oneWireTx, dataRecieved} <= 0;
	else begin
		enabled_sampled <= enable;
		shift           <= timeslotDone;
		done            <= enable & ((lastDataBitFlag & microSecTick) | presentPulseDone);
		
		
		// ready is low when bus is busy or module not enabled
		if (!enable | startResetPulse | startDataWrite | startDataRead)
			ready <= 0;
		else if (done | ~enabled_sampled)
			ready <= 1;

		// timeslot data starts with 'start' LOW pulse where and ends in idle where Tx is HIGH
		// during read cycles, Tx is also idle (HIGH)
		if (startPulse)
			dataTx <= 0;
		else if (!enable | timeslotDone | (timeslot & read_cycle ))
			dataTx <= 1;			
		else if (timeslot & write_cycle) 
			dataTx <= dataToSend;
			
		// 1-wire bus Rx/Tx signals
		dataRecieved    <= dataSample ? !oneWireRx : dataRecieved;
		oneWireTx       <= enable & (resetPulse | ~dataTx);
	end
end

endmodule
