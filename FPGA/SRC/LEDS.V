/************************************************************************************************
* LEDS module controls the PWM signal of each RGB LED. There are 3 sets of RGB LEDS on the board*
* and therefore the logic has 3 identical implementation. The module controls each color of the *
* RGB signals, where '1' turn on the LED, and '0' turns the LED off.                            *
*                                                                                               *
* Each color has RGB code value, and the module generates a PWM signals according to that code  *
* value. If the code value is 0 the signals is always off, if the code is 255. the signals is   *
* always on, and if it is 128, the signals is on half the time.                                 *
*                                                                                               *
* Another control signal is the bright control which implemented as PWM with duty cycle in 0.125*
* seconds resolution. If the duty cycle is set to 0, the led is off, If the duty cycle is 100%  *
* (0.125 * Frequency clocks), the LED is on, and if it is 50%, the led is on for 0.62 seconds.  *
************************************************************************************************/
`timescale 1 ns / 1 ps

`define PWM(signal, cnt, value)			signal <= (cnt == value) ? 0 : (cnt == 0) ? 1 : signal

module LEDS (
	output reg    led1_red         , led2_red         , led3_red         ,
	output reg    led1_green       , led2_green       , led3_green       ,
	output reg    led1_blue        , led2_blue        , led3_blue        ,
	
	input  [7:0]  led1_red_value   , led2_red_value   , led3_red_value   ,
	input  [7:0]  led1_green_value , led2_green_value , led3_green_value ,
	input  [7:0]  led1_blue_value  , led2_blue_value  , led3_blue_value  ,
	input  [7:0]  led1_DC_value    , led2_DC_value    , led3_DC_value    ,
	
	input         rst              ,
	input         clk
);

localparam	CLK_FRQ_MHZ                = 24  ,
			COLOR_CNT_MAX_VALUE        = 255 ,
			PULSE_PER_SECOND           = (CLK_FRQ_MHZ * 1024),							// Number of clocks per 1 second (a little bit more for implementation simplify)
			BRIGHTNESS_FACTOR          =  8  ,											// counter factor - should be 2^x for implementation simplify
			BRIGHTNESS_TIME_RESOLUTION = (PULSE_PER_SECOND / BRIGHTNESS_FACTOR) ;		// Brightness time resolution - 0.125 second

reg  [31:0] cnt_time       = 0;
reg  [ 7:0] cnt_color     = 0;

reg         led1_red_int   = 0;
reg         led1_green_int = 0;
reg         led1_blue_int  = 0;
reg         led1_dc_int    = 0;

reg         led2_red_int   = 0;
reg         led2_green_int = 0;
reg         led2_blue_int  = 0;
reg         led2_dc_int    = 0;

reg         led3_red_int   = 0;
reg         led3_green_int = 0;
reg         led3_blue_int  = 0;
reg         led3_dc_int    = 0;


// free running counters one for color code and another one for 5 seconds duty cycle
always @(posedge clk or posedge rst)
begin
	if (rst)
		{cnt_color, cnt_time} <= 0;
	else begin
		if (cnt_color ==   COLOR_CNT_MAX_VALUE       - 1)			cnt_color <= 0;
		else														cnt_color <= cnt_color + 1;
		
		if (cnt_time   == (BRIGHTNESS_TIME_RESOLUTION - 1))			cnt_time   <= 0;
		else														cnt_time   <= cnt_time   + 1;
	end
end

// generate PWM signals for each RGB LED color signal and for duty cycle signals
always @(posedge clk or posedge rst)
begin
	if (rst) begin
		{led1_red_int, led1_green_int, led1_blue_int, led1_dc_int} <= 0;
		{led2_red_int, led2_green_int, led2_blue_int, led2_dc_int} <= 0;
		{led3_red_int, led3_green_int, led3_blue_int, led3_dc_int} <= 0;
	end else begin		
		`PWM(led1_red_int  , cnt_color,  led1_red_value   );
		`PWM(led1_green_int, cnt_color,  led1_green_value );
		`PWM(led1_blue_int , cnt_color,  led1_blue_value  );
		`PWM(led1_dc_int   , cnt_time  , led1_DC_value * (BRIGHTNESS_TIME_RESOLUTION / 256));		// DC = (value / 256) * TIME_RESOLUTION

		`PWM(led2_red_int  , cnt_color,  led2_red_value   );
		`PWM(led2_green_int, cnt_color,  led2_green_value );
		`PWM(led2_blue_int , cnt_color,  led2_blue_value  );
		`PWM(led2_dc_int   , cnt_time  , led2_DC_value * (BRIGHTNESS_TIME_RESOLUTION / 256));		// DC = (value / 256) * TIME_RESOLUTION

		`PWM(led3_red_int  , cnt_color,  led3_red_value   );
		`PWM(led3_green_int, cnt_color,  led3_green_value );
		`PWM(led3_blue_int , cnt_color,  led3_blue_value  );
		`PWM(led3_dc_int   , cnt_time  , led3_DC_value * (BRIGHTNESS_TIME_RESOLUTION / 256));		// DC = (value / 256) * TIME_RESOLUTION		
	end
end

// 'AND' between duty cycle signal and LED color signal
always @(posedge clk or posedge rst)
begin
	if (rst) begin
		{led1_red, led1_green, led1_blue } <= 0;
		{led2_red, led2_green, led2_blue } <= 0;
		{led3_red, led3_green, led3_blue } <= 0;
	end else begin
		led1_red   <= led1_dc_int & led1_red_int    ;
		led1_green <= led1_dc_int & led1_green_int  ;
		led1_blue  <= led1_dc_int & led1_blue_int   ;

		led2_red   <= led2_dc_int & led2_red_int    ;
		led2_green <= led2_dc_int & led2_green_int  ;
		led2_blue  <= led2_dc_int & led2_blue_int   ;
                                    
		led3_red   <= led3_dc_int & led3_red_int    ;
		led3_green <= led3_dc_int & led3_green_int  ;
		led3_blue  <= led3_dc_int & led3_blue_int   ;
	end
end
endmodule


