/****************************************************************************
 *   Author: Alexandra Boulgakov                   
 *   Project: Current Integrator
 *
 *   Description:
 *   Current integrator code.
 *
 ***************************************************************************/
#include <project/driver_config.h>
#include <project/target_config.h>
#include <project/hardware.h>
#include <project/mcp3909.h>
#include <project/scandal_config.h>

#include <arch/timer.h>
#include <arch/gpio.h>
#include <arch/uart.h>
#include <arch/types.h>
#include <arch/can.h>
#include <arch/ssp.h>
#include <arch/wdt.h>

#include <math.h>
#include <scandal/eeprom.h>
#include <scandal/stdio.h>
#include <scandal/uart.h>
#include <scandal/leds.h>
#include <scandal/engine.h> /* for general scandal functions */
#include <scandal/message.h> /* for TELEM_LOW */

/* To allow for querying m and b values */
#include <scandal/utils.h>
#include <arch/flash.h>
#include <string.h>


/* global variables */
int i = 0; /* Used in main loop */
int64_t integral_scaling = 1000/DATA_INTEGRATE_INTERVAL;
int16_t chan0 = 0, chan1 = 0;
int64_t chan0_acc = 0, chan1_acc = 0; 
int64_t chan0_avg = 0, chan1_avg = 0, power_avg = 0;
int64_t chan0_integral = 0, power_integral = 0;
int64_t samples0 = 0, samples1 = 0 ;
//int64_t chan0_avg_old = 0, power_avg_old = 0;

uint64_t writebuf[2] = {0};

scandal_config my_config;
uint64_t chan0_m, chan0_b, chan1_m, chan1_b;

/*---------------------------------------------------------------------------*/

/* Helper function defs */
void send_data();
void int_trapz(int64_t *integral, int64_t *cur_val, int64_t *prev_val, int64_t time);
/*---------------------------------------------------------------------------*/

/* Setup functions */
void setup_ports(void) 
{
	GPIO_Init();

	// LEDS
	GPIO_SetDir(2,6,1); //Green LED, Out
	GPIO_SetDir(2,0,0); //Yel LED, Out

	// MCP3909 pins
	GPIO_SetDir(2,PGA,1); // PGA, Out
	GPIO_SetDir(2,nMCLR,1);// nMCLR, Out
	GPIO_SetDir(0,MCP3909_CS,1);// MCP3909_CS, Out
}

/*---------------------------------------------------------------------------*/

/* This is an in-channel handler. It gets called when a message comes in on the
 * registered node/channel combination as set up using the USB-CAN. */
void in_channel_0_handler(int32_t value, uint32_t src_time) {

	UART_printf("yay, my handler got called!\r\n");
	
	uint64_t buf[2] = {0};

	if (value == RESET_INTEGRAL) {
		chan0_integral = 0;
		power_integral = 0;
		sc_user_eeprom_write_block(0, (uint8_t *)buf, 8);					
	}
}

int main(void)
{

	setup_ports();
	scandal_init();
	mcp3909_init();
	
	int mcp3909_status = 0;

	UART_Init(115200);

	/* Initialise timers */
	sc_time_t one_sec_timer = sc_get_timer();

	sc_time_t data_integrate_timer = sc_get_timer();
	sc_time_t data_send_timer = sc_get_timer();
	sc_time_t data_save_timer = sc_get_timer();

	/* Read in previously saved integrated values */
	sc_user_eeprom_read_block(0, (uint8_t *)&chan0_integral, 8);
    sc_user_eeprom_read_block(8, (uint8_t *)&power_integral, 8);
	
	/* Set LEDs to known states, i.e. on */
	red_led(1);
	yellow_led(1);

	scandal_delay(100); /* wait for the UART clocks to settle */

	scandal_register_in_channel_handler(0, &in_channel_0_handler);

	//WDTInit();
	
	/* This is the main loop, go for ever! */
	while (1) {
		/* This checks whether there are pending requests from CAN, and sends a heartbeat message.
		 * The heartbeat message encodes some data in the first 4 bytes of the CAN message, such as
		 * the number of errors and the version of scandal */
		handle_scandal();

		/* Get current and voltage samples. Sampling happens once every main loop, but
		 * the values are only averaged every DATA_INTEGRATE_INTERVAL */
		mcp3909_status = mcp3909_sample(&chan0, &chan1);
		if (mcp3909_status == MCP3909_FAIL){
		  UART_printf("MCP_FAIL\r\n");
		  mcp3909_init();
		} else if(mcp3909_status == MCP3909_SUCCESS){
		  //UART_printf("SUCCESS c1=%d c2 =%d \r\n", chan0, chan1);
		}
		  		UART_printf("1 %d %d %d %d\r\n", chan0_acc, chan1_acc, samples0, samples1);

		chan0_acc += chan0;
		chan1_acc += chan1;
		samples0++;
		samples1++;
		UART_printf("2 %d %d %d %d\r\n", chan0_acc, chan1_acc, samples0, samples1);

		/* Flash an LED every second */
		if(sc_get_timer() >= one_sec_timer + 1000) {

			/* Twiddle the LEDs */
			toggle_red_led();

			UART_printf("time = %d, int_t = %d, send_t = %d, save_t = %d\r\n", (int)one_sec_timer, (int)data_integrate_timer, (int)data_send_timer, (int)data_save_timer);

			/* Update the timer */
			one_sec_timer = sc_get_timer();
	
		}

		/* Average samples and add to integrals every DATA_INTEGRATE_INTERVAL */
		if (sc_get_timer() >= data_integrate_timer + DATA_INTEGRATE_INTERVAL) {
			/*** Averaging stuff ***/
			/* chan0_acc and chan1_acc contain accumulated values when passed to this function
			 * and the average value after it is executed */
			scandal_get_scaleaverage64(CURRENTINT_CURRENT, &chan0_acc, &samples0);
			scandal_get_scaleaverage64(CURRENTINT_VOLTAGE, &chan1_acc, &samples1);

			/* move average values to new variables to avoid confusion and reset 
			 * accumulated variables and samples */			
			chan0_avg = chan0_acc;
			chan1_avg = chan1_acc;
			chan0_acc = 0;
			chan1_acc = 0;
			samples0 = 0;
			samples1 = 0;
			/* Get average power. [mA][mV]=[uW] so need to divie by 1000 to get [mW] */
			power_avg = (chan0_avg * chan1_avg)/1000;

			/*** Integration stuff ***/
			//int_trapz(&chan0_integral, &chan0_avg, &chan0_avg_old, DATA_INTEGRATE_INTERVAL);
			//int_trapz(&power_integral, &power_avg, &power_avg_old, DATA_INTEGRATE_INTERVAL);
			chan0_integral += chan0_avg / integral_scaling;
			power_integral += power_avg / integral_scaling;

			/* Move new average values to old for next trapezoidal integration */
			//chan0_avg_old = chan0_avg;
			//power_avg_old = power_avg;
			
			//UART_printf("INT\r\n"); //Happens very often so lets comment this out for now!

			data_integrate_timer = sc_get_timer();

		}

		/* Process and send data every DATA_SEND_INTERVAL */ 
		if (sc_get_timer() >= data_send_timer + DATA_SEND_INTERVAL) {

			/* Read config, see if it has changed */			
			sc_read_conf(&my_config);
			chan0_m = my_config.outs[0].m;
			chan0_b = my_config.outs[0].b;
			chan1_m = my_config.outs[1].m;
			chan1_b = my_config.outs[1].b;

			/* Send the data */
			send_data();

			/* Look we're sending stuff */
			toggle_yellow_led();
			UART_printf("SEND\r\n");
			data_send_timer = sc_get_timer();
		}

		/* Save integrated values to flash every DATA_SAVE_INTERVAL */
		if (sc_get_timer() >= data_save_timer + DATA_SAVE_INTERVAL) {
			
			writebuf[0] = chan0_integral;
			writebuf[1] = power_integral;
			
			sc_user_eeprom_write_block(0, (uint8_t *)writebuf, 16);
	
			/* Acknowledge the fact that we just did a flash write */
			scandal_send_channel(TELEM_LOW, 			 	/* priority */
								5,     						/* channel num */
								DATA_SAVE_INTERVAL		 	/* value */
			);
			
			data_save_timer = sc_get_timer();
		}


		/* Feed the watchdog */
		//WDTFeed();
	}
}

void send_data()
{			

	/* Current and voltage */
	scandal_send_channel(TELEM_LOW, 			 	/* priority */
						CURRENTINT_CURRENT,     	/* channel num */
						chan0_avg    			 	/* value */
	);

	scandal_send_channel(TELEM_LOW, 				/* priority */
						CURRENTINT_VOLTAGE,      	/* channel num */
						chan1_avg    				/* value */
	);
	

	/* INT_SCALING is necessary to convert from mA*ms to Ah. */
	scandal_send_channel(TELEM_LOW, 				/* priority */
						2,      /* channel num */
						chan0_integral/3600 	/* value */
	);

	scandal_send_channel(TELEM_LOW, 				/* priority */
						3,      		/* channel num */
						power_avg 	/* value */
	);

	/* INT_SCALING is necessary to convert from mW*ms to Wh. */
	scandal_send_channel(TELEM_LOW, 				/* priority */
						4,      	/* channel num */
						power_integral/3600 	/* value */
	);

	scandal_send_channel(TELEM_LOW, 			 	 //priority
						11,     // channel num
						chan0_m	 	// value );
	);
			
	scandal_send_channel(TELEM_LOW, 			 	 //priority
						12,     // channel num
						chan0_b	 	// value );
	);
	
	scandal_send_channel(TELEM_LOW, 			 	 //priority
						13,     // channel num
						chan1_m	 	// value );
	);
			
	scandal_send_channel(TELEM_LOW, 			 	 //priority
						14,     // channel num
						chan1_b	 	// value );
	);
}

/* Calculates trapezoidal integral between two values and adds it to the integral */
void int_trapz(int64_t *integral, int64_t *cur_val, int64_t *prev_val, int64_t time)
{
	/* Values are in mA and time is in ms */
	int64_t increment = ((*cur_val + *prev_val)/2) * (1000/time);
	*integral = *integral + increment;
}
