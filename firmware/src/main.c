/****************************************************************************
 *   Author: Alexandra Boulgakov                   
 *   Project: Current Integrator
 *
 *   Description:
 *   Current integrator code.
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/
#include <project/driver_config.h>
#include <project/target_config.h>
#include <project/hardware.h>
#include <project/mcp3909.h>

#include <arch/timer32.h>
#include <arch/gpio.h>
#include <arch/uart.h>
#include <arch/type.h>
#include <arch/can.h>
#include <arch/ssp.h>
#include <math.h>

#include <scandal/engine.h> /* for general scandal functions */
#include <scandal/message.h> /* for TELEM_LOW */

/* global variables */
int i = 0; /* Used in main loop */
int16_t chan0 = 0, chan1 = 0;
int64_t chan0_acc = 0, chan1_acc = 0; 
int64_t chan0_avg = 0, chan1_avg = 0, power_avg = 0;
int64_t chan0_avg_old = 0, power_avg_old = 0; // for integration
int64_t chan0_integral = 0;
int64_t power_integral = 0;
int64_t samples = 0;

/*---------------------------------------------------------------------------*/

/* Helper function defs */
void send_data();
void int_trapz(int64_t *integral, int64_t *cur_val, int64_t *prev_val, int64_t time);
/*---------------------------------------------------------------------------*/

/* Setup functions */
void setup_ports(void) 
{
	GPIOInit();

	// LEDS
	GPIOSetDir(2,6,1); //Green LED, Out
	GPIOSetDir(2,0,0); //Yel LED, Out

	// MCP3909 pins
	GPIOSetDir(2,PGA,1); // PGA, Out
	GPIOSetDir(2,nMCLR,1);// nMCLR, Out
	GPIOSetDir(0,MCP3909_CS,1);// MCP3909_CS, Out
}

/*---------------------------------------------------------------------------*/

int main(void)
{

	setup_ports();
	scandal_init();
	mcp3909_init();
	UARTInit(115200);

	/* Initialise timers */
	sc_time_t one_sec_timer = sc_get_timer();
	sc_time_t test_in_timer = sc_get_timer();

	sc_time_t data_integrate_timer = sc_get_timer();
	sc_time_t data_send_timer = sc_get_timer();
	sc_time_t data_save_timer = sc_get_timer();

	/* Read in previously saved integrated values */
	sc_user_eeprom_read_block(0, &chan0_integral, 8);

	/* Set LEDs to known states, i.e. on */
	red_led(1);
	yellow_led(1);

	scandal_delay(100); /* wait for the UART clocks to settle */

	/* This is the main loop, go for ever! */
	while (1) {
		/* This checks whether there are pending requests from CAN, and sends a heartbeat message.
		 * The heartbeat message encodes some data in the first 4 bytes of the CAN message, such as
		 * the number of errors and the version of scandal */
		handle_scandal();

		/* Get current and voltage samples. Sampling happens once every main loop, but
		 * the values are only averaged every DATA_INTEGRATE_INTERVAL */
		mcp3909_sample(&chan0, &chan1);
		chan0_acc += chan0;
		chan1_acc += chan1;
		samples++;


		/* Flash an LED every second */
		if(sc_get_timer() >= one_sec_timer + 1000) {

			/* Twiddle the LEDs */
			toggle_red_led();

			/* Update the timer */
			one_sec_timer = sc_get_timer();
	
		}

		/* Average samples and add to integrals every DATA_INTEGRATE_INTERVAL */
		if (sc_get_timer() >= data_integrate_timer + DATA_INTEGRATE_INTERVAL) {
			
			/*** Averaging stuff ***/
			/* chan0_acc and chan1_acc contain accumulated values when passed to this function
			 * and the average value after it is executed */
			scandal_get_scaleaverage64(CURRENTINT_CURRENT, &chan0_acc, &samples);
			scandal_get_scaleaverage64(CURRENTINT_VOLTAGE, &chan1_acc, &samples);

			/* move average values to new variables to avoid confusion and reset 
			 * accumulated variables and samples */			
			chan0_avg = chan0_acc;
			chan1_avg = chan1_acc;
			chan0_acc = 0;
			chan1_acc = 0;
			samples = 0;

			/* Get average power. [mA][mV]=[uW] so need to divie by 1000 to get [mW] */
			power_avg = (chan0_avg * chan1_avg)/1000;

			/*** Integration stuff ***/
			//int_trapz(&chan0_integral, &chan0_avg, &chan0_avg_old, DATA_INTEGRATE_INTERVAL);
			//int_trapz(&power_integral, &power_avg, &power_avg_old, DATA_INTEGRATE_INTERVAL);
			chan0_integral += chan0_avg;
			power_integral += power_avg;

			/* Move new average values to old for next trapezoidal integration */
			chan0_avg_old = chan0_avg / (1000/DATA_INTEGRATE_INTERVAL);
			power_avg_old = power_avg / (1000/DATA_INTEGRATE_INTERVAL);

			data_integrate_timer = sc_get_timer();

		}

		/* Process and send data every DATA_SEND_INTERVAL */ 
		if (sc_get_timer() >= data_send_timer + DATA_SEND_INTERVAL) {

			send_data();

			/* Look we're sending stuff */
			toggle_yellow_led();

			data_send_timer = sc_get_timer();
		}

		/* Save integrated values to flash every DATA_SAVE_INTERVAL */
		if (sc_get_timer() >= data_save_timer + DATA_SAVE_INTERVAL) {
			sc_user_eeprom_write_block(0, &chan0_integral, 8);					

			/* Current and voltage */
			scandal_send_channel(TELEM_LOW, 			 	/* priority */
								5,     	/* channel num */
								DATA_SAVE_INTERVAL		 	/* value */
		
			);
			
			data_save_timer = sc_get_timer();
		}
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
}

/* Calculates trapezoidal integral between two values and adds it to the integral */
void int_trapz(int64_t *integral, int64_t *cur_val, int64_t *prev_val, int64_t time)
{
	/* Values are in mA and time is in ms */
	int64_t increment = ((*cur_val + *prev_val)/2) * (1000/time);
	*integral = *integral + increment;
}
