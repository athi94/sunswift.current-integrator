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
int64_t chan0_acc_old = 0, chan1_acc_old =0;
int64_t integral = 0;
int64_t samples = 0;

/*---------------------------------------------------------------------------*/

/* Helper function defs */
void prepare_data_for_sending();
void send_data();
void integrate(int64_t *integral, int64_t *cur_value, int64_t *prev_value, int64_t time_interval);
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

	sc_time_t one_sec_timer = sc_get_timer(); /* Initialise the timer variable */
	sc_time_t test_in_timer = sc_get_timer(); /* Initialise the timer variable */
	sc_time_t data_send_timer = sc_get_timer();
	sc_time_t data_save_timer = sc_get_timer();

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

		/* Get current and voltage samples. Sampling happens at the MCU clock frequency and the
		 * values are later averaged over the DATA_SEND_INTERVAL */
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

		/* Process and send data every DATA_SEND_INTERVAL */ 
		if (sc_get_timer() >= data_send_timer + DATA_SEND_INTERVAL) {
		
			prepare_data_for_sending();
			send_data();

			/* Look we're doing stuff */
			toggle_yellow_led();

			/* Reset accumulated values, sample counter and timer */
			chan0_acc_old = chan0_acc;
			chan1_acc_old = chan1_acc;
			chan0_acc = 0;
			chan1_acc = 0;
			samples = 0;
			data_send_timer = sc_get_timer();
		}

		/* Save integrated values to flash every DATA_SAVE_INTERVAL */
		if (sc_get_timer() >= data_save_timer + DATA_SAVE_INTERVAL) {
			/*Stuff*/		
		}

	}
}

void prepare_data_for_sending()
{

	/* Deal with CURRENT and VOLTAGE channels first */
	/* Get average values for current and voltage*/
		
	scandal_get_scaleaverage64(CURRENTINT_CURRENT, &chan0_acc, &samples);
	scandal_get_scaleaverage64(CURRENTINT_VOLTAGE, &chan1_acc, &samples);

	/* Get updated integrated value for current and voltage */
	integrate(&integral, &chan0_acc, &chan0_acc_old, DATA_SEND_INTERVAL);
	
	/* Deal with POWER channel */

	/* Deal with TEMPERATURE channel */
}

void send_data()
{			
	scandal_send_channel(TELEM_LOW, /* priority */
						CURRENTINT_CURRENT,      /* channel num */
						chan0_acc    /* value */
	);

	scandal_send_channel(TELEM_LOW, /* priority */
						CURRENTINT_VOLTAGE,      /* channel num */
						chan1    /* value */
	);
	
	scandal_send_channel(TELEM_LOW, /* priority */
						2,      /* channel num */
						integral/3600000    /* value */
	);
}


void integrate(int64_t *integral, int64_t *cur_value, int64_t *prev_value, int64_t time_interval)
{
	int64_t increment = (*cur_value + *prev_value) * time_interval;
	*integral = *integral + (increment << 1);
}
