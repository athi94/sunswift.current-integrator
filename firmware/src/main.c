/***************************************************************************
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

int main(void)
{
	int i = 0; /* Used in main loop */
	uint16_t chan0 = 0x123;
	uint16_t chan1 = 0x123;
	uint32_t value = 0x0;

	setup_ports();
	scandal_init();

	mcp3909_init();
	UARTInit(115200);

	sc_time_t one_sec_timer = sc_get_timer(); /* Initialise the timer variable */
	sc_time_t test_in_timer = sc_get_timer(); /* Initialise the timer variable */

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

		mcp3909_sample(&chan0, &chan1);

		/* Send a UART message and flash an LED every second */
		if(sc_get_timer() >= one_sec_timer + 1000) {
			/* Send the message */
			UART_printf("This is the 1 second timer... %d\n\r", i++);
			UART_printf("chan 0: %d chan 1: %d chan 0: 0x%x chan 1: 0x%x\n\r", chan0, chan1, chan0, chan1);
			/* Send a channel message with a ADC value at low priority on channel 0 */

			scandal_send_channel(TELEM_LOW, /* priority */
									1,      /* channel num */
									(uint32_t)chan0    /* value */
			);
			scandal_send_channel(TELEM_LOW, /* priority */
									2,      /* channel num */
									(uint32_t)chan1    /* value */
			);

			/* Twiddle the LEDs */
			toggle_red_led();

			/* Update the timer */
			one_sec_timer = sc_get_timer();
		}
	}
}
