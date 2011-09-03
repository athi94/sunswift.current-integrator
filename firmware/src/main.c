/***************************************************************************
 *   $Id:: i2c_main.c 4785 2010-09-03 22:39:27Z nxp21346                    $
 *   Project: NXP LPC11xx I2C example
 *
 *   Description:
 *     This file contains I2C test modules, main entry, to test I2C APIs.
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

#include <arch/timer32.h>
#include <arch/gpio.h>
#include <arch/uart.h>
#include <arch/type.h>
#include <arch/can.h>
#include <arch/ssp.h>
#include <arch/i2c.h>
#include <math.h>

#include <scandal/engine.h> /* for general scandal functions */
#include <scandal/message.h> /* for TELEM_LOW */

void setup_ports(void) 
{
	GPIOInit();

	// LEDS
	GPIOSetDir(2,6,1); //Green LED, Out
	GPIOSetDir(2,0,1); //Yel LED, Out

	// MCP3909 pins
	GPIOSetDir(2,PGA,1); // PGA, Out
	GPIOSetDir(2,nMCLR,1);// nMCLR, Out
	GPIOSetDir(0,MCP3909_CS,1)// MCP3909_CS, Out
}

void in_channel_0_handler(int32_t value, uint32_t src_time) {
	UART_printf("in_channel_0_handler got called with value %d time at source %d\n\r", value, src_time);
}

int main(void)
{
	int i = 0; /* Used in main loop */
	uint32_t value = 0xaa;

	setup_ports();
	scandal_init();

	UARTInit(115200);
	mcp3909_init();

	sc_time_t one_sec_timer = sc_get_timer(); /* Initialise the timer variable */
	sc_time_t test_in_timer = sc_get_timer(); /* Initialise the timer variable */

	/* Set LEDs to known states, i.e. on */
	red_led(1);
	yellow_led(1);

	scandal_delay(100); /* wait for the UART clocks to settle */

	/* Display welcome header over UART */
	UART_printf("Welcome to the template project! This is coming out over UART1\n\r");
	UART_printf("The 2 debug LEDs should blink at a rate of 1HZ\n\r");
	UART_printf("If you configure the in channel 0, I should print a message upon receipt of such a channel message\n\r");

	scandal_register_in_channel_handler(0, &in_channel_0_handler);

	/* This is the main loop, go for ever! */
	while (1) {
		/* This checks whether there are pending requests from CAN, and sends a heartbeat message.
		 * The heartbeat message encodes some data in the first 4 bytes of the CAN message, such as
		 * the number of errors and the version of scandal */
		handle_scandal();

		/* Send a UART message and flash an LED every second */
		if(sc_get_timer() >= one_sec_timer + 1000) {
			/* Send the message */
			UART_printf("This is the 1 second timer... %d\n\r", i++);

			/* Send a channel message with a blerg value at low priority on channel 0 */
			scandal_send_channel(TELEM_LOW, /* priority */
									0,      /* channel num */
									value    /* value */
			);

			/* Twiddle the LEDs */
			toggle_yellow_led();
			toggle_red_led();

			/* Update the timer */
			one_sec_timer = sc_get_timer();
		}

		if(scandal_get_in_channel_rcvd_time(TEMPLATE_TEST_IN) > test_in_timer) {

			value = scandal_get_in_channel_value(TEMPLATE_TEST_IN);

			UART_printf("I received a channel message in the main loop on in_channel 0, value  %d at time %d\n\r", value, scandal_get_in_channel_rcvd_time(TEMPLATE_TEST_IN));

			if(scandal_get_in_channel_value(TEMPLATE_TEST_IN) == 1) {
				toggle_red_led();
			} else {
				toggle_yellow_led();
			}

			test_in_timer = scandal_get_in_channel_rcvd_time(TEMPLATE_TEST_IN);
		}

	}
}
