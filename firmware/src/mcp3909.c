#include "hardware.h"
#include "other_spi.h"
#include "scandal_timer.h"
#include "scandal_led.h"
#include "spi_devices.h"

#define MULT_OUT  0xA1
#define MULT_OUT2 0xA9
#define DUAL_PRE  0xA4
#define DUAL_POST 0xAC

void timer_delay(uint32_t ms){
  uint32_t timer; 
  uint32_t compare = 0; 
  timer = sc_get_timer();
 
  while(compare < timer)
    compare = sc_get_timer() - ms;
}


void delay(uint32_t count){
  volatile uint32_t i; 
  
  for(i=count; i>0; i--)
    ;
}

void mcp3909_init(void){
  init_spi0(); 

  /* Apply a 1s reset pulse to nMCLR */ 
  ASSERT_NMCLR();
  delay(500000); 
  DEASSERT_NMCLR(); 

  //yellow_led(1);
  
  //delay(50000);
  //yellow_led(0);

  //  delay(10); 

  /* Should be in dual channel transfer now */ 
  //yellow_led(1); 

  //ENABLE_MCP3909();    //Doesn't seem to be working. clock frequency also seems low
  

  //Put something in the buffer

  //delay(10);

  P1OUT &= ~MCP3909_CS;

  TXBUF0 = DUAL_PRE; 

  while((IFG1 & URXIFG0) == 0)
      ;
  P1OUT |= MCP3909_CS;

  //DISABLE_MCP3909(); 

  delay(10000); 

}


void mcp3909_sample(int16_t* chan0, int16_t* chan1){
  uint8_t bytes[4]; 

  /* Ok. Now we can wait for a DR pulse. */
  P3SEL &=~(SOMI0); 
  ENABLE_MCP3909(); 
  while((P3IN & SOMI0) == 0);
  while((P3IN & SOMI0) != 0);
  P3SEL |= SOMI0; 
 
  bytes[0] = spi0_transfer(0x00); 
  bytes[1] = spi0_transfer(0x00); 
  bytes[2] = spi0_transfer(0x00); 
  bytes[3] = spi0_transfer(0x00); 

  DISABLE_MCP3909(); 

  *chan1 = 
    ((int16_t)bytes[0] << 8 | 
     (int16_t)bytes[1]);
  *chan0 = 
    ((int16_t)bytes[2] << 8 | 
     (int16_t)bytes[3]); 
}
