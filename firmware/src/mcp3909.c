#include <project/hardware.h>
#include <scandal/timer.h>
#include <scandal/led.h>

#include <project/driver_config.h>
#include <arch/gpio.h>
#include <arch/ssp.h>

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
  SSP_IOConfig(1);
  SSP_Init(1); 

  /* Apply a 1s reset pulse to nMCLR */ 
  GPIOSetValue(2,nMCLR,0);
  scandal_naive_delay(20);
  GPIOSetValue(2,nMCLR,1);

  GPIOSetValue(2,PGA,1);

  uint8_t mode = DUAL_PRE;
  SSP_Send(1,&mode,1);
  GPIOSetValue(0,MCP3909_CS,1);

  scandal_naive_delay(100); 

}


void mcp3909_sample(int16_t* chan0, int16_t* chan1){
  uint8_t bytes[4]; 

  GPIOSetValue(0,MCP3909_CS,0);

  LPC_IOCON->PIO2_2 &= ~0x07; //set P2.2 to GPIO mode, to read dataready from MCP3909
  while(GPIOGetValue(2, 2) == 0);
  while(GPIOGetValue(2, 2) != 0);
  LPC_IOCON->PIO2_2 |= 0x02; //set P2.2 to MISO mode, to read actual data from MCP3909
 
  scandal_naive_delay(100);
  SSP_Receive(1, bytes, 4);

  GPIOSetValue(0,MCP3909_CS,1);

  *chan1 = 
    ((int16_t)bytes[0] << 8 | 
     (int16_t)bytes[1]);
  *chan0 = 
    ((int16_t)bytes[2] << 8 | 
     (int16_t)bytes[3]);
}
