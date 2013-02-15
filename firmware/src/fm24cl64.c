/****************************************************************************
 *   Author: Tong Wu
 *   
 *   Description:
 *   An addon to existing Current Integrator Code
 *   Adds Interfacing for LPC - FM24CL64 Communications
 * 
 ***************************************************************************/

#include <project/driver_config.h>
#include <project/fm24cl64.h>
#include <project/ansi_color.h>
#include <project/crc.h>

#include <scandal/stdio.h>

#include <arch/gpio.h>
#include <arch/i2c.h>

#define MEM_RANGE 0x1FFF // 8192 Addresses
#define I2C_MEM_ADDR 0xA0
#define I2C_MEM_READ_BIT 1
#define I2C_MEM_WRITE_BIT 0

// I2C Variables
extern volatile uint32_t I2CCount;
extern volatile uint8_t I2CMasterBuffer[BUFSIZE];
extern volatile uint8_t I2CSlaveBuffer[BUFSIZE];
extern volatile uint32_t I2CMasterState;
extern volatile uint32_t I2CReadLength, I2CWriteLength;

// Helper Function Prototypes
void memReset();
void memWriteProtect(int onOff);
void memWrite(uint16_t startAddress, uint8_t writeLength, uint8_t *writebuf);
void memClearSlaveBuffer();

void memInit(int reset) {
    // Initialize values for FM24CL64 Functions
    
    UART_printf("Starting memInit...\r\n");
    
    // Remove Write Protect
    UART_printf("Lifting Write Protect...\r\n");
    GPIO_SetDir(0,3,1);
    
    // Init I2C
    UART_printf("I2C Init Success: %d\r\n", (int)I2CInit((uint32_t)I2CMASTER));
    
    // Reset Memory if required
    if (reset) {
        memReset();
    }
}

void memReset() {
    // Total Memory Wipe & Pointer Reset
    uint8_t writebuf[BUFSIZE] = {0x00};
    
    UART_printf("Starting memReset...\r\n");
    
    // Setup I2C
    int i;
    for (i = 0; i < ((1 + MEM_RANGE)/32); i++) {
        memWrite(i*32, 32, writebuf);
    }
    
    // Reset Counter
    writebuf[0] = 0x00;
    writebuf[1] = 0x02;
    memWrite(0x00, 2, writebuf);
}

void memWriteProtect(int onOff) {
    // onOff controls WP On or Off
    // On = 1, Off = 0
    
    if (onOff) {
        UART_printf("memWriteProtect: On\r\n");
        GPIO_SetValue(0,3,1);
    } else {
        UART_printf("memWriteProtect: Off\r\n");
        GPIO_SetValue(0,3,0);
    }
}

void memWrite(uint16_t startAddress, uint8_t writeLength, uint8_t *writebuf) {
    // Writes Data to FM24CL64
    // startAddress is the first address to be written to
    // writeLength is the number of elements in writebuf
    // writebuf is the data to be written, Maximum size is 61 elements
    
    UART_printf("Starting memWrite...\r\n");
    UART_printf(ANSI_BLUE"[ Address: %d ] [ WriteLength: %d ]\r\n"ANSI_RESET, (int)startAddress, (int)writeLength);
    // Setup I2C For Write Operation
    I2CWriteLength = 3 + writeLength;
    I2CReadLength = 0;
    
    // Send FM24CL64 I2C Address
    I2CMasterBuffer[0] = I2C_MEM_ADDR | I2C_MEM_WRITE_BIT;
    
    // Send Mem Address
    I2CMasterBuffer[1] = startAddress>>8;
    I2CMasterBuffer[2] = startAddress;
    
    // Send Data
    UART_printf("Buffering\r\n[ ");
    uint8_t i;
    for (i = 0; i < writeLength; i++) {
        UART_printf("%d ", i);
        I2CMasterBuffer[3 + i] = writebuf[i];
    }
    UART_printf("]\r\n");
    
    // Write
    memWriteProtect(0);
    UART_printf("Write Success: %d \r\n", (int)I2CEngine());
    memWriteProtect(1);
}

uint8_t * memRead(uint16_t startAddress, uint8_t readLength) {
    // Read Data from FM24CL64
    // startAddress is the first address to be read from
    // readLength is the number of elements to be read
    // Function returns a pointer to an uint8_t array of 64 elements
    
    // Clear Slave Buffer
    memClearSlaveBuffer();
    
    // Setup I2C For Read Operation
    I2CWriteLength = 3;
    I2CReadLength = readLength;
    
    // Send FM24CL64 I2C Address
    I2CMasterBuffer[0] = I2C_MEM_ADDR | I2C_MEM_WRITE_BIT;
    
    // Send Mem Address
    I2CMasterBuffer[1] = (startAddress>>8);
    I2CMasterBuffer[2] = startAddress;
    
    I2CMasterBuffer[3] = I2C_MEM_ADDR | I2C_MEM_READ_BIT;
    
    UART_printf("I2C Read Success: %d \r\n", (int)I2CEngine());
    
    //uint8_t i;
    //uint8_t returnArray[readLength];
    //for (i = 0; i < readLength; i++) {
    //    returnArray[i] = I2CSlaveBuffer[i];
    //}
    
    return I2CSlaveBuffer;
}

void memClearSlaveBuffer() {
    // Wipes I2CSlaveBuffer
    
    UART_printf("Starting memClearSlaveBuffer...\r\n");
    uint8_t i;
    for (i = 0; i < BUFSIZE; i++) {
        I2CSlaveBuffer[i] = 0x00;
    }
}

uint16_t memGetPointer() {
    // Read FM24CL64 Pointer
    
    uint16_t pointer;
    memRead(0x00, 2);
    pointer = I2CSlaveBuffer[0];
    pointer = (pointer<<8) | I2CSlaveBuffer[1];
    return pointer;
}

void memWriteSeq(uint16_t startAddress, uint8_t writeLength, uint8_t *writebuf) {
    
    // Write & Increment Pointer
    uint8_t newPointer[2];
    if (startAddress + writeLength < 2048) {
        memWrite(startAddress, writeLength, writebuf);
        UART_printf("Incrementing Pointer: %d + %d\r\n", startAddress, writeLength);
        newPointer[0] = ((startAddress + writeLength)>>8);
        newPointer[1] = (startAddress + writeLength);
        memWrite(0x00, 2, newPointer);
    } else {
        UART_printf("Resetting Pointer\r\n");
        
        startAddress = 0x0002;
        
        newPointer[0] = 0x00;
        newPointer[1] = 0x02;
        memWrite(0x0000, 2, newPointer);
        memWrite(startAddress, writeLength, writebuf);
        
        UART_printf("Incrementing Pointer: %d + %d\r\n", startAddress, writeLength);
        newPointer[0] = ((startAddress + writeLength)>>8);
        newPointer[1] = (startAddress + writeLength);
        memWrite(0x00, 2, newPointer);
    }
}

// Wrappers
void memWriteBlock (uint8_t *data, uint8_t length) {
    
    
    /************************************************************************
     *   |  chan0_int |  power_int |  crcString |                           *
     *   |  uint64_t  |  uint64_t  |  uint16_t  |                           *
     ***********************************************************************/
    
    // CRC data
    uint16_t crcString;
    crcString = crcCompute(data, length);
    
    UART_printf(ANSI_BLUE"CRCPre = %d\r\n"ANSI_RESET, (int)crcString);
    
    uint8_t block[BLOCK_SIZE];
    
    // Add Data
    int i;
    for (i = 0; i < length; i++) {
        block[i] = data[i];
    }
    
    // Add CRC
    block[i] = (crcString>>8);
    block[i + 1] = crcString;
    
    
    memWriteSeq(memGetPointer(), BLOCK_SIZE, block);
}

uint8_t *memReadBlock (uint8_t blockNumber) {
    uint16_t startAddress = (blockNumber * BLOCK_SIZE) + 2;
    return memRead(startAddress, BLOCK_SIZE);
}

uint64_t memGetChan0_int(uint8_t *block) {
    // Retrieve chan0_int
    return ((uint64_t *)block)[0];
}

uint64_t memGetPower_int(uint8_t *block) {
    // Retrieve power_int
    return ((uint64_t *)block)[1];
}

uint16_t memGetCRC(uint8_t *block) {
    // Retrieve CRC
    uint16_t CRC = 0;
    CRC = block[16];
    CRC = (CRC<<8) | block[17];
    
    return CRC;
}
