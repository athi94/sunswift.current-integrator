/****************************************************************************
 * 
 * FM24CL64 Header
 * 
 ***************************************************************************/

#define BLOCK_SIZE 18

// Initialize values for FM24CL64 Functions
// when reset = TRUE the entire memory will be wiped & pointer reset
void memInit(int reset);

// Writes Data to FM24CL64
// startAddress is the first address to be written to
// writeLength is the number of elements in writebuf
// writebuf is the data to be written, Maximum size is 61 elements
void memWriteSeq(uint16_t startAddress, uint8_t writeLength, uint8_t *writebuf);

// Read Data from FM24CL64
// startAddress is the first address to be read from
// readLength is the number of elements to be read
// Function returns a pointer to an uint8_t array of 64 elements
uint8_t * memRead(uint16_t startAddress, uint8_t readLength);

// Read FM24CL64 Pointer
uint16_t memGetPointer();

// Make new block and write at next slot
void memWriteBlock (uint8_t *data, uint8_t length);

// Read a block, blockNumber starts from 0
uint8_t *memReadBlock (uint8_t blockNumber);

// Retrieve chan0_int
uint64_t memGetChan0_int(uint8_t *block);

// Retrieve power_int
uint64_t memGetPower_int(uint8_t *block);

// Retrieve CRC
uint16_t memGetCRC(uint8_t *block);