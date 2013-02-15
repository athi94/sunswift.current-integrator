typedef uint16_t crc;
#define POLYNOMIAL 0x8005;

// Takes an array of 8-bit ints, nBytes is how long
crc crcCompute (uint8_t *data, uint8_t nBytes);