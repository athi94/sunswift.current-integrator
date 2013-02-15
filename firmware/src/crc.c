#include <project/driver_config.h>
#include <project/crc.h>

#define WIDTH  (8 * sizeof(crc))
#define TOPBIT (1 << (WIDTH - 1))

crc crcCompute (uint8_t *data, uint8_t nBytes) {
    crc remainder = 0;
    
    uint8_t byte;
    for (byte = 0; byte < nBytes; byte++) {
        // Bring the next byte into the remainder.
        remainder ^= (data[byte] << (WIDTH - 8));
        
        // Perform modulo-2 division, a bit at a time.
        uint8_t bit;
        for (bit = 8; bit > 0; bit--) {
            // Try to divide the current data bit.
            if (remainder & TOPBIT) {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            } else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder;
}
