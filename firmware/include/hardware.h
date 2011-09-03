/* Hardware definitions */

#define CLOCK_SPEED     4000000

/* Port 0 */
#define MCP3909_CS		8

/* Port 1 */
#define TEMP_EN			9

/* Port 2 */
#define PGA           	7
#define nMCLR           8

/* ADC channel definitions */
#define	TEMP_MEAS		5
#define	12V_MEAS		1

/* Useful macros */ 
#define ASSERT_NMCLR()   GPIOSetValue(2,nMCLR,0);
#define DEASSERT_NMCLR() GPIOSetValue(2,nMCLR,1);


