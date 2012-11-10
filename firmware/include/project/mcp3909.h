#define MCP3909_SUCCESS 1
#define MCP3909_FAIL 0
void mcp3909_init(void);
int mcp3909_sample(int16_t* chan0, int16_t* chan1); 
