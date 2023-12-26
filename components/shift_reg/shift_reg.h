#ifndef shift_reg
#define shift_reg
typedef enum {
    SET_SIPO_MASK,
    SET_SIPO_STATES,
    SEND_SIPO_STATES,
    SEND_PISO_STATES
} shift_task_action_t;

#define GENERIC_TASK_PRIO 1  // Any unspecified task
#define NUM_SIPO CONFIG_NUM_SIPO
#define NUM_PISO CONFIG_NUM_PISO
#define SHIFT_CLOCK_GPIO 15     // Pin 2 on all SN74HC165N and Pin 11 on all SN74HC595N
#define PISO_LOAD_GPIO   2     // Pin 1 on all SN74HC165N
#define PISO_DATA_GPIO   13     // Pin 9 on FIRST SN74HC165N
#define SIPO_LATCH_GPIO  14     // Pin 12 on all SN74HC595N
#define SIPO_DATA_GPIO   12     // Pin 14 on FIRST SN74HC595N

// Bit shifting macros for readability
#define bitRead(a,b) (!!((a) & (1ULL<<(b))))            // return valure of bit (b) in var (a)
#define bitWrite(a,b,x) ((a) = (a & ~(1ULL<<b))|(x<<b)) // write (x) to bit (b) in var (a)

// Function prototypes
void shift_init(void);
bool shift_read(uint8_t pin);
void shift_write(uint8_t pin, bool val);
void shift_show(void);

#endif