#ifndef prop_gpio
#define prop_gpio
typedef enum {
    SET_GPIO_MASK,
    SET_GPIO_STATES,
    SEND_GPIO_STATES
} gpio_task_action_t;

typedef enum {
    INPUT,
    INPUT_PULLUP,
    INPUT_PULLDOWN,
    INPUT_INTERRUPT,
    OUTPUT
} gpio_mode_wrapper_t;

typedef enum {
    RISING,
    FALLING,
    ANY
} gpio_interrupt_t;

// Bit shifting macros for readability
#define bitRead(a,b) (!!((a) & (1ULL<<(b))))            // return valure of bit (b) in var (a)
#define bitWrite(a,b,x) ((a) = (a & ~(1ULL<<b))|(x<<b)) // write (x) to bit (b) in var (a)
#define READBYTE(a, b)    (((a) >> ((b) * 8)) & 0xFF)   // read byte (b) in var (a)
#define BYTESHIFTL(a, b)    (((a) <<= ((b) * 8)))       // shift all bits of var (a) to the left by (b) bytes

// In case ON and OFF is preferred in pin r/w functions
#define ON 1
#define OFF 0

// Function prototypes
void gpio_task(void *arg);
void GPIO_Init(void);
void gpio_mode(uint8_t pin, gpio_mode_wrapper_t mode, gpio_interrupt_t type);
bool gpio_read(uint8_t pin);
esp_err_t gpio_write(uint8_t pin, bool level);

#endif