#ifndef sound
#define sound

#define UART_TX_GPIO 17
#define UART_RX_GPIO 16
#define UART UART_NUM_2
#define GENERIC_TASK_PRIO 1  // Any unspecified task

void sound_init(void);
#endif
