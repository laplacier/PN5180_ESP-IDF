#ifndef twai_can
#define twai_can

// CAN message prio and type
#define CMD_WRITE_ALL 0
#define CMD_WRITE_ONE 1
#define CMD_READ      2
#define INHERIT       4
#define PING_REQ_ALL  5
#define PING_RESP     6
#define PING_REQ_ONE  7

// rx_payload[0] contains the following flags
#define FLAG_WRITE 0x10 // Command is write
#define FLAG_PING  0x20 // Ping response must contain all states
#define FLAG_RES   0x40 // Control_Task requires ctrl_done_sem from component
#define FLAG_CONT  0x80 // Component has additional data to send

// GPIO Pin assignments
#define CAN_TX_GPIO                     32 // ESP32 Tx Pin to CAN Pin
#define CAN_RX_GPIO                     33 // ESP32 Rx Pin to CAN Pin

// FreeRTOS task priorities
#define CTRL_TASK_PRIO                  10 // Control_Task
#define RX_TASK_PRIO                    9  // Rx_Task
#define TX_TASK_PRIO                    8  // Tx_Task

#define ERR_DELAY_US                    800     //Approximate time for arbitration phase at 25KBPS
#define ERR_PERIOD_US                   80      //Approximate time for two bits at 25KBPS

#define ID_PROP                         0x01 // 8-bit Prop ID, combined with 3-bit priority for full CAN ID

typedef enum {
    CTRL_HELLO,
    CTRL_PING,
    CTRL_CMD,
    CTRL_SEND_PUZZLE,
    CTRL_SEND_GPIO,
    CTRL_SEND_SOUND,
    CTRL_SEND_SHIFT_SIPO,
    CTRL_SEND_SHIFT_PISO,
    CTRL_SEND_NFC
} ctrl_task_action_t;

typedef enum {
    TX_PING,
    TX_DATA,
    TX_DND,
    TX_INHERIT,
    INHERIT_PASS
} tx_task_action_t;

typedef enum {
    RX_WRITE_ALL,
    RX_WRITE,
    RX_READ,
    RX_UNUSED,
    RX_INHERIT,
    RX_PING_REQ_ALL,
    RX_PING_RES,
    RX_PING_REQ
} rx_task_action_t;

typedef enum {
    GAME_STATE,
    GPIO_MASK,
    GPIO_STATE,
    SOUND,
    SHIFT_SIPO_MASK,
    SHIFT_SIPO_STATE,
    SHIFT_PISO_STATE,
    NFC_SOF,
    NFC_DATA,
    NFC_EOF
} can_command_t;

void fake_bus_task(void *arg);
void twai_can_init(void);

#endif