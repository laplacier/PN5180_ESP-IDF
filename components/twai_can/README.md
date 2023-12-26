| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- |

# twai_can component
Utilizes the built in twai_can functions of the ESP32 CAN controller to deliver beneficial information about the props in the network and full control of the props to the operator and technician.

## CAN Messages

Messages sent on the CAN network follow the ISO11898-1 standard implementation. **Table 1** shows the data of significance contained in each standard CAN frame when analyzing traffic on the CAN network

**Table 1. CAN Message Fields**
| Field Start | Field End | Value | Description      |
|-------------|-----------|:-----:|------------------|
| ID10        | ID8       | 0-7   | Message Priority |
| ID7         | ID0       | 0-255 | Sender ID        |
| DLC3        | DLC0      | 0-8   | Data Length      |
| DATA7       | DATA0     | 0-255 | Target ID        |
| DATA15      | DATA8     | 0-255 | Command          |
| DATA23      | DATA16    | 0-255 | Command Payload  |
| DATA31      | DATA24    | 0-255 | Command Payload  |
| DATA39      | DATA32    | 0-255 | Command Payload  |
| DATA47      | DATA40    | 0-255 | Command Payload  |
| DATA55      | DATA48    | 0-255 | Command Payload  |
| DATA63      | DATA56    | 0-255 | Command Payload  |


### Message Priority
Identifies the significance and type of the message. The lower the value, the higher the priority. Messages with a higher priority will send first when sent at the same time as a lower priority message. Lower priority messages will be tried again after the higher priority message is finished sending.

**Table 2. Priority Information**
| Value |               Description              |
|------:|----------------------------------------|
|   0   | [Write command to ALL props](#command) |
|   1   | [Write command to ONE prop](#command)  |
|   2   | [Read command](#command)               |
|   3   | Unused                                 |
|   4   | [Inheritance](#prop-inheritance)       |
|   5   | [Ping to ALL props](#ping-requests)    |
|   6   | [Ping response](#ping-requests)        |
|   7   | [Ping to ONE prop](#ping-requests)     |

### Sender ID
The ID of the prop that generated the message. For ping transactions, the Sender ID of the ping request becomes the the Target ID of the ping response.

### Data Length
The number of bytes included in the CAN data frame. A minimum of 1 byte will be sent to include the Target ID in a ping request. For a Command, the command byte follows the Target ID and the next 0-6 bytes are the Command Payload.

### Target ID
The ID of the prop the message is intended for. All props will see the message regardless of which prop the message is intended for and can choose to keep a record or ignore the message.

### Command
The instruction to be carried out by the target prop. **Table 3** describes the commands contained in a regular command. For ping requests, see [Ping requests](#ping-requests).

**Table 3. Issuable Commands**
| Value |         Component         |                          Description                           |
| ----: |---------------------------|----------------------------------------------------------------|
| 0     | [puzzle](../puzzle)       | [GAME_STATE](../puzzle/README.md#set-game-state)               |
| 1     | [GPIO](../gpio_prop)      | [GPIO_MASK](../gpio_prop/README.md#set-gpio-mask)              |
| 2     | [GPIO](../gpio_prop)      | [GPIO_STATE](../gpio_prop/README.md#set-gpio-state)            |
| 3     | [sound](../sound)         | [SOUND](../sound/README.md#set-sound)                          |
| 4     | [shift_reg](../shift_reg) | [SHIFT_MASK](../shift_reg/README.md#set-shift-register-mask)   |
| 5     | [shift_reg](../shift_reg) | [SHIFT_STATE](../shift_reg/README.md#set-shift-register-state) |
| 6     | [nfc](../nfc)             | [NFC_SOF](../nfc/README.md#write-nfc-tag)                      |
| 7     | [nfc](../nfc)             | [NFC_DATA](../nfc/README.md#write-nfc-tag)                     |
| 8     | [nfc](../nfc)             | [NFC_EOF](../nfc/README.md#write-nfc-tag)                      |

### Command Payload
Additional instructions as required by the prop to accompany the command. Click on a command in **Table 3** to navigate to the payload description and options for each command.

## Ping requests
During operation, a ping request may be received to confirm existence or state. **Table 4** describes the commands contained in a ping request. A ping request contains no payload.

**Table 4. Ping Request Commands**
| Value | Description         |
|-------|---------------------|
| 0     | Send empty response |
| 1     | Send ALL states     |

## How to configure
This component can be configured by opening the ESP-IDF Configuration editor and navigating to "Prop CAN Options".

**Table 4. Configuration Options**
|                 Setting                 | Type | Default                |
|-----------------------------------------|------|------------------------|
| [Enable CAN](#enable-can)               | bool | True                   |
| [CAN_TX Pin](#can_tx-pin)               | int  | 33                     |
| [CAN_RX Pin](#can_rx-pin)               | int  | 32                     |
| [Prop ID](#prop-id)                     | int  | 1                      |
| [Operation Mode](#operation-mode)       | enum | "Receive and Transmit" |
| [Enable Inheritance](#prop-inheritance) | bool | false                  |

### Enable CAN
Allow the prop to use CAN, which is the primary method for communicating between props. If CAN is disabled, the prop's default game stage will be 1 (Active) instead of 0 (Idle/Disabled) to prevent the prop from being unable to activate. You will need to be sure to develop your own method to reset the puzzle upon completion if CAN is disabled.

### CAN_TX Pin
Can be any unused IO capable pin.

### CAN_RX Pin
Can be any unused IO capable pin.

### Prop ID
0 is reserved for the game flow controller. Can assign any ID from 1 to 255. Each prop must use a unique ID to prevent conflicts on the CAN bus. The lowest ID on the CAN bus will have the highest priority when sending the same type of message.

### Operation mode   
Receive and Transmit - Normal operation. Can send and receive messages on the CAN bus.
    
Listen only - Can receive messages on the CAN bus and ACK them, but will not send messages or respond to message requests.

### Prop inheritance
*CURRENTLY UNIMPLEMENTED*. Allows the prop to participate in prop inheritance. When a device is sent a ping request multiple times without a ping reponse, any prop with this setting enabled will attempt to request to inherit the missing prop. If the request is successful, the prop will "inherit" the send/receive responsibilities of the missing prop and mimic their ID. This is useful for props that unexpectedly fail or disconnect from the CAN bus during a game to allow smooth, continuous operation. If the inherited prop returns and sends a ping reponse, the prop will no longer be mimic'd.

## API reference
The following functions are included to utilize this component:

### bool CAN_Receive(uint32_t delay) note: transfer to puzzle README
Checks a queue which populates with messages sent to the prop ID for "delay" milliseconds converted from nonblocking FreeRTOS task ticks. The queue can contain an integer value which corresponds to the commands listed in **Table 3**.

Each message is handled automatically by the function and transferred to its respective component. For option 1, change state, the global game state variable will be modified to the requested value. The twai_can components will be blocked from sending additional commands until the puzzle component calls and resolves the command in this function. If **rx_payload[]** is busy being written to by the CAN component, this function will be blocked from executing and return false.

### bool CAN_Send(uint8_t target_id, uint8_t command)
Sends a message on the CAN bus with the inputted [command](#command) to the target_id of the prop. The same messages that can be received are the messages that can be sent. However, a payload corresponding to the command must be sent with the command. The payload must be written to the global variable **tx_payload[]** before calling this function. If **tx_payload[]** is busy being read by the CAN component, this function will be blocked from executing and return false.

```
// Set game state
tx_payload[1] = (0x1 << 4) | 0x1; // Write | Length = 1
tx_payload[2] = 1;                // Set game state to active/ready state
CAN_Send(GAME_STATE,1);           // Set game state of prop with ID 1

// Send GPIO output mask
tx_payload[1] = (0x0 << 4) | 0x5; // Read | Length = 5
tx_payload[2] = 66;               // (0b01000011) Modify GPIO 0, 1, and 6
tx_payload[3] = 12;               // (0b00001100) Modify GPIO 10, 11
tx_payload[4] = 0;                // (0b00000000)
tx_payload[5] = 64;               // (0b01000000) Modify GPIO 30
tx_payload[6] = 0;                // (0b00000000)
CAN_Send(1,GPIO_MASK);            // Send GPIO mask to prop with ID 1

// Send GPIO output state
tx_payload[1] = (0x0 << 4) | 0x5; // Read | Length = 5
tx_payload[2] = 255;              // (0b11111111) All GPIO HIGH (only 0,1,6 set)
tx_payload[3] = 0;                // (0b00000000) All GPIO LOW  (only 10,11 set)
tx_payload[4] = 0;                // (0b00000000) All GPIO LOW  (none set)
tx_payload[5] = 0;                // (0b00000000) All GPIO LOW  (only 30 set)
tx_payload[6] = 255;              // (0b11111111) All GPIO HIGH (none set)
CAN_Send(2,GPIO_STATE);           // Send GPIO states to prop with ID 2

// Set/Play music track
tx_payload[1] = (0x1 << 4) | 0x1; // Write | Length = 1
tx_payload[2] = 3;                // Select sound track 3
CAN_Send(1,MUSIC);                // Set/Play sound track to play to prop with ID 1

// Set shift_reg output mask
tx_payload[1] = (0x1 << 4) | 0x5; // Write | Length = 5
tx_payload[2] = 66;               // (0b01000011) Modify PISO 0 pins A, B, and G
tx_payload[3] = 12;               // (0b00001100) Modify PISO 1 pins C and D
tx_payload[4] = 0;                // (0b00000000)
tx_payload[5] = 64;               // (0b01000000) Modify PISO 3 pin G
tx_payload[6] = 0;                // (0b00000000)
CAN_Send(1,SHIFT_MASK);           // Set shift_reg mask of prop with ID 1

// Set shift_reg output state
tx_payload[1] = (0x1 << 4) | 0x5; // Write | Length = 5
tx_payload[2] = 255;              // (0b11111111) Set all PISO 0 pins HIGH (only A,B,G set)
tx_payload[3] = 0;                // (0b00000000) Set all PISO 1 pins LOW  (only C,D set)
tx_payload[4] = 0;                // (0b00000000) Set all PISO 2 pins LOW  (none set)
tx_payload[5] = 0;                // (0b00000000) Set all PISO 3 pins LOW  (only G set)
tx_payload[6] = 255;              // (0b11111111) Set all PISO 4 pins HIGH (none set)
CAN_Send_Command(1,SHIFT_STATE);  // Set shift_reg state of prop with ID 1
```