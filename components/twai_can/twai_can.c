#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "soc/gpio_sig_map.h" // For TWAI_TX_IDX
#include "twai_can.h"
#include "puzzle.h"
#include "sound.h"
#include "shift_reg.h"
#include "gpio_prop.h"
#include "iso15693.h"
#include "pn5180.h"

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NO_ACK);
//static const twai_message_t ping_resp = {.identifier = 0b11000000000 + ID_PROP, .data_length_code = 0,
//                                         .data = {0,0,0,0,0,0,0,0}, .self = 1};
static void ctrl_task(void *arg);
static void ctrl_router();
static void tx_task(void *arg);
static void rx_task(void *arg);

QueueHandle_t ctrl_task_queue;
QueueHandle_t tx_task_queue;
SemaphoreHandle_t ctrl_task_sem;
SemaphoreHandle_t ctrl_done_sem;
SemaphoreHandle_t rx_task_sem;
SemaphoreHandle_t rx_payload_sem;
SemaphoreHandle_t tx_task_sem;
SemaphoreHandle_t tx_payload_sem;

// sound
extern TaskHandle_t sound_task_handle;

// gpio
extern QueueHandle_t gpio_task_queue;
extern SemaphoreHandle_t gpio_task_sem;

// shift_reg
extern uint8_t dataIn[NUM_PISO]; // Data read from PISO registers
extern QueueHandle_t shift_task_queue;
extern SemaphoreHandle_t shift_task_sem;

// nfc
extern ISO15693NFC_t nfc; // Struct holding data from nfc tags
extern QueueHandle_t nfc_task_queue;
extern SemaphoreHandle_t nfc_task_sem;
extern SemaphoreHandle_t nfc_cont_sem;

// puzzle
extern SemaphoreHandle_t puzzle_task_sem;
extern QueueHandle_t puzzle_task_queue;

uint8_t rx_payload[9];
uint8_t tx_payload[9];

void twai_can_init(void){
  tx_task_queue = xQueueCreate(1, sizeof(tx_task_action_t));
  ctrl_task_queue = xQueueCreate(1, sizeof(ctrl_task_action_t));
  ctrl_task_sem = xSemaphoreCreateCounting( 10, 0 );
  ctrl_done_sem = xSemaphoreCreateBinary();
  rx_task_sem = xSemaphoreCreateBinary();
  tx_task_sem = xSemaphoreCreateCounting( 10, 0 );
  rx_payload_sem = xSemaphoreCreateBinary(); // Allows rx_task to write to the rx data payload
  tx_payload_sem = xSemaphoreCreateBinary(); // Allows puzzle_task to write the tx data payload

  xTaskCreatePinnedToCore(tx_task, "CAN_Tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(ctrl_task, "CAN_Controller", 4096, NULL, CTRL_TASK_PRIO, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(rx_task, "CAN_Rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(fake_bus_task, "CAN_Fake_Bus_Task", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);

  //Install TWAI driver, trigger tasks to start
  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
  ESP_LOGI("TWAI_CAN", "Driver installed");
  ESP_ERROR_CHECK(twai_start());
  ESP_LOGI("TWAI_CAN", "Driver started");

  ctrl_task_action_t ctrl_action = CTRL_HELLO;
  xQueueSend(ctrl_task_queue, &ctrl_action, portMAX_DELAY); // Send BEGIN job to control task
  xSemaphoreGive(ctrl_task_sem);                            // Unblock control task
  ESP_LOGI("TWAI_CAN", "Setup complete");
}

static void ctrl_task(void *arg){
  ctrl_task_action_t ctrl_action;
  tx_task_action_t tx_action;
  can_command_t command;
  static const char* TAG = "CAN_Controller";
  for(;;){
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY); // Blocked from executing until a task gives a semaphore
    xQueueReceive(ctrl_task_queue, &ctrl_action, pdMS_TO_TICKS(10)); // Pull task from queue
    switch(ctrl_action){
      case CTRL_HELLO:
        xSemaphoreGive(rx_payload_sem); // Give control of rx_payload to rx_task
        xSemaphoreGive(rx_task_sem); // Allow rx_task to begin receiving CAN messages
        tx_action = TX_PING;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xSemaphoreGive(tx_task_sem);
        ESP_LOGI(TAG, "Hello! Sent ping task to TX");
        break;
      case CTRL_PING:
        xSemaphoreTake(tx_payload_sem, portMAX_DELAY); // Blocked until tx_task is no longer reading tx_payload
        for(int i=0; i<sizeof(rx_payload); i++){
          tx_payload[i] = rx_payload[i];
        }
        xSemaphoreGive(rx_payload_sem); // Give control of rx_payload to rx_task
        if(!(tx_payload[0] & FLAG_PING)){ // If no ping flag, send empty response
          tx_action = TX_PING;
          xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
          xSemaphoreGive(tx_task_sem);
          ESP_LOGI(TAG, "Sent ping task to TX");
        }
        else{ // If ping flag, we need to send every state
          tx_action = TX_DATA;
          tx_payload[0] &= 0b00001111; // Clear flags, read mode
          tx_payload[0] |= FLAG_RES;   // Receive semaphore from component

          // puzzle
          tx_payload[2] = (uint8_t)GAME_STATE;
          ctrl_router();

          // gpio
          xSemaphoreTake(tx_payload_sem, portMAX_DELAY); // Blocked until tx_task is no longer reading tx_payload
          tx_payload[2] = (uint8_t)GPIO_STATE;
          ctrl_router();

          // shift_reg
          xSemaphoreTake(tx_payload_sem, portMAX_DELAY); // Blocked until tx_task is no longer reading tx_payload
          tx_payload[2] = (uint8_t)SHIFT_SIPO_STATE;
          ctrl_router();

          xSemaphoreTake(tx_payload_sem, portMAX_DELAY); // Blocked until tx_task is no longer reading tx_payload
          tx_payload[2] = (uint8_t)SHIFT_PISO_STATE;
          ctrl_router();

          // nfc
          xSemaphoreTake(tx_payload_sem, portMAX_DELAY); // Blocked until tx_task is no longer reading tx_payload
          tx_payload[2] = (uint8_t)NFC_SOF;
          ctrl_router();
        }
        break;
      case CTRL_CMD:
        xSemaphoreTake(tx_payload_sem, portMAX_DELAY); // Blocked until tx_task is no longer reading tx_payload
        for(int i=0; i<sizeof(rx_payload); i++){
          tx_payload[i] = rx_payload[i];
        }
        xSemaphoreGive(rx_payload_sem); // Give control of rx_payload to rx_task
        ctrl_router();
        break;
      case CTRL_SEND_PUZZLE: // Puzzle component sending state
        tx_payload[0] = 0x02; // No flags, length = 2
        tx_payload[1] = 0x00; // Send to master ID
        command = GAME_STATE;
        tx_payload[2] = (uint8_t)command;
        ctrl_router();
        break;
      case CTRL_SEND_GPIO: // GPIO component sending state
        tx_payload[0] = 0x02; // No flags, length = 2
        tx_payload[1] = 0x00; // Send to master ID
        command = GPIO_STATE;
        tx_payload[2] = (uint8_t)command;
        ctrl_router();
        break;
      case CTRL_SEND_SOUND: // Sound component never sends
        break;
      case CTRL_SEND_SHIFT_SIPO: // Shift_reg component sending SIPO state
        tx_payload[0] = 0x02; // No flags, length = 2
        tx_payload[1] = 0x00; // Send to master ID
        command = SHIFT_SIPO_STATE;
        tx_payload[2] = (uint8_t)command;
        ctrl_router();
        break;
      case CTRL_SEND_SHIFT_PISO: // Shift_reg component sending PISO state
        tx_payload[0] = 0x02; // No flags, length = 2
        tx_payload[1] = 0x00; // Send to master ID
        command = SHIFT_PISO_STATE;
        tx_payload[2] = (uint8_t)command;
        ctrl_router();
        break;
      case CTRL_SEND_NFC: // NFC component sending state
        tx_payload[0] = 0x02; // No flags, length = 2
        tx_payload[1] = 0x00; // Send to master ID
        command = NFC_SOF;
        tx_payload[2] = (uint8_t)command;
        ctrl_router();
        break;
    }
  }
}

static void rx_task(void *arg){
  static const char* TAG = "CAN_Rx";
  static const char* type[8]= {"WRITE_ALL","WRITE","READ","unused","INHERIT","PING_REQ_ALL","PING_RES","PING_REQ"};
  rx_task_action_t msg_type;
  twai_message_t rx_msg;
  ctrl_task_action_t ctrl_action;
  xSemaphoreTake(rx_task_sem, portMAX_DELAY); // Blocked from beginning until ctrl_task gives semaphore
  xSemaphoreGive(rx_payload_sem);
  ESP_LOGI(TAG, "Task initialized");
  while(1){ // Runs forever after taking semaphore
    //Wait for message
    if (twai_receive(&rx_msg, pdMS_TO_TICKS(11000)) == ESP_OK) { // Wait for messages on CAN bus
      ESP_LOGI(TAG, "Received message...");
      rx_payload[1] = (uint8_t)(rx_msg.identifier & 0xFF);
      msg_type = (rx_task_action_t)(rx_msg.identifier >> 8);
      ESP_LOGI(TAG, "From_ID: %d, Type: %s, To_ID: %d",rx_payload[1],type[msg_type],rx_msg.data[0]);
      switch(msg_type){
        case RX_WRITE:
          if(ID_PROP != rx_msg.data[0]) break; // Must be addressed to us
        case RX_WRITE_ALL: // Write command to ALL props
          ctrl_action = CTRL_CMD;
          xSemaphoreTake(rx_payload_sem, portMAX_DELAY); // Can only take if payload not being read by ctrl_task
          rx_payload[0] = FLAG_WRITE | rx_msg.data_length_code;
          for(int i=1; i<rx_msg.data_length_code; i++){
            rx_payload[i+1] = rx_msg.data[i];
          }
          xQueueSend(ctrl_task_queue, &ctrl_action, portMAX_DELAY);
          xSemaphoreGive(ctrl_task_sem);
          break;
        case RX_READ: // Read command
          if(ID_PROP != rx_msg.data[0]) // Must be addressed to us
            break;
          xSemaphoreTake(rx_payload_sem, portMAX_DELAY); // Can only take if payload not being read by ctrl_task
          rx_payload[0] = rx_msg.data_length_code;
          for(int i=1; i<rx_msg.data_length_code; i++){
            rx_payload[i+1] = rx_msg.data[i];
          }
          xQueueSend(ctrl_task_queue, &ctrl_action, portMAX_DELAY);
          xSemaphoreGive(ctrl_task_sem);
          break;
        case RX_PING_REQ: // Ping request
          if(ID_PROP != rx_msg.data[0]) break; // Must be addressed to us
        case RX_PING_REQ_ALL: // Ping request to ALL props
          ctrl_action = CTRL_PING;
          rx_payload[0] = rx_msg.data[1] ? FLAG_PING : 0;
          xQueueSend(ctrl_task_queue, &ctrl_action, portMAX_DELAY);
          xSemaphoreGive(ctrl_task_sem);
          break;
        case RX_PING_RES: // Ping response
          // Implement recording ping responses from other props in this case
          break;
        default: // Unknown command
          ESP_LOGE(TAG, "Unknown command received from CAN bus");
      }
    } else {
      //ESP_LOGI(TAG, "No messages on the bus?");
    }
  }
}

static void tx_task(void *arg){
  tx_task_action_t action;
  twai_message_t tx_msg = {.identifier = 0b11000000000 + ID_PROP, .data_length_code = 0,
                                        .data = {0,0,0,0,0,0,0,0}, .self = 1};
  static const char* TAG = "CAN_Tx";
  ESP_LOGI(TAG, "Task initialized");
  for(;;){
    xSemaphoreTake(tx_task_sem, portMAX_DELAY); // Blocked from executing until ctrl_task gives semaphore
    xQueueReceive(tx_task_queue, &action, pdMS_TO_TICKS(10)); // Pull task from queue
    switch(action){
      case TX_PING:
        tx_msg.identifier = (6 << 8) | ID_PROP;
        tx_msg.data_length_code = 0; // No target or payload, therefore no length
        //Queue message for transmission
        if (twai_transmit(&tx_msg, portMAX_DELAY) == ESP_OK) {
          ESP_LOGI(TAG, "Transmitted ping response");
        } else {
          ESP_LOGE(TAG, "Failed to transmit ping response");
        }
        break;
      case TX_DATA:
        tx_msg.identifier = (6 << 8) | ID_PROP;
        tx_msg.data_length_code = tx_payload[0] & 0x0F;
        for(int i=0; i<tx_msg.data_length_code; i++){
          tx_msg.data[i] = tx_payload[i+1];
        }
        if (twai_transmit(&tx_msg, portMAX_DELAY) == ESP_OK) {
          ESP_LOGI(TAG, "Transmitted message");
        } else {
          ESP_LOGE(TAG, "Failed to transmit message");
        }
        break;
      default:
        ESP_LOGE(TAG, "Unknown action received: %d",action);
    }
    xSemaphoreGive(tx_payload_sem);
  }
}

static void ctrl_router(){
  tx_task_action_t tx_action = TX_DATA;
  puzzle_task_action_t puzzle_action;
  gpio_task_action_t gpio_action;
  shift_task_action_t shift_action;
  nfc_task_action_t nfc_action;
  can_command_t command;
  static const char* TAG = "CAN_Router";
  command = tx_payload[2];
  tx_payload[0] -= 2; // Reduce payload length by 2 (ignore command, target)
  switch(command){
    case GAME_STATE: // Set/send the game state
      puzzle_action = (tx_payload[0] & FLAG_WRITE) ? SET_STATE : SEND_STATE;
      xQueueSend(puzzle_task_queue, &puzzle_action, portMAX_DELAY);
      xSemaphoreGive(puzzle_task_sem);
      ESP_LOGI(TAG, "Sent state to Puzzle");
      if( (tx_payload[0] & FLAG_RES) || !(tx_payload[0] & FLAG_WRITE) ){
        xSemaphoreTake(ctrl_done_sem, portMAX_DELAY); // Blocked from executing until a task gives a semaphore
        tx_payload[0] += 2; // Increase payload length by 2 (add Command, Target_ID)
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xSemaphoreGive(tx_task_sem);
      }
      break;
    case GPIO_MASK: // GPIO mask of pins, write only
      gpio_action = SET_GPIO_MASK;
      xQueueSend(gpio_task_queue, &gpio_action, portMAX_DELAY);
      xSemaphoreGive(gpio_task_sem);
      ESP_LOGI(TAG, "Sent mask to GPIO");
      break;
    case GPIO_STATE: // GPIO states of pins to set/send
      gpio_action = (tx_payload[0] & FLAG_WRITE) ? SET_GPIO_STATES : SEND_GPIO_STATES;
      xQueueSend(gpio_task_queue, &gpio_action, portMAX_DELAY);
      xSemaphoreGive(gpio_task_sem);
      ESP_LOGI(TAG, "Sent states to GPIO");
      if( (tx_payload[0] & FLAG_RES) || !(tx_payload[0] & FLAG_WRITE) ){
        xSemaphoreTake(ctrl_done_sem, portMAX_DELAY); // Blocked from executing until a task gives a semaphore
        tx_payload[0] += 2; // Increase payload length by 2 (add Command, Target_ID)
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xSemaphoreGive(tx_task_sem);
      }
      break;
    case SOUND: // Play music, write only
      xTaskNotify(sound_task_handle,tx_payload[3],eSetValueWithOverwrite);
      break;
    case SHIFT_SIPO_MASK: // Mask of shift register pins, write only
      shift_action = SET_SIPO_MASK;
      xQueueSend(shift_task_queue, &shift_action, portMAX_DELAY);
      xSemaphoreGive(shift_task_sem);
      ESP_LOGI(TAG, "Sent mask to Shift Register");
      break;
    case SHIFT_SIPO_STATE: // States of shift SIPOs to set/send
      shift_action = (tx_payload[0] & FLAG_WRITE) ? SET_SIPO_STATES : SEND_SIPO_STATES;
      xQueueSend(shift_task_queue, &shift_action, portMAX_DELAY);
      xSemaphoreGive(shift_task_sem);
      ESP_LOGI(TAG, "Sent states to Shift Register");
      if( (tx_payload[0] & FLAG_RES) || !(tx_payload[0] & FLAG_WRITE) ){
        xSemaphoreTake(ctrl_done_sem, portMAX_DELAY); // Blocked from executing until a task gives a semaphore
        tx_payload[0] += 2; // Increase payload length by 2 (add Command, Target_ID)
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xSemaphoreGive(tx_task_sem);
      }
      break;
    case SHIFT_PISO_STATE: // States of shift PISOs, read only
      tx_payload[0] &= ~FLAG_WRITE; // Force read
      shift_action = SEND_PISO_STATES;
      xQueueSend(shift_task_queue, &shift_action, portMAX_DELAY);
      xSemaphoreGive(shift_task_sem);
      ESP_LOGI(TAG, "Sent states to Shift Register");
      xSemaphoreTake(ctrl_done_sem, portMAX_DELAY); // Blocked from executing until a task gives a semaphore
      tx_payload[0] += 2; // Increase payload length by 2 (add Command, Target_ID)
      xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
      xSemaphoreGive(tx_task_sem);
      break;
    case NFC_SOF: // NFC Write/Send SOF or Read request
      nfc_action = (tx_payload[0] & FLAG_WRITE) ? NFC_WRITE_DATA : NFC_SEND_DATA;
      xQueueSend(nfc_task_queue, &nfc_action, portMAX_DELAY);
      xSemaphoreGive(nfc_task_sem);
      ESP_LOGI(TAG, "Sent SOF to NFC");
      if(nfc_action == NFC_SEND_DATA){
        xSemaphoreTake(ctrl_done_sem, portMAX_DELAY); // Blocked from executing until a task gives a semaphore
        if(nfc.uid[7] != 0xE0){
          tx_payload[0] &= 0xF0;
        }
        else{
          tx_payload[0] = 0; // Clear flags and length
          uint16_t memSize = nfc.numBlocks * nfc.blockSize;
          uint8_t numTxn = memSize / 6;
          uint8_t txLength = 6;
          if(memSize % 6) numTxn++;
          for(int i=0; i<numTxn; i++){
            if(i == 0) tx_payload[2] = 6; // First txn is SOF
            if(i == numTxn - 1){ // Last txn?
              if(i != 0) tx_payload[2] = 8; // If not first txn, is EOF
              if(memSize % 6) txLength = memSize % 6; // If remainder, txlength is remainder
            }
            for(int j=0; j<txLength; j++){
              tx_payload[i+3] = nfc.blockData[i*8 + j];
            }
            tx_payload[0] = txLength;
            xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
            xSemaphoreGive(tx_task_sem);
            xSemaphoreTake(tx_payload_sem, portMAX_DELAY); // Blocked until tx_task is no longer reading tx_payload
          }
        }
        xSemaphoreGive(tx_payload_sem); // Replace last semaphore take
        xSemaphoreGive(nfc_cont_sem); // Allow nfc component to resume
      }
      break;
    case NFC_DATA: // NFC Write continuation
      nfc_action = NFC_WRITE_DATA;
      xQueueSend(nfc_task_queue, &nfc_action, portMAX_DELAY);
      xSemaphoreGive(nfc_task_sem);
      ESP_LOGI(TAG, "Sent Data to NFC");
      break;
    case NFC_EOF: // NFC Write EOF
      nfc_action = NFC_WRITE_DATA;
      xQueueSend(nfc_task_queue, &nfc_action, portMAX_DELAY);
      xSemaphoreGive(nfc_task_sem);
      ESP_LOGI(TAG, "Sent EOF to NFC");
      break;
    default:
      ESP_LOGE(TAG, "Unknown command from CAN bus");
  }
}

void fake_bus_task(void *arg){
  twai_message_t gpio_mask = {.identifier = 0b00100000000, .data_length_code = 7,
                                      .data = {0x01,0x01,0xFF,0xFF,0xFF,0xFF,0xFF}, .self = 1};
  twai_message_t gpio_states = {.identifier = 0b00100000000, .data_length_code = 7,
                                      .data = {0x01,0x02,0x00,0x00,0x00,0x00,0x00}, .self = 1};
  twai_message_t play_sound = {.identifier = 0b00100000000, .data_length_code = 3,
                                      .data = {0x01,0x03,0x01}, .self = 1};
  static const char* TAG = "CAN_Fake_Bus";
  ESP_LOGI(TAG, "Task initialized");
  for(;;){
    // Fake ping request
    //twai_transmit(&ping_req, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(5000)); // Simulating no messages for 5 seconds
    //Fake GPIO change
    twai_transmit(&gpio_mask, portMAX_DELAY);
    twai_transmit(&gpio_states, portMAX_DELAY);
    for(int i=2; i<8; i++){
        gpio_states.data[i]++; // Alter the pins being changed for next loop
    }
    vTaskDelay(pdMS_TO_TICKS(5000)); // Simulating no messages for 5 seconds
    // Fake sound command
    twai_transmit(&play_sound, portMAX_DELAY);
    //vTaskDelay(pdMS_TO_TICKS(5000)); // Simulating no messages for 5 seconds
  }
}