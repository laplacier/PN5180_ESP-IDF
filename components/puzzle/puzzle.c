#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "puzzle.h"
#include "twai_can.h"
#include "sound.h"
#include "shift_reg.h"
#include "gpio_prop.h"
#include "iso15693.h"
#include "pn5180.h"

//-------------Private Prototypes--------------//
static void puzzle_main(void *arg);
static void puzzle_task(void *arg);
static esp_err_t CAN_Receive(uint32_t delay);

//---------External Global Variables-----------//

//twai_can
extern QueueHandle_t ctrl_task_queue;
extern SemaphoreHandle_t ctrl_task_sem;
extern SemaphoreHandle_t ctrl_done_sem;
extern uint8_t tx_payload[9];

//sound
extern TaskHandle_t sound_task_handle;

//gpio
extern QueueHandle_t gpio_task_queue;
extern SemaphoreHandle_t gpio_task_sem;

//shift_reg
//extern uint8_t pisoData[NUM_PISO];
//extern uint8_t sipoData[NUM_SIPO];
//extern QueueHandle_t shift_task_queue;
//extern SemaphoreHandle_t shift_task_sem;

//nfc
extern ISO15693NFC_t nfc; // Struct holding data from nfc tags

//--------------Global Variables---------------//
SemaphoreHandle_t puzzle_task_sem;
QueueHandle_t puzzle_task_queue;
uint8_t game_state = 0;
//uint8_t piso_old[NUM_PISO];

/*
 *
 */
static void puzzle_main(void *arg){
  // Setup
  static const char* TAG = "User";
  /*for(int i=0; i<NUM_PISO; i++){
    piso_old[i] = dataIn[i];
  }

  // Loop forever
  while(1){
    for(int i=0; i<8; i++){
      if(bitRead(dataIn[0],i) != bitRead(piso_old[0],i)){
        ESP_LOGI(TAG, "PISO#0 Pin %d: %d",i,bitRead(dataIn[0],i));
      }
    }
    piso_old[0] = dataIn[0];

    shift_write(1,1);
    shift_show();
    vTaskDelay(pdMS_TO_TICKS(1000));
    shift_write(1,0);
    shift_show();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }*/

  while(1){

  }
}

/*
 * Helper functions - DO NOT EDIT
 * Functions required for flow control between other components
 */
void puzzle_init(void){
  puzzle_task_queue = xQueueCreate(1, sizeof(puzzle_task_action_t));
  puzzle_task_sem = xSemaphoreCreateCounting( 10, 0 );
  xTaskCreatePinnedToCore(puzzle_task, "Puzzle", 4096, NULL, PUZZLE_TASK_PRIO, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(puzzle_main, "User", 4096, NULL, PUZZLE_TASK_PRIO, NULL, tskNO_AFFINITY);
  ESP_LOGI("Puzzle", "Setup complete");
}

static void puzzle_task(void *arg){
  ctrl_task_action_t ctrl_action = CTRL_SEND_PUZZLE;
  uint8_t game_state_old = game_state;
  while(1){
    if(game_state != game_state_old){                           // If game state changed...
      game_state_old = game_state;                              // Record as last state
      xQueueSend(ctrl_task_queue, &ctrl_action, portMAX_DELAY); // Queue up to send game state on CAN bus
    }
    CAN_Receive(10);                                            // Check for tasks from CAN bus
  }
}

static esp_err_t CAN_Receive(uint32_t delay){
  static const char* TAG = "Puzzle";
  puzzle_task_action_t puzzle_action;
  if(xSemaphoreTake(puzzle_task_sem, pdMS_TO_TICKS(delay)) == pdTRUE){      // Blocked from executing until ctrl_task gives semaphore
    xQueueReceive(puzzle_task_queue, &puzzle_action, pdMS_TO_TICKS(delay)); // Pull task from queue
    switch(puzzle_action){               // Read command from the CAN bus
      case SET_STATE:                    // Action: Set game state
        game_state = tx_payload[3];
        if(tx_payload[0] & FLAG_RES){    // If a response is requested...
          xSemaphoreGive(ctrl_done_sem); // Inform ctrl_task data is ready
        }
        ESP_LOGI(TAG, "Set game state");
        break;
      case SEND_STATE:                   // Action: Send current game state
        tx_payload[0] &= 0xF0;           // Preserve flags
        tx_payload[0] |= 0x01;           // Length = 1
        tx_payload[2] = 0;               // Command = GAME_STATE
        tx_payload[3] = game_state;      // GAME_STATE payload
        xSemaphoreGive(ctrl_done_sem);   // Inform ctrl_task data is ready
        ESP_LOGI(TAG, "Sent game state");
        break;
      default:
        ESP_LOGE(TAG, "Unknown action received");
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
  }
  else{
    return ESP_ERR_TIMEOUT;
  }
}