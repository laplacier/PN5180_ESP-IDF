#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "puzzle.h"
#include "twai_can.h"
#include "shift_reg.h"
#include "sound.h"
#include "gpio_prop.h"

extern QueueHandle_t ctrl_task_queue;
extern SemaphoreHandle_t ctrl_task_sem;
extern SemaphoreHandle_t ctrl_done_sem;
extern uint8_t tx_payload[9];

TaskHandle_t shift_task_handle;
SemaphoreHandle_t shift_task_sem;
QueueHandle_t shift_task_queue;

static void piso_update(void);
static void sipo_update(void);
static void pulsePin(uint8_t pinName, uint32_t pulseTime);
static void shift_task(void *arg);
static uint8_t pisoData_old[NUM_PISO];    // Last PISO state
static uint8_t sipoData_old[NUM_SIPO];    // Last SIPO state
static uint8_t maskSIPO[NUM_SIPO]; // Mask applied to SIPO registers

uint8_t pisoData[NUM_PISO];        // Data read from PISO registers
uint8_t sipoData[NUM_SIPO];        // Data sent to SIPO registers

void shift_init(void){
  uint64_t mask = 0;
  for(int i=0; i<NUM_SIPO; i++){
    sipoData[i] = 0;
  }
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  
  // Outputs
  io_conf.mode = GPIO_MODE_INPUT_OUTPUT; // We want to simplify sending pin states to other props, so in/out required
  mask |= 1ULL << SHIFT_CLOCK_GPIO;
  mask |= 1ULL << PISO_LOAD_GPIO;
  mask |= 1ULL << SIPO_LATCH_GPIO;
  mask |= 1ULL << SIPO_DATA_GPIO;
  io_conf.pin_bit_mask = mask;
  gpio_config(&io_conf);

  // Inputs
  mask = 0;
  io_conf.mode = GPIO_MODE_INPUT;
  mask |= 1ULL << PISO_DATA_GPIO;
  io_conf.pin_bit_mask = mask;
  gpio_config(&io_conf);

  // Create shift register tasks, queues, semaphores
  shift_task_queue = xQueueCreate(10, sizeof(shift_task_action_t));
  shift_task_sem = xSemaphoreCreateCounting( 10, 0 );
  xTaskCreatePinnedToCore(shift_task, "SIPO", 2048, NULL, GENERIC_TASK_PRIO, NULL, tskNO_AFFINITY);
  ESP_LOGI("Shift_Reg", "Setup complete");
}

// User functions
bool shift_read(uint8_t pin){
  uint8_t piso_num = pin >> 3;
  pin %= 8;
  bool val = bitRead(pisoData[piso_num], pin);
  return val;
}

void shift_write(uint8_t pin, bool val){
  uint8_t sipo_num = pin >> 3;
  if(sipo_num > NUM_SIPO){
    ESP_LOGE("Shift_Reg", "Attempted to write to SIPO#%d, does not exist",sipo_num);
  }
  else{
    pin %= 8;
    bitWrite(sipoData[sipo_num], pin, val);
  }
}

void shift_show(void){
  sipo_update();
}

static void pulsePin(uint8_t pinName, uint32_t pulseTime){
  gpio_write(pinName, 0);
  esp_rom_delay_us(pulseTime);
  gpio_write(pinName, 1);
  esp_rom_delay_us(pulseTime);
}

static void piso_update(void){
  ctrl_task_action_t ctrl_action = CTRL_SEND_SHIFT_PISO;
  bool flag_send_CAN = 0;

  gpio_write(SHIFT_CLOCK_GPIO, 1);               // Data is shifted when clock pin changes from LOW to HIGH, so ensure it starts HIGH
  pulsePin(PISO_LOAD_GPIO, 5);                             // Pulse load pin to snapshot all PISO pin states and start at the beginning
  for(int i=0; i<NUM_PISO; i++){                     // For each PISO register...
    for(int j=7; j>=0; j--){                        // For each bit in the PISO register...
      bitWrite(pisoData[i], j, gpio_read(PISO_DATA_GPIO)); // Write the current bit to the corresponding bit in the input variable
      pulsePin(SHIFT_CLOCK_GPIO, 5);                        // Pulse the clock to shift the next bit in from the PISO register
    }
    if(pisoData_old[i] != pisoData[i]){
      flag_send_CAN = 1;
      pisoData_old[i] = pisoData[i];
    }
  }

  if(flag_send_CAN){
    xQueueSend(ctrl_task_queue, &ctrl_action, portMAX_DELAY);
    xSemaphoreGive(ctrl_task_sem);
  }
}

static void sipo_update(void){
  ctrl_task_action_t ctrl_action = CTRL_SEND_SHIFT_SIPO;
  bool flag_send_CAN = 0;

  for(int i=NUM_SIPO-1; i>=0; i--){
    for(int j=7; j>=0; j--){
      gpio_write(SIPO_DATA_GPIO, bitRead(sipoData[i], j));
      pulsePin(SHIFT_CLOCK_GPIO, 5);
    }
    if(sipoData_old[i] != sipoData[i]){
      flag_send_CAN = 1;
      sipoData_old[i] = sipoData[i];
    }
  }
  pulsePin(SIPO_LATCH_GPIO, 5);
  ESP_LOGI("sipo","%d",sipoData[0]);

  if(flag_send_CAN){
    xQueueSend(ctrl_task_queue, &ctrl_action, portMAX_DELAY);
    xSemaphoreGive(ctrl_task_sem);
  }
}

static void shift_task(void *arg){
  can_command_t command;
  shift_task_action_t shift_action;
  static const char* TAG = "Shift_Reg";
  
  while(1){
    if(xSemaphoreTake(shift_task_sem, pdMS_TO_TICKS(10)) == pdTRUE){ // Blocked from executing until puzzle_task gives a semaphore
      xQueueReceive(shift_task_queue, &shift_action, portMAX_DELAY); // Pull task from queue
      switch(shift_action){
        case SET_SIPO_MASK:
          for(int i=0; i<NUM_SIPO; i++){
            maskSIPO[i] = tx_payload[i+3]; // Copy byte of mask to now empty byte in var
          }
          ESP_LOGI(TAG, "Set mask");
          break;
        case SET_SIPO_STATES:
          for(int i=0; i<NUM_SIPO; i++){
            sipoData[i] &= ~(maskSIPO[i]);
            sipoData[i] |= (tx_payload[i+3] & maskSIPO[i]);
          }
          sipo_update();
          ESP_LOGI(TAG, "Set output states");
          break;
        case SEND_SIPO_STATES:                                      // Command: Send states to CAN_TX payload
          command = SHIFT_SIPO_STATE;
          tx_payload[0] &= 0xF0;
          tx_payload[0] |= NUM_SIPO;                         // Read | Length = NUM_SIPO
          tx_payload[2] = (uint8_t)command;
          for(int i=0; i<NUM_SIPO; i++){
            tx_payload[i+3] = sipoData[i]; // Transfer the current state bytes to the CAN_TX payload
          }
          xSemaphoreGive(ctrl_done_sem);
          break;
        case SEND_PISO_STATES:                                      // Command: Send states to CAN_TX payload
          command = SHIFT_PISO_STATE;
          tx_payload[0] &= 0xF0;
          tx_payload[0] |= NUM_PISO;                         // Read | Length = NUM_PISO
          tx_payload[2] = command;
          for(int i=0; i<NUM_PISO; i++){
            tx_payload[i+3] = pisoData[i]; // Transfer the current state bytes to the CAN_TX payload
          }
          xSemaphoreGive(ctrl_done_sem);
          break;
      }
    }
    piso_update();
  }
}