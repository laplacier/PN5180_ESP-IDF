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


 /*
  * GPIO 0 - Pulled high on dev boards. If pulled low during boot while GPIO 2 is low/NC, will trigger serial bootloader. DO NOT USE.
  * GPIO 1 - USB TX on dev boards, DO NOT USE.
  * GPIO 2 - Pulled low on dev boards. If pulled low/NC during boot while GPIO 0 is low, will trigger serial bootloader. Use caution.
  * GPIO 3 - USB RX on dev boards, DO NOT USE.
  * GPIO 4 - OK
  * GPIO 5 - If pulled high/low on boot, can alter SDIO Slave timing. Use caution.
  * GPIO 6-11 - Connceted to internal FLASH. DO NOT USE. 
  * GPIO 12 - Pulled low by default. If pulled high during boot, device will fail to start. Use caution.
  * GPIO 13-15 - OK
  * GPIO 16 - OK, UART2_RX
  * GPIO 17 - OK, UART2_TX
  * GPIO 18-19 - OK
  * GPIO 20 - Not usable in any board. DO NOT USE.
  * GPIO 21 - OK, I2C_SDA
  * GPIO 22 - OK, I2C_SCL
  * GPIO 23 - OK
  * GPIO 24 - Not usable in any board. DO NOT USE.
  * GPIO 25-27 - OK
  * GPIO 28-31 - Not usable in any board. DO NOT USE.
  * GPIO 32-33 - OK
  * GPIO 34-36 - OK, input only.
  * GPIO 37-38 - Not usable in any board. DO NOT USE.
  * GPIO 39 - OK, input only.
 */

// twai_can
extern QueueHandle_t ctrl_task_queue;
extern SemaphoreHandle_t ctrl_task_sem;
extern SemaphoreHandle_t ctrl_done_sem;
extern uint8_t tx_payload[9];

SemaphoreHandle_t gpio_task_sem;
QueueHandle_t gpio_task_queue;

static void gpio_update(void);
static uint64_t gpio_states = 0;            // Initial state of pins
static uint64_t gpio_states_old = 0;        // Previous state of pins
static uint64_t mask_protect = 0b1111111111111111111111110110000011110001000100000000111111001011; // Default DNU pins implemented
static const char* TAG = "GPIO";

void GPIO_Init(void){
  /*if(CONFIG_ENABLE_SOUND){
    ESP_LOGI(TAG, "Protecting sound pins");
    mask_protect |= 1ULL << UART_TX_GPIO;
    mask_protect |= 1ULL << UART_RX_GPIO;
  }
  if(CONFIG_ENABLE_CAN){
    ESP_LOGI(TAG, "Protecting CAN pins");
    mask_protect |= 1ULL << CAN_TX_GPIO;
    mask_protect |= 1ULL << CAN_RX_GPIO;
  }
  if(CONFIG_ENABLE_SHIFT){
    ESP_LOGI(TAG, "Protecting shift register pins");
    mask_protect |= 1ULL << SHIFT_CLOCK_GPIO;
    mask_protect |= 1ULL << PISO_LOAD_GPIO;
    mask_protect |= 1ULL << PISO_DATA_GPIO;
    mask_protect |= 1ULL << SIPO_LATCH_GPIO;
    mask_protect |= 1ULL << SIPO_DATA_GPIO;
  }*/

  gpio_task_queue = xQueueCreate(10, sizeof(gpio_task_action_t));
  gpio_task_sem = xSemaphoreCreateCounting( 10, 0 ); 
  if(gpio_task_queue == NULL){
    ESP_LOGI(TAG, "GPIO queue failed to create!");
  }
  xTaskCreatePinnedToCore(gpio_task, "GPIO", 2048, NULL, GENERIC_TASK_PRIO, NULL, tskNO_AFFINITY);
  ESP_LOGI(TAG, "Setup complete");
}

void gpio_task(void *arg){
  gpio_task_action_t gpio_action;
  uint64_t gpio_mask = mask_protect; // Mask applied to select GPIO mins to modify
  while(1){
    if(xSemaphoreTake(gpio_task_sem, pdMS_TO_TICKS(10)) == pdTRUE){ // Blocked from executing until puzzle_task gives a semaphore
      xQueueReceive(gpio_task_queue, &gpio_action, portMAX_DELAY); // Pull task from queue
      uint8_t rxLength = tx_payload[0] & 0x0F;
      if(rxLength > 5) rxLength = 5;
      switch(gpio_action){
        case SET_GPIO_MASK:
          gpio_mask = 0;
          for(int i=0; i<rxLength; i++){
            BYTESHIFTL(gpio_mask,1); // Shift previous data over by a byte
            gpio_mask |= tx_payload[i+3]; // Copy byte of mask to now empty byte in var
          }
          gpio_mask &= ~(mask_protect); // Remove pins used by other components from mask
          ESP_LOGI(TAG, "Set mask");
          break;
        case SET_GPIO_STATES:
          gpio_states = 0;
          for(int i=0; i<rxLength; i++){
            BYTESHIFTL(gpio_states,1);      // Shift previous data over by a byte
            gpio_states |= tx_payload[i+3]; // Copy byte of pins to modify to now empty byte in var
          }
          if(tx_payload[0] & FLAG_RES) xSemaphoreGive(ctrl_done_sem);
          for(int i=0; i<34; i++){ // GPIO 34+ cannot be output, ignore
            if(gpio_mask & BIT(i)){ // If this pin is being set...
              gpio_set_level(i, ((gpio_states >> i) & 1U)); // Set the state of the pin
            }
          }
          ESP_LOGI(TAG, "Set output states");
          break;
        case SEND_GPIO_STATES:                                      // Command: Send states to CAN_TX payload
          gpio_states = 0;
          for(int i=0; i<39; i++){ // Snapshot state of all GPIO pins
            bitWrite(gpio_states,i,gpio_get_level(i));
          }
          for(int i=0; i<5; i++){
            tx_payload[i+3] = READBYTE(gpio_states,i); // Transfer the current state byte to the CAN_TX payload
          }
          xSemaphoreGive(ctrl_done_sem);
          break;
      }
    }
    gpio_update();
  }
}

/*
 * Wrapper functions below to simplify setting pins for user
*/
void gpio_mode(uint8_t pin, gpio_mode_wrapper_t mode, gpio_interrupt_t type){
    gpio_config_t io_conf = {};
    switch(mode){
      case OUTPUT:
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT_OUTPUT; // We want to simplify sending pin states, so in/out required
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        break;
      case INPUT:
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        break;
      case INPUT_PULLUP:
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 1;
        break;
      case INPUT_PULLDOWN:
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = 1;
        io_conf.pull_up_en = 0;
        break;
      case INPUT_INTERRUPT:
        if(type == RISING){
          io_conf.intr_type = GPIO_INTR_POSEDGE;
        }
        else if(type == FALLING){
          io_conf.intr_type = GPIO_INTR_NEGEDGE;
        }
        else{
          io_conf.intr_type = GPIO_INTR_ANYEDGE;
        }
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        break;
      default:
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        break;
    }
    io_conf.pin_bit_mask = (1ULL << pin);    // Select the pin in mask
    io_conf.pin_bit_mask &= ~(mask_protect); // Ensure it is not a pin from another component
    gpio_config(&io_conf);                   // configure GPIO with the given settings
}

bool gpio_read(uint8_t pin){
  if(mask_protect & BIT(pin)){
    ESP_LOGE(TAG, "Read protected: Pin is in use by another component");
    return ESP_FAIL;
  }
  return (gpio_get_level(pin));
}

esp_err_t gpio_write(uint8_t pin, bool level){
  if(mask_protect & BIT(pin)){
    ESP_LOGE(TAG, "Write protected: Pin is in use by another component");
    return ESP_FAIL;
  }
  return (gpio_set_level(pin, level));
}

static void gpio_update(void){
  ctrl_task_action_t ctrl_action = CTRL_SEND_GPIO;
  if(gpio_states != gpio_states_old){                         // If gpio state changed...
    gpio_states = gpio_states_old;                            // Record as last state
    xQueueSend(ctrl_task_queue, &ctrl_action, portMAX_DELAY); // Queue up to send gpio states on CAN bus
  }
}