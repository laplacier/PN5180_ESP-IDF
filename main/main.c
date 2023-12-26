/* PropRTOS Template
 *
*/

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "soc/gpio_sig_map.h" // For TWAI_TX_IDX
#include "puzzle.h"
#include "twai_can.h"
#include "sound.h"
#include "gpio_prop.h"
#include "shift_reg.h"
#include "pn5180.h"

void app_main(void){
  static const char* TAG = "Main";
  GPIO_Init();
  shift_init();
  sound_init();
  pn5180_init();
  puzzle_init();
  twai_can_init();
  ESP_LOGI(TAG, "All setups complete. Destroying main...");
}
