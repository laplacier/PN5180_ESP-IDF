#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "iso15693.h"
#include "pn5180.h"
static const char* TAG = "main.c";
void showIRQStatus(uint32_t irqStatus);
void printUID(const char* tag, uint8_t* uid, uint8_t len);

extern const char afi_string[14][30];
extern const char manufacturerCode[110][100];

void app_main(void)
{
  ESP_LOGI(TAG, "pn5180 example using ISO15493 NFC tags");

  pn5180_init(); // Initialize SPI communication, set up ISO15693, activate RF

  // PN5180 Product version
  uint8_t product[2];
  pn5180_readEEprom(PN5180_PRODUCT_VERSION, product, 2);
  if(0xff == product[1]){
    ESP_LOGE(TAG, "Initialization failed. Reset to restart.");
    while(1) vTaskDelay(portMAX_DELAY);
  }
  ESP_LOGI(TAG,"Product version: %d.%d",product[1],product[0]);

  
  // PN5180 Firmware version
  uint8_t firmware[2];
  pn5180_readEEprom(PN5180_FIRMWARE_VERSION, firmware, 2);
  ESP_LOGI(TAG,"Firmware version: %d.%d",firmware[1],firmware[0]);

  // PN5180 EEPROM version
  uint8_t eeprom[2];
  pn5180_readEEprom(PN5180_EEPROM_VERSION, eeprom, 2);
  ESP_LOGI(TAG,"EEPROM version: %d.%d",eeprom[1],eeprom[0]);

  //pn5180_setRF_off();
  ESP_LOGI(TAG, "Starting read cycle...");
  vTaskDelay(pdMS_TO_TICKS(1000));

  ISO15693NFC_t nfc;
  ISO15693Inventory_t inventory;
  inventory.numCard = 0;
  uint32_t pollCount = 1;
  while(1){

    // Multiple inventory
    ESP_LOGD(TAG, "Poll #%ld", pollCount);
    ISO15693ErrorCode_t rc = pn5180_getInventoryMultiple(&inventory);
    if (ISO15693_EC_OK != rc) {
      iso15693_printError(rc);
    }
    else if(!inventory.numCard){
      ESP_LOGI(TAG, "No cards detected.");
    }
    else{
      ESP_LOGD(TAG, "Polled %d cards.", inventory.numCard);
      for(int i=0; i<inventory.numCard; i++){
        printUID(TAG, inventory.uid[i], 8);
      }
    }
    pollCount++;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void printUID(const char* tag, uint8_t* uid, uint8_t len){
  printf("\033[32mI (%ld) %s: UID=", esp_log_timestamp(), tag);
  for(int i=7; i>=0; i--){
    if(uid[i] < 16) printf("0");
    printf("%X", uid[i]);
    if(i > 0) printf(":");
  }
  printf("\n\033[0m");
}
