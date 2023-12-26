// NAME: PN5180.cpp
//
// DESC: Implementation of PN5180 class.
//
// Copyright (c) 2018 by Andreas Trappmann. All rights reserved.
//
// This file is part of the PN5180 library for the Arduino environment.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "twai_can.h"
#include "iso15693.h"
#include "pn5180.h"

#define ESP32_HOST SPI3_HOST
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_NSS  16
#define PIN_NUM_BUSY 5
#define PIN_NUM_RST  17
spi_device_handle_t pn5180;
SemaphoreHandle_t nfc_task_sem;
SemaphoreHandle_t nfc_cont_sem;
QueueHandle_t nfc_task_queue;
static const char *TAG = "pn5180.c";
static uint8_t uid_old[8] = {0,0,0,0,0,0,0,0};

ISO15693NFC_t nfc;
uint8_t *writeBuffer = NULL;
uint8_t readBuffer[508];

//twai_can
extern QueueHandle_t ctrl_task_queue;
extern SemaphoreHandle_t ctrl_task_sem;
extern SemaphoreHandle_t ctrl_done_sem;
extern uint8_t tx_payload[9];

////////////////////////
// Private Prototypes //
////////////////////////
static void nfc_task(void *arg);
static void nfc_update(void);
static esp_err_t pn5180_txn(spi_device_handle_t dev, const void *tx, int txLen, void *rx, int rxLen);
static esp_err_t pn5180_busy_wait(uint32_t timeout);

///////////////
// Functions //
///////////////
void pn5180_init(void){
  gpio_config_t io_conf = {};

  // NSS and Reset are outputs
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  io_conf.pin_bit_mask = 0;
  io_conf.pin_bit_mask |= (1ULL << PIN_NUM_NSS);
  io_conf.pin_bit_mask |= (1ULL << PIN_NUM_RST);
  gpio_config(&io_conf);

  // BUSY is input
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = 0;
  io_conf.pin_bit_mask |= (1ULL << PIN_NUM_BUSY);
  gpio_config(&io_conf);

  gpio_set_level(PIN_NUM_NSS, 1); // Deselect device
  gpio_set_level(PIN_NUM_RST, 1); // Prevent reset

  // Configure physical bus settings for pn5180
  ESP_LOGD(TAG, "init: Initializing bus SPI...");
  spi_bus_config_t pn5180_buscfg={
      .miso_io_num = PIN_NUM_MISO,
      .mosi_io_num = PIN_NUM_MOSI,
      .sclk_io_num = PIN_NUM_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
  };

  // Configure software settings for pn5180
  spi_device_interface_config_t pn5180_devcfg={
      .clock_speed_hz = 7000000,
      .mode = 0,
      .spics_io_num = PIN_NUM_NSS,
      .queue_size = 208,
      .pre_cb = NULL,
  };

  // Apply settings
  ESP_ERROR_CHECK(spi_bus_initialize(ESP32_HOST, &pn5180_buscfg, 1));
  ESP_ERROR_CHECK(spi_bus_add_device(ESP32_HOST, &pn5180_devcfg, &pn5180));
  ESP_LOGI(TAG, "init: Bus SPI Initialized");

  // Reset device
  pn5180_reset();

  // Configure and turn on RF
  ESP_ERROR_CHECK(pn5180_setupRF());

  // NFC task, queue, sem
  nfc_task_queue = xQueueCreate(1, sizeof(nfc_task_action_t));
  nfc_task_sem = xSemaphoreCreateCounting( 10, 0 );
  nfc_cont_sem = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(nfc_task, "NFC", 4096, NULL, GENERIC_TASK_PRIO, NULL, tskNO_AFFINITY);
}

static void nfc_task(void *arg){
  nfc_task_action_t nfc_action;
  const char* TAG = "NFC";
  uint8_t writeBufferPos = 0;
  bool flag_nfcWrite = false;

  while(1){
    uint16_t memSize = nfc.numBlocks * nfc.blockSize;
    uint8_t rxLength;
    if(xSemaphoreTake(nfc_task_sem, pdMS_TO_TICKS(10)) == pdTRUE){ // Blocked from executing until puzzle_task gives a semaphore
      xQueueReceive(nfc_task_queue, &nfc_action, portMAX_DELAY); // Pull task from queue
      switch(nfc_action){
        case NFC_WRITE_DATA:
          rxLength = tx_payload[0] & 0x0F;
          if(tx_payload[2] == 0x06){ // WRITE_DATA SOF
            if(writeBuffer != NULL){
              free(writeBuffer);
              writeBuffer = NULL;
            }
            writeBuffer = (uint8_t*)malloc(memSize * sizeof(uint8_t));
            writeBufferPos = 0;
          }
          else if(tx_payload[2] == 0x08){ // WRITE_DATA EOF
            flag_nfcWrite = 1;
          }
          for(int i=0; i<rxLength; i++){
            if(writeBufferPos < memSize)
              writeBuffer[writeBufferPos++] = tx_payload[i+1];
          }
          xSemaphoreGive(ctrl_done_sem); // Inform ctrl_task tx_payload is ready
          break;
        case NFC_SEND_DATA:                // Command: Send states to CAN_TX payload
          xSemaphoreGive(ctrl_done_sem); // Let ctrl_task begin reading NFC data
          xSemaphoreTake(nfc_cont_sem, portMAX_DELAY); // ctrl_task is done with NFC data
          break;
      }
    }
    ISO15693ErrorCode_t rc = pn5180_getInventory(&nfc); // Inventory from NFC tag
    if (ISO15693_EC_OK == rc) {
      rc = pn5180_getSystemInfo(&nfc); // System information from NFC tag
      if (ISO15693_EC_OK == rc) {
        if(flag_nfcWrite){ // If we need to write data received from CAN to NFC tag
          for(int i=0; i<memSize; i++){
            nfc.blockData[i] = writeBuffer[i]; // Copy data to NFC struct
          }
          for(int i=0; i<nfc.numBlocks; i++){
            pn5180_writeSingleBlock(&nfc, i); // Write data to NFC tag
          }
          flag_nfcWrite = 0;
        }
        else{
          pn5180_readMultipleBlock(&nfc, 0, nfc.numBlocks); // Read all blocks
        }
      }
    }
    else{ // If no NFC tag read, clear data
      for(int i=0; i<8; i++){
        nfc.uid[0] = 0;
      }
      nfc.numBlocks = 0;
      nfc.blockSize = 0;
    }
    nfc_update();
  }
}

static void nfc_update(void){
  ctrl_task_action_t ctrl_action = CTRL_SEND_NFC;
  bool flag_send_CAN = 0;
  for(int i=7; i>=0; i--){
    if(nfc.uid[i] != uid_old[i]){
      flag_send_CAN = 1;
      uid_old[i] = nfc.uid[i];
    }
  }
  if(flag_send_CAN){                                          // If nfc state changed...
    flag_send_CAN = 0;
    xQueueSend(ctrl_task_queue, &ctrl_action, portMAX_DELAY); // Queue up to send nfc data on CAN bus
  }
}

esp_err_t pn5180_command(uint8_t *sendBuffer, size_t sendBufferLen, uint8_t *recvBuffer, size_t recvBufferLen) {
  esp_err_t ret;
  ////////////////////
  // Initialization //
  ////////////////////
  ESP_LOGD(TAG, "command: Write, wait for busy...");
  ret = pn5180_busy_wait(1000); // 1.
  if(ret == ESP_ERR_TIMEOUT){
    ESP_LOGE(TAG, "command: BUSY signal line timeout");
    return ret;
  }
  ESP_LOGD(TAG, "command: SPI transaction: write %d read %d", sendBufferLen, recvBufferLen);
  
  //////////////////
  // Send command //
  //////////////////
  ESP_LOGD(TAG, "command: Write data:");
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, sendBuffer, sendBufferLen, ESP_LOG_DEBUG);
  ret = pn5180_txn(pn5180, sendBuffer, sendBufferLen, NULL, 0); // 2. 3. 4.
  if(ret != ESP_OK){
    ESP_LOGE(TAG, "pn5180_command: SPI transaction write failed");
    return ret;
  }

  // Finish if write-only command
  if ((0 == recvBuffer) || (0 == recvBufferLen)) return ret;

  ESP_LOGD(TAG, "command: Read, wait for busy...");
  ret = pn5180_busy_wait(1000); // 5.
  if(ret == ESP_ERR_TIMEOUT){
    ESP_LOGE(TAG, "command: BUSY signal line timeout");
    return ret;
  }
  memset(recvBuffer, 0xFF, recvBufferLen);

  //////////////////////
  // Receive Response //
  //////////////////////
  ret = pn5180_txn(pn5180, recvBuffer, recvBufferLen, recvBuffer, recvBufferLen); // 6. 7. 8.
  if(ret != ESP_OK){
    ESP_LOGE(TAG, "pn5180_command: SPI transaction read failed");
    return ret;
  }

  ESP_LOGD(TAG, "command: Read data:");
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, recvBuffer, recvBufferLen, ESP_LOG_DEBUG);
  
  return ret;
}

/*
 * The BUSY signal is used to indicate that the PN5180 is not able to send or receive data
 * over the SPI interface
 */
static esp_err_t pn5180_busy_wait(uint32_t timeout){
  while (gpio_get_level(PIN_NUM_BUSY) && timeout > 0){
    vTaskDelay(pdMS_TO_TICKS(1));
    timeout--;
  }
  vTaskDelay(pdMS_TO_TICKS(1));
  if(gpio_get_level(PIN_NUM_BUSY)){
    ESP_LOGE(TAG, "busy_wait: Timeout waiting for BUSY pin LOW");
    return ESP_ERR_TIMEOUT;
  }
  return ESP_OK;
}

esp_err_t pn5180_reset(void) {
  uint32_t retries = 10;
  gpio_set_level(PIN_NUM_RST, 0);
  vTaskDelay(pdMS_TO_TICKS(1));
  gpio_set_level(PIN_NUM_RST, 1);
  while((PN5180_IDLE_IRQ_STAT & pn5180_getIRQStatus()) == 0 && retries > 0){
    vTaskDelay(pdMS_TO_TICKS(10));
    retries--;
  }
  if((PN5180_IDLE_IRQ_STAT & pn5180_getIRQStatus()) == 0){
    ESP_LOGE(TAG, "reset: Timeout waiting for IRQ state IDLE");
    return ESP_ERR_TIMEOUT;
  }
  return ESP_OK;
}

static esp_err_t pn5180_txn(spi_device_handle_t dev, const void *tx, int txLen, void *rx, int rxLen) {
    spi_transaction_t txn = {
        .length = txLen * 8,
        .tx_buffer = tx,
        .rxlength = rxLen * 8,
        .rx_buffer = rx,
    };
    return spi_device_transmit(dev, &txn);
}

/*
 * WRITE_REGISTER - 0x00
 */
esp_err_t pn5180_writeRegister(uint8_t reg, uint32_t value) {
  uint8_t *p = (uint8_t*)&value;
  uint8_t buf[6] = { PN5180_WRITE_REGISTER, reg, p[0], p[1], p[2], p[3] };
  return pn5180_command(buf, 6, 0, 0);
}

/*
 * WRITE_REGISTER_OR_MASK - 0x01
 */
esp_err_t pn5180_writeRegisterWithOrMask(uint8_t reg, uint32_t mask) {
  uint8_t *p = (uint8_t*)&mask;
  uint8_t buf[6] = { PN5180_WRITE_REGISTER_OR_MASK, reg, p[0], p[1], p[2], p[3] };
  return pn5180_command(buf, 6, 0, 0);
}

/*
 * WRITE_REGISTER_AND_MASK - 0x02
 */
esp_err_t pn5180_writeRegisterWithAndMask(uint8_t reg, uint32_t mask) {
  uint8_t *p = (uint8_t*)&mask;
  uint8_t buf[6] = { PN5180_WRITE_REGISTER_AND_MASK, reg, p[0], p[1], p[2], p[3] };
  return pn5180_command(buf, 6, 0, 0);
}

/*
 * READ_REGISTER - 0x04
 */
esp_err_t pn5180_readRegister(uint8_t reg, uint32_t *value) {
  uint8_t cmd[2] = { PN5180_READ_REGISTER, reg };
  return pn5180_command(cmd, 2, (uint8_t*)value, 4);
}

/*
 * WRITE_EEPROM - 0x06
 */
esp_err_t pn5180_writeEEprom(uint8_t addr, uint8_t *buffer, uint16_t len) {
  if ((addr > 254) || ((addr+len) > 254)) {
    ESP_LOGE(TAG, "writeEEprom: Size of tx buffer exceeds 253 bytes");
    return ESP_ERR_INVALID_SIZE;
  }
	uint8_t cmd[len + 2];
	cmd[0] = PN5180_WRITE_EEPROM;
	cmd[1] = addr;
	for (int i = 0; i < len; i++) cmd[2 + i] = buffer[i];
	return pn5180_command(cmd, len + 2, 0, 0);
}

/*
 * READ_EEPROM - 0x07
 */
esp_err_t pn5180_readEEprom(uint8_t addr, uint8_t *buffer, uint16_t len) {
  if ((addr > 254) || ((addr+len) > 254)) {
    ESP_LOGE(TAG, "readEEprom: Size of rx buffer exceeds 253 bytes");
    return ESP_ERR_INVALID_SIZE;
  }
  uint8_t cmd[3] = { PN5180_READ_EEPROM, addr, (uint8_t)(len) };
  return pn5180_command(cmd, 3, buffer, len);
}

uint32_t pn5180_getIRQStatus() {
  uint32_t irqStatus;
  pn5180_readRegister(PN5180_IRQ_STATUS, &irqStatus);
  return irqStatus;
}

esp_err_t pn5180_clearIRQStatus(uint32_t irqMask) {
  return pn5180_writeRegister(PN5180_IRQ_CLEAR, irqMask);
}

/*
 * LOAD_RF_CONFIG - 0x11
 */
esp_err_t pn5180_loadRFConfig(uint8_t txConf, uint8_t rxConf) {
  uint8_t cmd[3] = { PN5180_LOAD_RF_CONFIG, txConf, rxConf };
  return pn5180_command(cmd, 3, 0, 0);
}

/*
 * RF_ON - 0x16
 */
esp_err_t pn5180_setRF_on() {
  uint8_t cmd[2] = { PN5180_RF_ON, 0x00 };
  pn5180_command(cmd, 2, 0, 0);

  uint8_t retries = 50;
  while (0 == (PN5180_TX_RFON_IRQ_STAT & pn5180_getIRQStatus()) && retries > 0) {   // wait for RF field to set up (max 500ms)
    vTaskDelay(pdMS_TO_TICKS(10));
	  retries--;
  }
  ESP_LOGD(TAG, "setRF_on: IRQ State after set - %ld", pn5180_getIRQStatus());
  if(0 == (PN5180_TX_RFON_IRQ_STAT & pn5180_getIRQStatus())){
    ESP_LOGE(TAG, "setRF_on: Failed to detect IRQ state TX_RFON");
    return ESP_FAIL;
  }

  pn5180_clearIRQStatus(PN5180_TX_RFON_IRQ_STAT);
  return ESP_OK;
}

/*
 * RF_OFF - 0x17
 */
esp_err_t pn5180_setRF_off() {

  uint8_t cmd[2] = { PN5180_RF_OFF, 0x00 };
  pn5180_command(cmd, 2, 0, 0);

  uint8_t retries = 50;
  while (0 == (PN5180_TX_RFOFF_IRQ_STAT & pn5180_getIRQStatus()) && retries > 0) {   // wait for RF field to set up (max 500ms)
    vTaskDelay(pdMS_TO_TICKS(10));
	  retries--;
  }
  if(0 == (PN5180_TX_RFOFF_IRQ_STAT & pn5180_getIRQStatus())) return ESP_FAIL;

  pn5180_clearIRQStatus(PN5180_TX_RFOFF_IRQ_STAT);
  return ESP_OK;
}; 

/*
 * SEND_DATA - 0x09
 */
esp_err_t pn5180_sendData(uint8_t *data, uint16_t len, uint8_t validBits) {
  if (len > 260){
    ESP_LOGE(TAG, "sendData: Length of data exceeds 260 bytes");
    return ESP_ERR_INVALID_SIZE;
  }

  uint8_t buffer[len+2];
  buffer[0] = PN5180_SEND_DATA;
  buffer[1] = validBits; // number of valid bits of last byte are transmitted (0 = all bits are transmitted)
  for (int i=0; i<len; i++) {
    buffer[2+i] = data[i];
  }

  pn5180_writeRegisterWithAndMask(PN5180_SYSTEM_CONFIG, 0xfffffff8);  // Idle/StopCom Command
  pn5180_writeRegisterWithOrMask(PN5180_SYSTEM_CONFIG, 0x00000003);   // Transceive Command

  PN5180TransceiveState_t state = getTransceiveState();
  ESP_LOGD(TAG,"sendData: state=%d",(uint8_t)(state));
  if (PN5180_TS_WaitTransmit != state){
    ESP_LOGE(TAG, "sendData: TransceiveState not WaitTransmit");
    return ESP_ERR_INVALID_STATE;
  }

  return pn5180_command(buffer, len+2, 0, 0);
}

/*
 * READ_DATA - 0x0A
 */
uint8_t *pn5180_readData(int len) {
  if (len > 508) return 0L;
  uint8_t cmd[2] = { PN5180_READ_DATA, 0x00 };
  pn5180_command(cmd, 2, readBuffer, len);

  return readBuffer;
}

/*
 * Get TRANSCEIVE_STATE from RF_STATUS register
 */

PN5180TransceiveState_t getTransceiveState() {
  uint32_t rfStatus;
  esp_err_t ret;
  ret = pn5180_readRegister(PN5180_RF_STATUS, &rfStatus);
  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "Error reading RF_STATUS register");
    return PN5180_TS_Idle;
  }

  /*
   * TRANSCEIVE_STATEs:
   *  0 - idle
   *  1 - wait transmit
   *  2 - transmitting
   *  3 - wait receive
   *  4 - wait for data
   *  5 - receiving
   *  6 - loopback
   *  7 - reserved
   */
  ESP_LOGD(TAG, "getTransceiveState: rfStatus=%ld", rfStatus);
  uint8_t state = ((rfStatus >> 24) & 0x07);
  return state;
}

/* prepare LPCD registers */
esp_err_t pn5180_prepareLPCD() {
  //=======================================LPCD CONFIG================================================================================

  uint8_t data[255];
  uint8_t response[256];
    //1. Set Fieldon time                                           LPCD_FIELD_ON_TIME (0x36)
  uint8_t fieldOn = 0xF0;//0x## -> ##(base 10) x 8μs + 62 μs
  data[0] = fieldOn;
  pn5180_writeEEprom(0x36, data, 1);
  pn5180_readEEprom(0x36, response, 1);
  fieldOn = response[0];

    //2. Set threshold level                                         AGC_LPCD_THRESHOLD @ EEPROM 0x37
  uint8_t threshold = 0x03;
  data[0] = threshold;
  pn5180_writeEEprom(0x37, data, 1);
  pn5180_readEEprom(0x37, response, 1);
  threshold = response[0];

  //3. Select LPCD mode                                               LPCD_REFVAL_GPO_CONTROL (0x38)
  uint8_t lpcdMode = 0x01; // 1 = LPCD SELF CALIBRATION 
                           // 0 = LPCD AUTO CALIBRATION (this mode does not work, should look more into it, no reason why it shouldn't work)
  data[0] = lpcdMode;
  pn5180_writeEEprom(0x38, data, 1);
  pn5180_readEEprom(0x38, response, 1);
  lpcdMode = response[0];
  
  // LPCD_GPO_TOGGLE_BEFORE_FIELD_ON (0x39)
  uint8_t beforeFieldOn = 0xF0; 
  data[0] = beforeFieldOn;
  pn5180_writeEEprom(0x39, data, 1);
  pn5180_readEEprom(0x39, response, 1);
  beforeFieldOn = response[0];
  
  // LPCD_GPO_TOGGLE_AFTER_FIELD_ON (0x3A)
  uint8_t afterFieldOn = 0xF0; 
  data[0] = afterFieldOn;
  pn5180_writeEEprom(0x3A, data, 1);
  pn5180_readEEprom(0x3A, response, 1);
  afterFieldOn = response[0];
  vTaskDelay(pdMS_TO_TICKS(100));
  return ESP_OK;
}

/* switch the mode to LPCD (low power card detection)
 * Parameter 'wakeupCounterInMs' must be in the range from 0x0 - 0xA82
 * max. wake-up time is 2960 ms.
 */
esp_err_t pn5180_switchToLPCD(uint16_t wakeupCounterInMs) {
  // clear all IRQ flags
  pn5180_clearIRQStatus(0xffffffff); 
  // enable only LPCD and general error IRQ
  pn5180_writeRegister(PN5180_IRQ_ENABLE, PN5180_LPCD_IRQ_STAT | PN5180_GENERAL_ERROR_IRQ_STAT);  
  // switch mode to LPCD 
  uint8_t cmd[4] = { PN5180_SWITCH_MODE, 0x01, (uint8_t)(wakeupCounterInMs & 0xFF), (uint8_t)((wakeupCounterInMs >> 8U) & 0xFF) };
  return pn5180_command(cmd, sizeof(cmd), 0, 0);
}

void printIRQStatus(const char* tag, uint32_t irqStatus) {
  char states[255] = "";
  if (irqStatus & (1<< 0)) strcat(states,"RX ");
  if (irqStatus & (1<< 1)) strcat(states,"TX ");
  if (irqStatus & (1<< 2)) strcat(states,"IDLE ");
  if (irqStatus & (1<< 3)) strcat(states,"MODE_DETECTED ");
  if (irqStatus & (1<< 4)) strcat(states,"CARD_ACTIVATED ");
  if (irqStatus & (1<< 5)) strcat(states,"STATE_CHANGE ");
  if (irqStatus & (1<< 6)) strcat(states,"RFOFF_DET ");
  if (irqStatus & (1<< 7)) strcat(states,"RFON_DET ");
  if (irqStatus & (1<< 8)) strcat(states,"TX_RFOFF ");
  if (irqStatus & (1<< 9)) strcat(states,"TX_RFON ");
  if (irqStatus & (1<<10)) strcat(states,"RF_ACTIVE_ERROR ");
  if (irqStatus & (1<<11)) strcat(states,"TIMER0 ");
  if (irqStatus & (1<<12)) strcat(states,"TIMER1 ");
  if (irqStatus & (1<<13)) strcat(states,"TIMER2 ");
  if (irqStatus & (1<<14)) strcat(states,"RX_SOF_DET ");
  if (irqStatus & (1<<15)) strcat(states,"RX_SC_DET ");
  if (irqStatus & (1<<16)) strcat(states,"TEMPSENS_ERROR ");
  if (irqStatus & (1<<17)) strcat(states,"GENERAL_ERROR ");
  if (irqStatus & (1<<18)) strcat(states,"HV_ERROR ");
  if (irqStatus & (1<<19)) strcat(states,"LPCD ");
  ESP_LOGI(tag,"IRQ_Status: %s",states);
}

// Publicly available from https://www.kartenbezogene-identifier.de/de/chiphersteller-kennungen.html
const char manufacturerCode[110][100] = {
  "Unknown",
  "Motorola (UK)",
  "STMicroelectronics SA (FR)",
  "Hitachi Ltd (JP)",
  "NXP Semiconductors (DE)",
  "Infineon Technologies AG (DE)",
  "Cylink (US)",
  "Texas Instruments (FR)",
  "Fujitsu Limited (JP)",
  "Matsushita Electronics Corporation, Semiconductor Company (JP)",
  "NEC (JP)",
  "Oki Electric Industry Co Ltd (JP)",
  "Toshiba Corp (JP)",
  "Mitsubishi Electric Corp (JP)",
  "Samsung Electronics Co Ltd (KR)",
  "Hynix (KR)",
  "LG-Semiconductors Co Ltd (KR)",
  "Emosyn-EM Microelectronics (US)",
  "INSIDE Technology (FR)",
  "ORGA Kartensysteme GmbH (DE)",
  "Sharp Corporation (JP)",
  "ATMEL (FR)",
  "EM Microelectronic-Marin (CH)",
  "SMARTRAC TECHNOLOGY GmbH (DE)",
  "ZMD AG (DE)",
  "XICOR Inc (US)",
  "Sony Corporation (JP)",
  "Malaysia Microelectronic Solutions Sdn Bhd (MY)",
  "Emosyn (US)",
  "Shanghai Fudan Microelectronics Co Ltd (CN)",
  "Magellan Technology Pty Limited (AU)",
  "Melexis NV BO (CH)",
  "Renesas Technology Corp (JP)",
  "TAGSYS (FR)",
  "Transcore (US)",
  "Shanghai Belling Corp Ltd (CN)",
  "Masktech Germany GmbH (DE)",
  "Innovision Research and Technology Plc (UK)",
  "Hitachi ULSI Systems Co Ltd (JP)",
  "Yubico AB (SE)",
  "Ricoh (JP)",
  "ASK (FR)",
  "Unicore Microsystems LLC (RU)",
  "Dallas semiconductor/Maxim (US)",
  "Impinj Inc (US)",
  "RightPlug Alliance (US)",
  "Broadcom Corporation (US)",
  "MStar Semiconductor Inc (TW)",
  "BeeDar Technology Inc (US)",
  "RFIDsec (DK)",
  "Schweizer Electronic AG (DE)",
  "AMIC Technology Corp (TW)",
  "Mikron JSC (RU)",
  "Fraunhofer Institute for Photonic Microsystems (DE)",
  "IDS Microship AG (CH)",
  "Kovio (US)",
  "HMT Microelectronic Ltd (CH)",
  "Silicon Craft Technology (TH)",
  "Advanced Film Device Inc. (JP)",
  "Nitecrest Ltd (UK)",
  "Verayo Inc. (US)",
  "HID Global (US)",
  "Productivity Engineering Gmbh (DE)",
  "Austriamicrosystems AG (reserved) (AT)",
  "Gemalto SA (FR)",
  "Renesas Electronics Corporation (JP)",
  "3Alogics Inc (KR)",
  "Top TroniQ Asia Limited (Hong Kong)",
  "GenTag Inc (USA)",
  "Invengo Information Technology Co. Ltd (CN)",
  "Guangzhou Sysur Microelectronics, Inc (CN)",
  "CEITEC S.A. (BR)",
  "Shanghai Quanray Electronics Co. Ltd. (CN)",
  "MediaTek Inc (TW)",
  "Angstrem PJSC (RU)",
  "Celisic Semiconductor (Hong Kong) Limited (CN)",
  "LEGIC Identsystems AG (CH)",
  "Balluff GmbH (DE)",
  "Oberthur Technologies (FR)",
  "Silterra Malaysia Sdn. Bhd. (MY)",
  "DELTA Danish Electronics, Light & Acoustics (DK)",
  "Giesecke & Devrient GmbH (DE)",
  "Shenzhen China Vision Microelectronics Co., Ltd. (CN)",
  "Shanghai Feiju Microelectronics Co. Ltd. (CN)",
  "Intel Corporation (US)",
  "Microsensys GmbH (DE)",
  "Sonix Technology Co., Ltd. (TW)",
  "Qualcomm Technologies Inc (US)",
  "Realtek Semiconductor Corp (TW)",
  "Freevision Technologies Co. Ltd (CN)",
  "Giantec Semiconductor Inc. (CN)",
  "JSC Angstrem-T (RU)",
  "STARCHIP France",
  "SPIRTECH (FR)",
  "GANTNER Electronic GmbH (AT)",
  "Nordic Semiconductor (NO)",
  "Verisiti Inc (US)",
  "Wearlinks Technology Inc. (CN)",
  "Userstar Information Systems Co., Ltd (TW)",
  "Pragmatic Printing Ltd. (UK)",
  "Associacao do Laboratorio de Sistemas Integraveis Tecnologico - LSI-TEC (BR)",
  "Tendyron Corporation (CN)",
  "MUTO Smart Co., Ltd.(KR)",
  "ON Semiconductor (US)",
  "TÜBİTAK BİLGEM (TR)",
  "Huada Semiconductor Co., Ltd (CN)",
  "SEVENEY (FR)",
  "ISSM (FR)",
  "Wisesec Ltd (IL)",
  "Holtek (TW)"
};