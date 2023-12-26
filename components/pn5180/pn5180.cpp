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
//#define DEBUG 1
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <map>
#include <string>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "pn5180.hpp"

SemaphoreHandle_t nfc_task_sem;
SemaphoreHandle_t nfc_cont_sem;
QueueHandle_t nfc_task_queue;
static const char* TAG = "pn5180.cpp";

// Publicly available from https://www.kartenbezogene-identifier.de/de/chiphersteller-kennungen.html
const char manufacturerCode[110][100] = {
  "Unknown", "Motorola (UK)", "STMicroelectronics SA (FR)", "Hitachi Ltd (JP)", "NXP Semiconductors (DE)",
  "Infineon Technologies AG (DE)", "Cylink (US)", "Texas Instruments (FR)", "Fujitsu Limited (JP)", "Matsushita Electronics Corporation, Semiconductor Company (JP)",
  "NEC (JP)", "Oki Electric Industry Co Ltd (JP)", "Toshiba Corp (JP)", "Mitsubishi Electric Corp (JP)", "Samsung Electronics Co Ltd (KR)", "Hynix (KR)",
  "LG-Semiconductors Co Ltd (KR)", "Emosyn-EM Microelectronics (US)", "INSIDE Technology (FR)", "ORGA Kartensysteme GmbH (DE)", "Sharp Corporation (JP)",
  "ATMEL (FR)", "EM Microelectronic-Marin (CH)", "SMARTRAC TECHNOLOGY GmbH (DE)", "ZMD AG (DE)","XICOR Inc (US)",
  "Sony Corporation (JP)","Malaysia Microelectronic Solutions Sdn Bhd (MY)","Emosyn (US)","Shanghai Fudan Microelectronics Co Ltd (CN)","Magellan Technology Pty Limited (AU)",
  "Melexis NV BO (CH)","Renesas Technology Corp (JP)","TAGSYS (FR)","Transcore (US)","Shanghai Belling Corp Ltd (CN)",
  "Masktech Germany GmbH (DE)","Innovision Research and Technology Plc (UK)","Hitachi ULSI Systems Co Ltd (JP)","Yubico AB (SE)","Ricoh (JP)",
  "ASK (FR)","Unicore Microsystems LLC (RU)","Dallas semiconductor/Maxim (US)","Impinj Inc (US)","RightPlug Alliance (US)",
  "Broadcom Corporation (US)","MStar Semiconductor Inc (TW)","BeeDar Technology Inc (US)","RFIDsec (DK)","Schweizer Electronic AG (DE)",
  "AMIC Technology Corp (TW)","Mikron JSC (RU)","Fraunhofer Institute for Photonic Microsystems (DE)","IDS Microship AG (CH)","Kovio (US)",
  "HMT Microelectronic Ltd (CH)","Silicon Craft Technology (TH)","Advanced Film Device Inc. (JP)","Nitecrest Ltd (UK)","Verayo Inc. (US)",
  "HID Global (US)","Productivity Engineering Gmbh (DE)","Austriamicrosystems AG (reserved) (AT)","Gemalto SA (FR)","Renesas Electronics Corporation (JP)",
  "3Alogics Inc (KR)","Top TroniQ Asia Limited (Hong Kong)","GenTag Inc (USA)","Invengo Information Technology Co. Ltd (CN)","Guangzhou Sysur Microelectronics, Inc (CN)",
  "CEITEC S.A. (BR)","Shanghai Quanray Electronics Co. Ltd. (CN)","MediaTek Inc (TW)","Angstrem PJSC (RU)","Celisic Semiconductor (Hong Kong) Limited (CN)",
  "LEGIC Identsystems AG (CH)","Balluff GmbH (DE)","Oberthur Technologies (FR)","Silterra Malaysia Sdn. Bhd. (MY)","DELTA Danish Electronics, Light & Acoustics (DK)",
  "Giesecke & Devrient GmbH (DE)","Shenzhen China Vision Microelectronics Co., Ltd. (CN)","Shanghai Feiju Microelectronics Co. Ltd. (CN)","Intel Corporation (US)","Microsensys GmbH (DE)",
  "Sonix Technology Co., Ltd. (TW)","Qualcomm Technologies Inc (US)","Realtek Semiconductor Corp (TW)","Freevision Technologies Co. Ltd (CN)","Giantec Semiconductor Inc. (CN)",
  "JSC Angstrem-T (RU)","STARCHIP France","SPIRTECH (FR)","GANTNER Electronic GmbH (AT)","Nordic Semiconductor (NO)",
  "Verisiti Inc (US)","Wearlinks Technology Inc. (CN)","Userstar Information Systems Co., Ltd (TW)","Pragmatic Printing Ltd. (UK)","Associacao do Laboratorio de Sistemas Integraveis Tecnologico - LSI-TEC (BR)",
  "Tendyron Corporation (CN)","MUTO Smart Co., Ltd.(KR)","ON Semiconductor (US)","TÜBİTAK BİLGEM (TR)","Huada Semiconductor Co., Ltd (CN)",
  "SEVENEY (FR)","ISSM (FR)","Wisesec Ltd (IL)","Holtek (TW)"
};

const char afi_string[14][18] = {
  "All families","Transport","Financial",
  "Identification","Telecommunication","Medical",
  "Multimedia","Gaming","Data storage",
  "Item management","Express parcels","Postal services",
  "Airline bags","Unknown"
};

// https://www.nxp.com/docs/en/application-note/AN10833.pdf page 11/18
typedef struct {
  uint16_t numBlocks;
  uint16_t blockSize;
  uint16_t startBlock;
  uint16_t endBlock;
} MifareType_t;

/*
 * SAK = 0x08
 * https://www.nxp.com/docs/en/data-sheet/MF1S50YYX_V1.pdf
 */
const MifareType_t MifareClassic1K = {
  .numBlocks = 64,
  .blockSize = 16,
  .startBlock = 4,
  .endBlock = 63
};

/*
 * SAK = 0x18
 * https://www.nxp.com/docs/en/data-sheet/MF1S70YYX_V1.pdf
 */
const MifareType_t MifareClassic4K = {
  .numBlocks = 256,
  .blockSize = 16,
  .startBlock = 4,
  .endBlock = 255
};

/*
 * SAK = 0x00
 * https://www.nxp.com/docs/en/data-sheet/MF0ULX1.pdf
 * 80 + 164 byte variants, just default to smaller to be safe
 */
const MifareType_t MifareUltralight = {
  .numBlocks = 20,
  .blockSize = 4,
  .startBlock = 4,
  .endBlock = 15
};

std::map<uint8_t, MifareType_t> mifare = {
  { 0x00, MifareUltralight },
  { 0x08, MifareClassic1K },
  { 0x18, MifareClassic4K },
};

static const uint8_t mifare_info[] = {

};
pn5180::pn5180(uint8_t MISOpin, uint8_t MOSIpin, uint8_t CLKpin, uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin, spi_host_device_t host) :
  PN5180_MISO((gpio_num_t)MISOpin),
  PN5180_MOSI((gpio_num_t)MOSIpin),
  PN5180_CLK((gpio_num_t)CLKpin),
  PN5180_NSS((gpio_num_t)SSpin),
  PN5180_BUSY((gpio_num_t)BUSYpin),
  PN5180_RST((gpio_num_t)RSTpin),
  spi_host(host)
{
  /*
   * 11.4.1 Physical Host Interface
   * The interface of the PN5180 to a host microcontroller is based on a SPI interface,
   * extended by signal line BUSY. The maximum SPI speed is 7 Mbps and fixed to CPOL
   * = 0 and CPHA = 0.
   */
  // Settings for PN5180: 7Mbps, MSB first, SPI_MODE0 (CPOL=0, CPHA=0)
  // Configure physical bus settings for pn5180
  pn5180_buscfg={
      .mosi_io_num = (gpio_num_t)MOSIpin,
      .miso_io_num = (gpio_num_t)MISOpin,
      .sclk_io_num = (gpio_num_t)CLKpin,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .data4_io_num = -1,
      .data5_io_num = -1,
      .data6_io_num = -1,
      .data7_io_num = -1,
      /*
       * The number of bytes within the ‘Tx Data’ field must be in the range from 1 to 260
       * The number of bytes within the ‘Rx Data’ field range from 0 to 508
       * The host controls the number of bytes to be read via the SPI interface
       */
      .max_transfer_sz = 508,
  };

  // Configure software settings for pn5180
  pn5180_devcfg={
      .mode = 0,
      .clock_speed_hz = 7000000,
      .spics_io_num = (gpio_num_t)SSpin,
      .queue_size = 7,
      .pre_cb = NULL,
      .post_cb = NULL,
  };
}

uint8_t pn5180::getManufacturer(void) const { return manufacturer; }
uint8_t pn5180::getType(void) const { return type; }
uint8_t pn5180::getDsfid(void) const { return dsfid; }
uint8_t pn5180::getAfi(void) const { return afi; }
uint8_t pn5180::getICRef(void) const { return ic_ref; }
uint8_t pn5180::getBlockSize(void) const { return blockSize; }
uint8_t pn5180::getNumBlocks(void) const { return numBlocks; }
//uint8_t pn5180::getUID(void) const { return uid; }

///////////////
// Functions //
///////////////
void pn5180::begin(void){
  gpio_config_t io_conf = {};

  // NSS and Reset are outputs
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pin_bit_mask = 0;
  io_conf.pin_bit_mask |= (1ULL << PN5180_NSS);
  io_conf.pin_bit_mask |= (1ULL << PN5180_RST);
  gpio_config(&io_conf);

  // BUSY is input
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = 0;
  io_conf.pin_bit_mask |= (1ULL << PN5180_BUSY);
  gpio_config(&io_conf);

  gpio_set_level(PN5180_NSS, 1); // Deselect device
  gpio_set_level(PN5180_RST, 1); // Prevent reset

  // Apply settings
  ESP_LOGI(TAG, "init: Initializing bus SPI...");
  if(ESP_ERR_INVALID_STATE == spi_bus_initialize(spi_host, &pn5180_buscfg, SPI3_HOST)){
    ESP_LOGD(TAG, "SPI host %d already configured and in use, continuing...", spi_host);
  };
  ESP_ERROR_CHECK_WITHOUT_ABORT(spi_bus_add_device(spi_host, &pn5180_devcfg, &dev));
  ESP_LOGD(TAG, "init: Bus SPI Initialized");

  // Reset device
  reset();

  // PN5180 Product version
  readEEprom(PN5180_PRODUCT_VERSION, product, 2);
  if(0xff == product[1]){
    ESP_LOGE(TAG, "Initialization failed. Reset to restart.");
    while(1) vTaskDelay(portMAX_DELAY);
  }
  ESP_LOGD(TAG,"Product version: %d.%d",product[1],product[0]);

  
  // PN5180 Firmware version
  readEEprom(PN5180_FIRMWARE_VERSION, firmware, 2);
  ESP_LOGD(TAG,"Firmware version: %d.%d",firmware[1],firmware[0]);

  // PN5180 EEPROM version
  readEEprom(PN5180_EEPROM_VERSION, eeprom, 2);
  ESP_LOGD(TAG,"EEPROM version: %d.%d",eeprom[1],eeprom[0]);
}

void pn5180::end(void){
}

esp_err_t pn5180::reset(void) {
  uint32_t retries = 10;
  gpio_set_level(PN5180_RST, 0);
  vTaskDelay(pdMS_TO_TICKS(1));
  gpio_set_level(PN5180_RST, 1);
  while((PN5180_IDLE_IRQ_STAT & getIRQStatus()) == 0 && retries > 0){
    vTaskDelay(pdMS_TO_TICKS(10));
    retries--;
  }
  if((PN5180_IDLE_IRQ_STAT & getIRQStatus()) == 0){
    ESP_LOGE(TAG, "reset: Timeout waiting for IRQ state IDLE");
    return ESP_ERR_TIMEOUT;
  }
  return ESP_OK;
}

/*
 * WRITE_REGISTER - 0x00
 */
esp_err_t pn5180::writeRegister(uint8_t reg, uint32_t value) {
  uint8_t *p = (uint8_t*)&value;
  uint8_t buf[6] = { PN5180_WRITE_REGISTER, reg, p[0], p[1], p[2], p[3] };
  return transceiveCommand(buf, 6, 0, 0);
}

/*
 * WRITE_REGISTER_OR_MASK - 0x01
 */
esp_err_t pn5180::writeRegisterOrMask(uint8_t reg, uint32_t mask) {
  uint8_t *p = (uint8_t*)&mask;
  uint8_t buf[6] = { PN5180_WRITE_REGISTER_OR_MASK, reg, p[0], p[1], p[2], p[3] };
  return transceiveCommand(buf, 6, 0, 0);
}

/*
 * WRITE_REGISTER_AND_MASK - 0x02
 */
esp_err_t pn5180::writeRegisterAndMask(uint8_t reg, uint32_t mask) {
  uint8_t *p = (uint8_t*)&mask;
  uint8_t buf[6] = { PN5180_WRITE_REGISTER_AND_MASK, reg, p[0], p[1], p[2], p[3] };
  return transceiveCommand(buf, 6, 0, 0);
}

/*
 * WRITE_REGISTER MULTIPLE - 0x03
 */
esp_err_t pn5180::writeRegisterMultiple(uint8_t *reg, uint8_t *action, uint8_t len, uint32_t *value) {
  if(len > 42){
    ESP_LOGE(TAG, "writeRegisterMultiple: Max register writes limited to 42");
    return ESP_ERR_INVALID_SIZE;
  }
  if(!len){
    ESP_LOGE(TAG, "writeRegisterMultiple: Invalid length");
    return ESP_ERR_INVALID_SIZE;
  }
  
  uint8_t* cmd = (uint8_t*)heap_caps_malloc((len*6)+1, MALLOC_CAP_8BIT);
  if(cmd == NULL){
    ESP_LOGE(TAG, "writeRegisterMultiple: Failed malloc for cmd");
    return ESP_ERR_NO_MEM;
  }
  cmd[0] = PN5180_WRITE_REGISTER_MULTIPLE;
  /*
   * Command Structure:
   * 0x03, Write1, Write2, Write3, ....
   * Write struct: {Register, AND/OR/WRITE, Value}
   * Value written LSBFIRST - value[0], value[1], value[2], value[3] 
   */
  for(int i=0; i<len; i++){
    uint8_t *p = (uint8_t*)&value[i];
    uint8_t pos = i*6;
    cmd[pos+1] = reg[i];
    cmd[pos+2] = action[i];
    cmd[pos+3] = p[0];
    cmd[pos+4] = p[1];
    cmd[pos+5] = p[2];
    cmd[pos+6] = p[3];
  }
  esp_err_t ret = transceiveCommand(cmd, (len*6)+1, 0, 0);
  heap_caps_free(cmd);

  return ret;
}

/*
 * READ_REGISTER - 0x04
 */
esp_err_t pn5180::readRegister(uint8_t reg, uint32_t *value) {
  uint8_t cmd[2] = { PN5180_READ_REGISTER, reg };
  return transceiveCommand(cmd, 2, (uint8_t*)value, 4);
}

/*
 * READ_REGISTER_MULTIPLE - 0x05
 */
esp_err_t pn5180::readRegisterMultiple(uint8_t *reg, uint8_t len, uint32_t *value) {
  if(len > 18){
    ESP_LOGE(TAG, "readRegisterMultiple: Max register reads limited to 18");
    return ESP_ERR_INVALID_SIZE;
  }
  if(!len){
    ESP_LOGE(TAG, "readRegisterMultiple: Invalid length");
    return ESP_ERR_INVALID_SIZE;
  }
  
  uint8_t* cmd = (uint8_t*)heap_caps_malloc(len+1, MALLOC_CAP_8BIT);
  if(cmd == NULL){
    ESP_LOGE(TAG, "readRegisterMultiple: Failed malloc for cmd");
    return ESP_ERR_NO_MEM;
  }
  cmd[0] = PN5180_READ_REGISTER_MULTIPLE;
  memcpy(cmd+1, reg, len);
  esp_err_t ret = transceiveCommand(cmd, len+1, (uint8_t*)value, len*4);
  heap_caps_free(cmd);

  return ret;
}

/*
 * WRITE_EEPROM - 0x06
 */
esp_err_t pn5180::writeEEprom(uint8_t addr, uint8_t *buffer, uint16_t len) {
  if ((addr > 254) || ((addr+len) > 254)) {
    ESP_LOGE(TAG, "writeEEprom: Size of tx buffer cannot exceed 253 bytes");
    return ESP_ERR_INVALID_SIZE;
  }
	uint8_t cmd[len + 2];
	cmd[0] = PN5180_WRITE_EEPROM;
	cmd[1] = addr;
	for (int i = 0; i < len; i++) cmd[2 + i] = buffer[i];
	return transceiveCommand(cmd, len + 2, 0, 0);
}

/*
 * READ_EEPROM - 0x07
 */
esp_err_t pn5180::readEEprom(uint8_t addr, uint8_t *buffer, uint16_t len) {
  if ((addr > 254) || ((addr+len) > 254)) {
    ESP_LOGE(TAG, "readEEprom: Size of rx buffer exceeds 253 bytes");
    return ESP_ERR_INVALID_SIZE;
  }
  uint8_t cmd[3] = { PN5180_READ_EEPROM, addr, (uint8_t)(len) };
  return transceiveCommand(cmd, 3, buffer, len);
}

uint32_t pn5180::getIRQStatus() {
  uint32_t irqStatus;
  readRegister(PN5180_IRQ_STATUS, &irqStatus);
  return irqStatus;
}

esp_err_t pn5180::clearIRQStatus(uint32_t irqMask) {
  return writeRegister(PN5180_IRQ_CLEAR, irqMask);
}

/*
 * LOAD_RF_CONFIG - 0x11
 */
esp_err_t pn5180::loadRFConfig(uint8_t txConf, uint8_t rxConf) {
  uint8_t cmd[3] = { PN5180_LOAD_RF_CONFIG, txConf, rxConf };
  return transceiveCommand(cmd, 3, 0, 0);
}

/*
 * RF_ON - 0x16
 */
esp_err_t pn5180::setRF_on() {
  uint8_t cmd[2] = { PN5180_RF_ON, 0x00 };
  transceiveCommand(cmd, 2, 0, 0);

  uint8_t retries = 50;
  while (0 == (PN5180_TX_RFON_IRQ_STAT & getIRQStatus()) && retries > 0) {   // wait for RF field to set up (max 500ms)
    vTaskDelay(pdMS_TO_TICKS(10));
	  retries--;
  }
  ESP_LOGD(TAG, "setRF_on: IRQ State after set - %ld", getIRQStatus());
  if(0 == (PN5180_TX_RFON_IRQ_STAT & getIRQStatus())){
    ESP_LOGE(TAG, "setRF_on: Failed to detect IRQ state TX_RFON");
    return ESP_FAIL;
  }

  clearIRQStatus(PN5180_TX_RFON_IRQ_STAT);
  return ESP_OK;
}

/*
 * RF_OFF - 0x17
 */
esp_err_t pn5180::setRF_off() {

  uint8_t cmd[2] = { PN5180_RF_OFF, 0x00 };
  transceiveCommand(cmd, 2, 0, 0);

  uint8_t retries = 50;
  while (0 == (PN5180_TX_RFOFF_IRQ_STAT & getIRQStatus()) && retries > 0) {   // wait for RF field to set up (max 500ms)
    vTaskDelay(pdMS_TO_TICKS(10));
	  retries--;
  }
  if(0 == (PN5180_TX_RFOFF_IRQ_STAT & getIRQStatus())) {
    ESP_LOGE(TAG, "setRF_off: Failed to turn off RF, was it already off?");
    printIRQStatus(TAG, getIRQStatus());
    return ESP_FAIL;
  }

  clearIRQStatus(PN5180_TX_RFOFF_IRQ_STAT);
  return ESP_OK;
};

esp_err_t pn5180::setupRF(uint8_t protocol) {
  esp_err_t ret;
  switch(protocol){
    case 1:
      ESP_LOGD(TAG, "setupRF: Loading RF-Configuration for ISO14443...");
      ret = loadRFConfig(0x00, 0x80); // PI-106, PI-106
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "setupRF: Error loading ISO14443 RF Configuration");
        return ret;
      }
      break;
    case 2:
      ESP_LOGD(TAG, "setupRF: Loading RF-Configuration for ISO15693...");
      ret = loadRFConfig(0x0D, 0x8D); // ASK100
      if(ret != ESP_OK){
        ESP_LOGE(TAG, "setupRF: Error loading ISO15693 RF-Configuration");
        return ret;
      }
      break;
    case 3:
      ESP_LOGD(TAG, "setupRF: Loading RF-Configuration for ISO18003...");
      ret = loadRFConfig(0x0F, 0x8F); // Tari 18.88, Manchester 424_4
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "setupRF: Error loading ISO18003 RF Configuration");
        return ret;
      }
      break;
    default:
      ESP_LOGE(TAG, "setupRF: Unknown protocol passed to function");
      break;
  }
  ESP_LOGD(TAG, "setupRF: Turning ON RF field...");
  ret = setRF_on();
  if(ret != ESP_OK){
    ESP_LOGE(TAG, "setupRF: Error turning on RF");
    return ret;
  }
  return ESP_OK;
}

/*
 * SEND_DATA - 0x09
 */
esp_err_t pn5180::sendData(uint8_t *data, uint16_t len, uint8_t validBits) {
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

  uint8_t regs[2] = {PN5180_SYSTEM_CONFIG, PN5180_SYSTEM_CONFIG};
  uint8_t cmd[2] = {0x03, 0x02};
  uint32_t value[2] = {0xFFFFFFF8, 0x00000003};
  writeRegisterMultiple(regs,cmd,2,value);

  PN5180TransceiveState_t state = getTransceiveState();
  ESP_LOGD(TAG,"sendData: state=%d",(uint8_t)(state));
  if (PN5180_TS_WaitTransmit != state){
    ESP_LOGE(TAG, "sendData: TransceiveState not WaitTransmit");
    return ESP_ERR_INVALID_STATE;
  }

  return transceiveCommand(buffer, len+2, 0, 0);
}

/*
 * READ_DATA - 0x0A
 */
uint8_t* pn5180::readData(int len) {
  if (len > 508) return 0L;

  uint8_t cmd[2] = { PN5180_READ_DATA, 0x00 };
  transceiveCommand(cmd, 2, readBuffer, len);

  return readBuffer;
}

/*
 * Get TRANSCEIVE_STATE from RF_STATUS register
 */

PN5180TransceiveState_t pn5180::getTransceiveState() {
  uint32_t rfStatus;
  esp_err_t ret;
  ret = readRegister(PN5180_RF_STATUS, &rfStatus);
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
  PN5180TransceiveState_t state = (PN5180TransceiveState_t)((rfStatus >> 24) & 0x07);
  return state;
}

/* prepare LPCD registers */
esp_err_t pn5180::prepareLPCD() {
  //=======================================LPCD CONFIG================================================================================

  uint8_t data[255];
  uint8_t response[256];
    //1. Set Fieldon time                                           LPCD_FIELD_ON_TIME (0x36)
  uint8_t fieldOn = 0xF0;//0x## -> ##(base 10) x 8μs + 62 μs
  data[0] = fieldOn;
  writeEEprom(0x36, data, 1);
  readEEprom(0x36, response, 1);
  fieldOn = response[0];

    //2. Set threshold level                                         AGC_LPCD_THRESHOLD @ EEPROM 0x37
  uint8_t threshold = 0x03;
  data[0] = threshold;
  writeEEprom(0x37, data, 1);
  readEEprom(0x37, response, 1);
  threshold = response[0];

  //3. Select LPCD mode                                               LPCD_REFVAL_GPO_CONTROL (0x38)
  uint8_t lpcdMode = 0x01; // 1 = LPCD SELF CALIBRATION 
                           // 0 = LPCD AUTO CALIBRATION (this mode does not work, should look more into it, no reason why it shouldn't work)
  data[0] = lpcdMode;
  writeEEprom(0x38, data, 1);
  readEEprom(0x38, response, 1);
  lpcdMode = response[0];
  
  // LPCD_GPO_TOGGLE_BEFORE_FIELD_ON (0x39)
  uint8_t beforeFieldOn = 0xF0; 
  data[0] = beforeFieldOn;
  writeEEprom(0x39, data, 1);
  readEEprom(0x39, response, 1);
  beforeFieldOn = response[0];
  
  // LPCD_GPO_TOGGLE_AFTER_FIELD_ON (0x3A)
  uint8_t afterFieldOn = 0xF0; 
  data[0] = afterFieldOn;
  writeEEprom(0x3A, data, 1);
  readEEprom(0x3A, response, 1);
  afterFieldOn = response[0];
  vTaskDelay(pdMS_TO_TICKS(100));
  return ESP_OK;
}

/* switch the mode to LPCD (low power card detection)
 * Parameter 'wakeupCounterInMs' must be in the range from 0x0 - 0xA82
 * max. wake-up time is 2960 ms.
 */
esp_err_t pn5180::switchToLPCD(uint16_t wakeupCounterInMs) {
  // clear all IRQ flags
  clearIRQStatus(0xffffffff); 
  // enable only LPCD and general error IRQ
  writeRegister(PN5180_IRQ_ENABLE, PN5180_LPCD_IRQ_STAT | PN5180_GENERAL_ERROR_IRQ_STAT);  
  // switch mode to LPCD 
  uint8_t cmd[4] = { PN5180_SWITCH_MODE, 0x01, (uint8_t)(wakeupCounterInMs & 0xFF), (uint8_t)((wakeupCounterInMs >> 8U) & 0xFF) };
  return transceiveCommand(cmd, sizeof(cmd), 0, 0);
}

void pn5180::printIRQStatus(const char* tag, uint32_t irqStatus) {
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

void pn5180::printRfStatus(const char* tag, uint32_t rfStatus) {
  char states[255] = "";
  if (rfStatus & (1<<10)) strcat(states,"RX_ACTIVE ");
  if (rfStatus & (1<<11)) strcat(states,"TX_ACTIVE ");
  if (rfStatus & (1<<12)) strcat(states,"RX_ENABLE ");
  if (rfStatus & (1<<16)) strcat(states,"RF_DET ");
  if (rfStatus & (1<<17)) strcat(states,"TX_RF_STATUS ");
  if (rfStatus & (1<<18)) strcat(states,"CRC_OK ");
  if (rfStatus & (1<<19)) strcat(states,"DPLL_ENABLE ");
  char buf[255];
  sprintf(buf, "TRANSCEIVE_STATE=%d", (uint8_t)((rfStatus >> 24) & 0x07));
  strcat(states, buf);
  ESP_LOGI(tag,"RF_Status: %s",states);
}

ISO15693ErrorCode_t pn5180::iso15693Poll(){
  poll.numCard = 0;
  uint8_t numCol = 0;
  uint32_t collision[16] = {0};
  uint8_t maskLen = 0;

  do{
    if(numCol > 0){
      uint32_t mask = collision[0];
      do{
        mask >>= 4L;
        maskLen++;
      }while(mask > 0);
    } 
    uint8_t *readBuffer;
    uint8_t *p = (uint8_t*)&(collision[0]);
    //                      Flags,  CMD,
    uint8_t inventory[7] = { 0x06, 0x01, (uint8_t)(maskLen*4), p[0], p[1], p[2], p[3] };
    //                         |\- inventory flag + high data rate
    //                         \-- 16 slots: upto 16 cards, no AFI field present
    uint8_t cmdLen = 3 + (maskLen / 2) + (maskLen % 2);
    ESP_LOGD(TAG, "mask=%ld, maskLen=%d, cmdLen=%d", collision[0], maskLen, cmdLen);
    setupRF(ISO15693);               // 1. 2. Load ISO15693 config, RF on
    clearIRQStatus(0x000FFFFF);      // 3. Clear all IRQ_STATUS flags
    sendData(inventory, cmdLen, 0);  // 4. 5. 6. Idle/StopCom Command, Transceive Command, Inventory command

    for(int slot=0; slot<16; slot++){                                   // 7. Loop to check 16 time slots for data
      vTaskDelay(pdMS_TO_TICKS(15));
      uint32_t regStatus[2];
      uint8_t readRegs[2] = {PN5180_RX_STATUS, PN5180_IRQ_STATUS};
      readRegisterMultiple(readRegs, 2, regStatus);
      uint16_t len = (uint16_t)(regStatus[0] & 0x000001ff);
      if((regStatus[0] >> 18) & 0x01 && numCol < 16){                      // 7+ Determine if a collision occurred
        if(maskLen > 0) collision[numCol] = collision[0] | (slot << (maskLen * 2));
        else collision[numCol] = slot << (maskLen * 2);                // Yes, store position of collision
        ESP_LOGI(TAG, "Collision detected for UIDs matching %lX starting at LSB", collision[numCol]);
        numCol = numCol + 1;
      }
      else if(!(regStatus[1] & PN5180_RX_IRQ_STAT) || !len){               // 8. Check if a card has responded
        ESP_LOGI(TAG, "getInventoryMultiple: No card in this time slot. State=%ld", regStatus[1]);
      }
      else{
        ESP_LOGI(TAG, "slot=%d, irqStatus: %ld, RX_STATUS: %ld, Response length=%d", slot, regStatus[1], regStatus[0], len);
        readBuffer = readData(len);                                     // 9. Read reception buffer
        if(0L == readBuffer){
          ESP_LOGE(TAG,"getInventoryMultiple: ERROR in readData!");
          return ISO15693_EC_UNKNOWN_ERROR;
        }

        // Record raw UID data                                          // 10. Record all data to Inventory struct
        for (int i=0; i<8; i++) {
          poll.uid[poll.numCard][i] = readBuffer[2+i];
        }

        ESP_LOGI(TAG,"getInventory: Response flags: 0x%X, Data Storage Format ID: 0x%X", readBuffer[0], readBuffer[1]);
        poll.numCard++;
      }
      
      if(slot+1 < 16){ // If we have more cards to poll for...
        writeRegisterAndMask(PN5180_TX_CONFIG, 0xFFFFFB3F);  // 11. Next SEND_DATA will only include EOF
        printIRQStatus(TAG, getIRQStatus());
        clearIRQStatus(0x000FFFFF);                              // 14. Clear all IRQ_STATUS flags
        sendData(inventory, 0, 0);                               // 12. 13. 15. Idle/StopCom Command, Transceive Command, Send EOF
      }
    }
    setRF_off();                                                 // 16. Switch off RF field
    if(numCol) {
      numCol--;
      for(int i=0; i<numCol; i++){
        collision[i] = collision[i+1];
      }
    }
  }while(numCol);
  return ISO15693_EC_OK;
}

PN5180Error_t pn5180::getInventory() {
  setupRF(ISO14443);
  PN5180Error_t state = iso14443poll(ISO14443_REQA);
  if(state){
    clearInfo();
    setRF_off();
    if(PN5180_ERR_NO_CARD != state) reset();
    setupRF(ISO15693);
    state = iso15693PollSingle();
    if(state){
      clearInfo();
    }
    else{
      state = iso15693GetSystemInfo();
    }
  }
  setRF_off();
  return state;
}

PN5180Error_t pn5180::getData(uint8_t blockNo) {
  setupRF(ISO14443);
  PN5180Error_t state = readSingleBlock(ISO14443, 0);
  if(PN5180_ERR_NO_CARD == state){
    setupRF(ISO15693);
    state = readSingleBlock(ISO15693, 0);
  }
  setRF_off();
  //reset();
  return state;
}

PN5180Error_t pn5180::readSingleBlock(uint8_t protocol, uint8_t blockNo) {
  bool success;
  PN5180Error_t state;
  switch(protocol){
    case ISO14443:
      state = iso14443poll(ISO14443_REQA);
      if(state){
        clearInfo();
        return state;
      }
      if(blockNo + startBlock > endBlock){
        ESP_LOGE(TAG, "Chosen block not a valid block");
        return PN5180_ERR_INVALID_PARAM;
      }
      success = mifareAuthenticate(blockNo+startBlock);
      if(!success){
        ESP_LOGE(TAG, "Failed to authenticate Mifare tag!");
        mifareHalt();
        return PN5180_ERR_NO_RESP;
      }
      success = mifareBlockRead(blockNo+startBlock);
      if(!success){
        ESP_LOGE(TAG, "Failed to read Mifare tag!");
        mifareHalt();
        return PN5180_ERR_NO_RESP;
      }
      mifareHalt();
      break;
    case ISO15693:
      // Find UID and info
      state = iso15693PollSingle();
      if(state){
        clearInfo();
        return state;
      }
      //success = iso15693BlockRead(blockNo+startBlock);
      break;
    case ISO18003:
      break;
    default:
      ESP_LOGE(TAG, "readSingleBlock: Unknown protocol selected");
      return PN5180_ERR_UNKNOWN;
      break;
  }
  return PN5180_OK;
}
PN5180Error_t pn5180::getInventoryMultiple() {
  return PN5180_OK;
}
void pn5180::printInfo() {
  ESP_LOGI(TAG, "MemSize=%d BlockSize=%d NumBlocks=%d", blockSize * numBlocks, blockSize, numBlocks);
  ESP_LOGI(TAG, "FirstBlock=%d LastBlock=%d", startBlock, endBlock);
}

void pn5180::clearInfo() {
  memset(uid, 0, sizeof(uid));
  type = blockSize = numBlocks = startBlock = endBlock = 0;
  manufacturer = uidLength = afi = ic_ref = 0;
  dsfid = 0;
}

/*
 * Inventory, code=01
 *
 * Request format: SOF, Req.Flags, Inventory, AFI (opt.), Mask len, Mask value, CRC16, EOF
 * Response format: SOF, Resp.Flags, DSFID, UID, CRC16, EOF
 *
 */
PN5180Error_t pn5180::iso15693PollSingle() {
  //                      Flags,  CMD, maskLen
  uint8_t inventory[3] = { 0x26, 0x01, 0x00 };
  //                         |\- inventory flag + high data rate
  //                         \-- 1 slot: only one card, no AFI field present
  ESP_LOGD(TAG,"getInventory: Get Inventory...");
  clearIRQStatus(0x000FFFFF);      // 3. Clear all IRQ_STATUS flags
  sendData(inventory, 3, 0);  // 4. 5. 6. Idle/StopCom Command, Transceive Command, Inventory command
  vTaskDelay(pdMS_TO_TICKS(20));

  uint32_t regStatus[2];
  uint8_t readRegs[2] = {PN5180_RX_STATUS, PN5180_IRQ_STATUS};
  readRegisterMultiple(readRegs, 2, regStatus);
  uint16_t len = (uint16_t)(regStatus[0] & 0x000001ff);
  ESP_LOGD(TAG, "irqStatus: %ld, RX_STATUS: %ld, Response length=%d", regStatus[1], regStatus[0], len);

  if(!len){
    ESP_LOGD(TAG, "iso15693PollSingle: No card");
    return PN5180_ERR_NO_CARD;
  }
  uint8_t *readBuffer;
  readBuffer = readData(len);

  if(readBuffer[9] != 224){
    ESP_LOGD(TAG, "iso15693PollSingle: UID in unrecognized format! %X should be E0", readBuffer[9]);
    return PN5180_ERR_UNKNOWN; // UIDs always start with E0h (224d)
  }

  // Record raw UID data
  for (int i=0; i<8; i++) {
    uid[i] = readBuffer[2+i];
  }

  /*
   * https://www.nxp.com/docs/en/data-sheet/SL2S2002_SL2S2102.pdf
   *  
   * The 64-bit unique identifier (UID) is programmed during the production process according
   * to ISO/IEC 15693-3 and cannot be changed afterwards.
   * 
   * UID: AA:BB:CCDDDDDDDDDD
   * 
   * AA - Always E0
   * BB - Manufacturer Code (0x04 = NXP Semiconductors)
   * CC - Random ID, sometimes used by manufacturer for Tag Type (0x01 = ICODE SLIX)
   * DDDDDDDDDD - Random ID
   */

  // Record Manufacturer code
  manufacturer = uid[6];

  // Record IC type
  type = uid[5];

  ESP_LOGD(TAG,"iso15693PollSingle: Response flags: 0x%X, Data Storage Format ID: 0x%X", readBuffer[0], readBuffer[1]);
  uidLength = 8;
  return PN5180_OK;
}

/*
 * Get System Information, code=2B
 *
 * Request format: SOF, Req.Flags, GetSysInfo, UID (opt.), CRC16, EOF
 * Response format:
 *  when ERROR flag is set:
 *    SOF, Resp.Flags, ErrorCode, CRC16, EOF
 *
 *     Response Flags:
  *    xxxx.3xx0
  *         |||\_ Error flag: 0=no error, 1=error detected, see error field
  *         \____ Extension flag: 0=no extension, 1=protocol format is extended
  *
  *  If Error flag is set, the following error codes are defined:
  *    01 = The command is not supported, i.e. the request code is not recognized.
  *    02 = The command is not recognized, i.e. a format error occurred.
  *    03 = The option is not supported.
  *    0F = Unknown error.
  *    10 = The specific block is not available.
  *    11 = The specific block is already locked and cannot be locked again.
  *    12 = The specific block is locked and cannot be changed.
  *    13 = The specific block was not successfully programmed.
  *    14 = The specific block was not successfully locked.
  *    A0-DF = Custom command error codes
  *
 *  when ERROR flag is NOT set:
 *    SOF, Flags, InfoFlags, UID, DSFID (opt.), AFI (opt.), Other fields (opt.), CRC16, EOF
 *
 *    InfoFlags:
 *    xxxx.3210
 *         |||\_ DSFID: 0=DSFID not supported, DSFID field NOT present; 1=DSFID supported, DSFID field present
 *         ||\__ AFI: 0=AFI not supported, AFI field not present; 1=AFI supported, AFI field present
 *         |\___ VICC memory size:
 *         |        0=Information on VICC memory size is not supported. Memory size field is present. ???
 *         |        1=Information on VICC memory size is supported. Memory size field is present.
 *         \____ IC reference:
 *                  0=Information on IC reference is not supported. IC reference field is not present.
 *                  1=Information on IC reference is supported. IC reference field is not present.
 *
 *    VICC memory size:
 *      xxxb.bbbb nnnn.nnnn
 *        bbbbb - Block size is expressed in number of bytes, on 5 bits, allowing to specify up to 32 bytes i.e. 256 bits.
 *        nnnn.nnnn - Number of blocks is on 8 bits, allowing to specify up to 256 blocks.
 *
 *    IC reference: The IC reference is on 8 bits and its meaning is defined by the IC manufacturer.
 */
PN5180Error_t pn5180::iso15693GetSystemInfo() {
  uint8_t sysInfo[10] = { 0x22, 0x2B, 1,2,3,4,5,6,7,8 };  // UID has LSB first!
  for (int i=0; i<8; i++) {
    sysInfo[2+i] = uid[i];
  }

  ESP_LOGD(TAG,"getSystemInfo: Get System Information");
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, sysInfo, sizeof(sysInfo), ESP_LOG_DEBUG);

  clearIRQStatus(0x000FFFFF); // 3. Clear all IRQ_STATUS flags
  sendData(sysInfo, 10, 0); // 4. 5. 6. Idle/StopCom Command, Transceive Command, Inventory command
  vTaskDelay(pdMS_TO_TICKS(15));

  uint32_t irqR = getIRQStatus();
  if (!(irqR & PN5180_RX_SOF_DET_IRQ_STAT)){ // 7. Check if RX data being sent
    //printIRQStatus(TAG, irqR);
    ESP_LOGD(TAG, "getSystemInfo: No data response.");
    return PN5180_ERR_NO_CARD;
  }
  uint16_t retries = 5;
  while (!(irqR & PN5180_RX_IRQ_STAT) && retries > 0) {   // 8. wait for RX end of frame (max 50ms)
    vTaskDelay(10 / portTICK_PERIOD_MS);
    irqR = getIRQStatus();
	  retries--;
  }
  ESP_LOGD(TAG, "getSystemInfo: RX_IRQ_STAT Retries - %d", retries);
  uint32_t rxStatus;
  readRegister(PN5180_RX_STATUS, &rxStatus);
  uint16_t len = (uint16_t)(rxStatus & 0x000001ff);
  if(!(irqR & PN5180_RX_IRQ_STAT)){
    ESP_LOGD(TAG, "getSystemInfo: Timeout waiting for response to end. Datalen=%d", len);
    return PN5180_ERR_TIMEOUT;
  }
  if(!len){
    ESP_LOGD(TAG, "getSystemInfo: Length is 0!!");
    return PN5180_ERR_NO_RESP;
  }
  uint8_t *resultPtr = readBuffer;
  resultPtr = readData(len);

  for (int i=0; i<8; i++) {
    uid[i] = resultPtr[2+i];
  }

  uint8_t *p = &resultPtr[10];
  uint8_t infoFlags = resultPtr[1];
  if(infoFlags & 0x01) { // DSFID flag
    dsfid = (uint8_t)(*p++);
    ESP_LOGD(TAG, "getSystemInfo: DSFID=%X", dsfid); // Data storage format identifier
  }
  else{
    dsfid = 0;
    ESP_LOGD(TAG,"getSystemInfo: No DSFID");
  }
  
  if (infoFlags & 0x02) { // AFI flag
    afi = *p++;
    afi >>= 4;
  }
  else{
    afi = 0;
    ESP_LOGD(TAG,"getSystemInfo: No AFI");
  }

  if (infoFlags & 0x04) { // VICC Memory size
    numBlocks = *p++;
    blockSize = *p++;
    blockSize &= 0x1F;
    startBlock = 0;
    endBlock = numBlocks;
    numBlocks++;
    blockSize++;
  
    ESP_LOGD(TAG, "getSystemInfo: VICC MemSize=%d BlockSize=%d NumBlocks=%d", blockSize * numBlocks, blockSize, numBlocks);
  }
  else{
    blockSize = 0;
    numBlocks = 0;
    ESP_LOGD(TAG, "getSystemInfo: No VICC memory size");
  }
   
  if (infoFlags & 0x08) { // IC reference
    ic_ref = (uint8_t)(*p++);
    ESP_LOGD(TAG, "getSystemInfo: IC Ref=%X", ic_ref);
  }
  else{
    ic_ref = 0; 
    ESP_LOGD(TAG,"getSystemInfo: No IC ref");
  }

  return PN5180_OK;
}

/*
 * Read single block, code=20
 *
 * Request format: SOF, Req.Flags, ReadSingleBlock, UID (opt.), BlockNumber, CRC16, EOF
 * Response format:
 *  when ERROR flag is set:
 *    SOF, Resp.Flags, ErrorCode, CRC16, EOF
 *
 *     Response Flags:
  *    xxxx.3xx0
  *         |||\_ Error flag: 0=no error, 1=error detected, see error field
  *         \____ Extension flag: 0=no extension, 1=protocol format is extended
  *
  *  If Error flag is set, the following error codes are defined:
  *    01 = The command is not supported, i.e. the request code is not recognized.
  *    02 = The command is not recognized, i.e. a format error occurred.
  *    03 = The option is not supported.
  *    0F = Unknown error.
  *    10 = The specific block is not available.
  *    11 = The specific block is already locked and cannot be locked again.
  *    12 = The specific block is locked and cannot be changed.
  *    13 = The specific block was not successfully programmed.
  *    14 = The specific block was not successfully locked.
  *    A0-DF = Custom command error codes
 *
 *  when ERROR flag is NOT set:
 *    SOF, Flags, BlockData (len=blockLength), CRC16, EOF
 */
ISO15693ErrorCode_t pn5180::iso15693ReadSingleBlock(uint8_t blockNo) {
  //                              flags, cmd, uid,             blockNo
  uint8_t readSingleBlockCmd[11] = { 0x22, 0x20, 1,2,3,4,5,6,7,8, blockNo }; // UID has LSB first!
  //                                |\- high data rate
  //                                \-- no options, addressed by UID
  for (int i=0; i<8; i++) {
    readSingleBlockCmd[2+i] = uid[i];
  }

  ESP_LOGD(TAG,"readSingleBlock: Read Single Block #%d, size=%d: ", blockNo, blockSize);
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, readSingleBlockCmd, sizeof(readSingleBlockCmd), ESP_LOG_DEBUG);

  ESP_LOGD(TAG, "readSingleBlock: Loading RF-Configuration for ISO15693...");
  if(loadRFConfig(0x0D, 0x8D) != ESP_OK){               // 1. Load the ISO15693 protocol into the RF registers
    ESP_LOGE(TAG, "readSingleBlock: Error loading ISO15693 RF-Configuration");
    return ISO15693_EC_UNKNOWN_ERROR;
  }

  ESP_LOGD(TAG, "readSingleBlock: Turning ON RF field for ISO15693...");
  if(setRF_on() != ESP_OK){                             // 2. Switch the RF field ON.
    ESP_LOGE(TAG, "readSingleBlock: Error turning on RF for ISO15693");
    return ISO15693_EC_UNKNOWN_ERROR;
  }

  clearIRQStatus(0x000FFFFF); // 3. Clear all IRQ_STATUS flags
  sendData(readSingleBlockCmd, 11, 0); // 4. 5. 6. Idle/StopCom Command, Transceive Command, Inventory command
  vTaskDelay(pdMS_TO_TICKS(15));

  uint32_t irqR = getIRQStatus();
  if (!(irqR & PN5180_RX_SOF_DET_IRQ_STAT)){ // 7. Check if RX data being sent
    printIRQStatus(TAG, irqR);
    ESP_LOGE(TAG, "readSingleBlock: No data response.");
    setRF_off();
    return EC_NO_CARD;
  }
  uint16_t retries = 5;
  while (!(irqR & PN5180_RX_IRQ_STAT) && retries > 0) {   // 8. wait for RX end of frame (max 50ms)
    vTaskDelay(10 / portTICK_PERIOD_MS);
    irqR = getIRQStatus();
	  retries--;
  }
  uint32_t rxStatus;
  readRegister(PN5180_RX_STATUS, &rxStatus);
  uint16_t len = (uint16_t)(rxStatus & 0x000001ff);
  if(!(irqR & PN5180_RX_IRQ_STAT)){
    ESP_LOGE(TAG, "readSingleBlock: Timeout waiting for response to end. Datalen=%d", len);
    setRF_off();
    reset();
    return EC_NO_CARD;
  }
  if(!len){
    ESP_LOGE(TAG, "readSingleBlock: Length is 0!!");
    setRF_off();
    return EC_NO_CARD;
  }

  uint8_t *resultPtr = readBuffer;
  resultPtr = readData(len);

  uint8_t startAddr = blockNo * blockSize;
  for (int i=0; i<blockSize; i++) {
    blockData[startAddr + i] = resultPtr[1+i];
  }

  ESP_LOGD(TAG,"readSingleBlock: Value=");
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, blockData, blockSize, ESP_LOG_DEBUG);

  setRF_off();
  return ISO15693_EC_OK;
}

/*
 * 
 */
PN5180Error_t pn5180::iso14443poll(uint8_t atqaCmd){
	if (writeRegisterAndMask(PN5180_SYSTEM_CONFIG, 0xFFFFFFBF) != ESP_OK) {     // 3. Switch the CRC extension off in Tx direction
		ESP_LOGE(TAG, "iso14443Poll: Failed to disable crypto");
		return PN5180_ERR_REGISTER;
	}
  if (writeRegisterAndMask(PN5180_CRC_RX_CONFIG, 0xFFFFFFFE) != ESP_OK) {     // 3. Switch the CRC extension off in Tx direction
		ESP_LOGE(TAG, "iso14443Poll: Failed to disable CRC TX");
		return PN5180_ERR_REGISTER;
	}
	if (writeRegisterAndMask(PN5180_CRC_TX_CONFIG, 0xFFFFFFFE) != ESP_OK) {     // 4. Switch the CRC extension off in Rx direction
		ESP_LOGE(TAG, "iso14443Poll: Failed to disable CRC RX");
		return PN5180_ERR_REGISTER;
	}
  clearIRQStatus(0x000FFFFF);                                                 // 5. Clear the interrupt register IRQ_STATUS

  uint8_t cmd[7] = {atqaCmd,0,0,0,0,0,0};
  sendData(cmd, 1, 7);                                                        // 6. 7. 8. Idle/StopCom Command, Transceive Command, REQA command
  
  vTaskDelay(pdMS_TO_TICKS(15));
  uint32_t irqStatus = getIRQStatus();
  if(!(irqStatus & PN5180_RX_IRQ_STAT)){                                      // 9. Wait until a Card has responded via checking the IRQ_STATUS register
    ESP_LOGD(TAG, "iso14443Poll: No card detected. State=%ld", irqStatus);
    //printIRQStatus(TAG, irqStatus);
    return PN5180_ERR_NO_CARD;
  }
  uint32_t rxStatus;
  readRegister(PN5180_RX_STATUS, &rxStatus);
  uint16_t len = (uint16_t)(rxStatus & 0x000001FF);
  
  uint8_t* resultPtr = readBuffer;
  resultPtr = readData(2);                                                    // 10. Read the reception buffer (ATQA)

  /* Table specifies the coding of 2 byte ATQA, All RFU bits shall be set to (0)b
   * | 15 14 13 12 | 11 10 09 08 |  07 06  |  05 |     04 03 02 01 00      |
   * |     RFU     |   Private   | UID len | RFU | Bit frame anticollision |
   * 
   * UID len = 0(single UID), 1(double UID), 2(triple UID), 3(RFU)
   * 
   * However, the recommendation for MIFARE is to ignore ATQA, and only use this as
   * a means to recognize the presence of a card.
   */ 
  ESP_LOGD(TAG, "iso14443Poll: ATQA: %d:%d, len: %d", resultPtr[0], resultPtr[1], len);

  /* ANTICOLLISION CASCADE LEVEL 1 */
  uint8_t casc1[2] = {0x93, 0x20};
  memcpy(cmd, casc1, 2);
  clearIRQStatus(0xFFFFFFFF);
	sendData(cmd, 2, 0x00); // Cascade level 1 (0x93), NVB (0x20)
  vTaskDelay(pdMS_TO_TICKS(15));

  /* Ensure we detect the expected length of the response (5 bytes) */
  readRegister(PN5180_RX_STATUS, &rxStatus);
  len = (uint16_t)(rxStatus & 0x000001FF);
  if (len != 5 && len != 0) {
		ESP_LOGD(TAG, "iso14443Poll: Card present but collision detected, try again...");
    return PN5180_ERR_COLLISION;
	}
  if (!len){
    ESP_LOGD(TAG, "iso14443Poll: No length detected! Did the card move?");
    return PN5180_ERR_NO_RESP;
  }

  /* Obtain and process SAK response */
  uint8_t* sakBuf = sak;
	sakBuf = readData(len); // Get SAK
  ESP_LOGD(TAG, "len=%d, SAK=%x:%x:%x:%x:%x", len, sakBuf[0], sakBuf[1], sakBuf[2], sakBuf[3], sakBuf[4]);


  if (writeRegisterOrMask(PN5180_CRC_RX_CONFIG, 0x01) != ESP_OK) { // 3. Switch the CRC extension on in Tx direction
		ESP_LOGE(TAG, "iso14443Poll: Failed to enable CRC TX");
    return PN5180_ERR_REGISTER;
	}
	if (writeRegisterOrMask(PN5180_CRC_TX_CONFIG, 0x01) != ESP_OK) { // 4. Switch the CRC extension on in Rx direction
		ESP_LOGE(TAG, "iso14443Poll: Failed to enable CRC RX");
    return PN5180_ERR_REGISTER;
	}

  cmd[1] = 0x70;
  memcpy(cmd+2, sakBuf, 5);
  clearIRQStatus(0x000FFFFF);

  ESP_LOGD(TAG, "sendCmd: %x:%x:%x:%x:%x:%x:%x", cmd[0],cmd[1],cmd[2],cmd[3],cmd[4], cmd[5], cmd[6]);
	sendData(cmd, 7, 0x00); // Cascade level (cascLevel), NVB (0x70)
  vTaskDelay(pdMS_TO_TICKS(15));
  
  readRegister(PN5180_RX_STATUS, &rxStatus);
  len = (uint16_t)(rxStatus & 0x000001ff);

  uint8_t* uidBuf = uid;
  if(!len){
    ESP_LOGI(TAG, "iso14443Poll: No length detected. We are done.");
    uidLength = 4;
    type = ISO14443;
    memcpy(uidBuf, cmd+2, 4);
    return PN5180_OK;
  }

  uint8_t* sak2Buf;
  sak2Buf = readData(len); // Read SAK (should always be len 1)
  ESP_LOGD(TAG, "readBuffer: %x:%x:%x:%x:%x", readBuffer[0],readBuffer[1],readBuffer[2],readBuffer[3],readBuffer[4]);
  ESP_LOGD(TAG, "sendCmd: %x:%x:%x:%x:%x:%x:%x", cmd[0],cmd[1],cmd[2],cmd[3],cmd[4], cmd[5], cmd[6]);

/////////
  ESP_LOGD(TAG, "ATS: %d", sak2Buf[0]);
  if (!(sak2Buf[0] & 0x04)) { // Check bit 3 of 2nd SAK. If 0, the uidLength = 4
    ESP_LOGD(TAG, "iso14443Poll: sak_sel2 confirmed. We should be activated.");
		uidLength = 4;
    type = ISO14443;
    iso14443GetSystemInfo(sak2Buf[0]);
    memcpy(uidBuf, cmd+2, 4);
    return PN5180_OK;
	}
  /*ESP_LOGI(TAG, "UID ongoing: %d:%d:%d", uid[0],uid[1],uid[2]);
  // Cascade level 2
  p = iso14443Collision(0x95);
  if(p[0] == 255){
    return ESP_ERR_INVALID_STATE;
  }
  else if (!(sak2 & 0x04)) { // Check bit 3 of 2nd SAK. If 0, the uidLength = 7
		for(int i=0; i<4; i++){
      uid[3+i] = sak[i];
    }
    ESP_LOGI(TAG, "UID len7: %d:%d:%d:%d:%d:%d:%d", uid[0],uid[1],uid[2],uid[3], uid[4],uid[5],uid[6]);
    uidLength = 7;
    setRF_off();
    printUID();
    return ESP_OK;
	}
  for(int i=0; i<3; i++){
    uid[3+i] = sak[i+1];
  }
  ESP_LOGI(TAG, "UID ongoing: %d:%d:%d:%d:%d:%d", uid[0],uid[1],uid[2],uid[3], uid[4],uid[5]);
  // Cascade level 3
  p = iso14443Collision(0x97);
  if(p[0] == 255){
    return ESP_ERR_INVALID_STATE;
  }
  else if (!(sak2 & 0x04)) { // Check bit 3 of 2nd SAK. If 0, the uidLength = 10
		for(int i=0; i<4; i++){
      uid[6+i] = sak[i];
    }
    uidLength = 10;
    setRF_off();
    printUID();
    return ESP_OK;
	}*/
  return PN5180_ERR_UNKNOWN; // If we got this far something went wrong.
}

void pn5180::iso14443GetSystemInfo(uint8_t ats){
  ESP_LOGD(TAG, "Looking for mifate tag type 0x%x", ats);
  const MifareType_t mType = mifare.find(ats)->second;
  numBlocks = mType.numBlocks;
  blockSize = mType.blockSize;
  startBlock = mType.startBlock;
  endBlock = mType.endBlock;
}

bool pn5180::mifareAuthenticate(uint8_t blockNo){
  uint8_t auth[13] = {0x0C, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x60, blockNo, uid[0], uid[1], uid[2], uid[3]};
  clearIRQStatus(0x000FFFFF);

  uint8_t *resultPtr = readBuffer;
  transceiveCommand(auth, 13, resultPtr, 1);

  return (resultPtr[0]) ? false : true;
}

bool pn5180::mifareBlockRead(uint8_t blockNo) {
  uint8_t readB[2] = {0x30, 4};
  printUID();

  clearIRQStatus(0x000FFFFF);
  sendData(readB, 2, 0);
  vTaskDelay(pdMS_TO_TICKS(15));
  uint32_t rxStatus;
  readRegister(PN5180_RX_STATUS, &rxStatus);
  uint16_t len = (uint16_t)(rxStatus & 0x000001FF);
  
  ESP_LOGD(TAG, "mifareRead: len=%d", len);
  if(!len){
    ESP_LOGD(TAG, "Unable to read Mifare block");
    return false;
  }

  uint8_t *resultPtr = readBuffer;
  resultPtr = readData(len);
  for(int i=0; i<len; i++){
    printf("%x : ", resultPtr[i]);
  }
  printf("\n");
  
	return true;
}

bool pn5180::mifareBlockWrite(uint8_t *data, uint8_t blockNo) {
  uint8_t writeCmd[2] = {0xA0, blockNo};
  writeRegisterAndMask(PN5180_CRC_RX_CONFIG, 0xFFFFFFFE);
  
  clearIRQStatus(0x000FFFFF);
  sendData(writeCmd, 2, 0);
  vTaskDelay(pdMS_TO_TICKS(15));
  uint32_t rxStatus;
  readRegister(PN5180_RX_STATUS, &rxStatus);
  uint16_t len = (uint16_t)(rxStatus & 0x000001ff);

  ESP_LOGI(TAG, "mifareWrite: len=%d", len);
  printIRQStatus(TAG, getIRQStatus());
  uint8_t *resultPtr = readBuffer;
  resultPtr = readData(len);
  for(int i=0; i<len; i++){
    printf("%x : ", resultPtr[i]);
  }
  printf("\n");
  printIRQStatus(TAG, getIRQStatus());

  clearIRQStatus(0x000FFFFF);
  sendData(data, 16, 0);
  vTaskDelay(pdMS_TO_TICKS(15));
  readRegister(PN5180_RX_STATUS, &rxStatus);
  len = (uint16_t)(rxStatus & 0x000001ff);
  if(!len) {
    ESP_LOGD(TAG, "Length 0, expected Length 16");
    return false;
  }

  return true;
}

void pn5180::mifareHalt(void) {
  uint8_t halt[2] = {0x50, 0x00};

  writeRegisterOrMask(PN5180_CRC_RX_CONFIG, 0x1);
  clearIRQStatus(0x000FFFFF);
	sendData(halt, 2, 0x00);
  vTaskDelay(pdMS_TO_TICKS(15));
}

/*
 * Write single block, code=21
 *
 * Request format: SOF, Requ.Flags, WriteSingleBlock, UID (opt.), BlockNumber, BlockData (len=blcokLength), CRC16, EOF
 * Response format:
 *  when ERROR flag is set:
 *    SOF, Resp.Flags, ErrorCode, CRC16, EOF
 *
 *     Response Flags:
  *    xxxx.3xx0
  *         |||\_ Error flag: 0=no error, 1=error detected, see error field
  *         \____ Extension flag: 0=no extension, 1=protocol format is extended
  *
  *  If Error flag is set, the following error codes are defined:
  *    01 = The command is not supported, i.e. the request code is not recognized.
  *    02 = The command is not recognized, i.e. a format error occurred.
  *    03 = The option is not supported.
  *    0F = Unknown error.
  *    10 = The specific block is not available.
  *    11 = The specific block is already locked and cannot be locked again.
  *    12 = The specific block is locked and cannot be changed.
  *    13 = The specific block was not successfully programmed.
  *    14 = The specific block was not successfully locked.
  *    A0-DF = Custom command error codes
 *
 *  when ERROR flag is NOT set:
 *    SOF, Resp.Flags, CRC16, EOF
 */
ISO15693ErrorCode_t pn5180::writeSingleBlock(uint8_t blockNo, uint8_t* blockData, uint8_t len) {
  if(len > blockSize){ // Data to write is too large
    return ISO15693_EC_BLOCK_NOT_AVAILABLE;
  }
  
  //                            flags, cmd, uid,             blockNo
  uint8_t writeSingleBlockCmd[] = { 0x22, 0x21, 1,2,3,4,5,6,7,8, blockNo }; // UID has LSB first!
  //                               |\- high data rate
  //                               \-- no options, addressed by UID

  uint8_t writeCmdSize = sizeof(writeSingleBlockCmd) + blockSize;
  uint8_t* writeCmd = (uint8_t*)heap_caps_malloc(writeCmdSize, MALLOC_CAP_8BIT);
  ESP_LOGD(TAG,"Free malloc after: %d", heap_caps_get_free_size(MALLOC_CAP_8BIT));
  uint8_t pos = 0;
  writeCmd[pos++] = writeSingleBlockCmd[0];
  writeCmd[pos++] = writeSingleBlockCmd[1];
  for (int i=0; i<8; i++) {
    writeCmd[pos++] = uid[i];
  }
  writeCmd[pos++] = blockNo;
  uint8_t startAddr = blockNo * blockSize;
  // Start of actual data creation loop
  for (int i=0; i<len; i++) {
    writeCmd[pos++] = blockData[startAddr + i];
  }
  // End of data loop

  ESP_LOGI(TAG,"writeSingleBlock: Write Single Block #%d, size=%d: ", blockNo, blockSize);
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, writeCmd, writeCmdSize, ESP_LOG_INFO);

  uint8_t *resultPtr;
  ISO15693ErrorCode_t rc = ISO15693Command(writeCmd, writeCmdSize, &resultPtr);
  heap_caps_free(writeCmd);
  return rc;
}

/*
 * Read multiple block, code=23
 *
 * Request format: SOF, Req.Flags, ReadMultipleBlock, UID (opt.), FirstBlockNumber, numBlocks, CRC16, EOF
 * Response format:
 *  when ERROR flag is set:
 *    SOF, Resp.Flags, ErrorCode, CRC16, EOF
 *
 *     Response Flags:
  *    xxxx.3xx0
  *         |||\_ Error flag: 0=no error, 1=error detected, see error field
  *         \____ Extension flag: 0=no extension, 1=protocol format is extended
  *
  *  If Error flag is set, the following error codes are defined:
  *    01 = The command is not supported, i.e. the request code is not recognized.
  *    02 = The command is not recognized, i.e. a format error occurred.
  *    03 = The option is not supported.
  *    0F = Unknown error.
  *    10 = The specific block is not available.
  *    11 = The specific block is already locked and cannot be locked again.
  *    12 = The specific block is locked and cannot be changed.
  *    13 = The specific block was not successfully programmed.
  *    14 = The specific block was not successfully locked.
  *    A0-DF = Custom command error codes
 *
 *  when ERROR flag is NOT set:
 *    SOF, Flags, BlockData (len=nfc->blockSize * numBlock), CRC16, EOF
 */
ISO15693ErrorCode_t pn5180::readMultipleBlock(uint8_t blockNo, uint8_t numBlock) {
  if(blockNo > numBlocks-1){
    ESP_LOGE(TAG, "Starting block exceeds length of data");
    return ISO15693_EC_BLOCK_NOT_AVAILABLE;
  }
  if( (blockNo + numBlock) > numBlocks ){
    ESP_LOGE(TAG, "End of block exceeds length of data");
    return ISO15693_EC_BLOCK_NOT_AVAILABLE;
  }
  //if(len < (numBlock * blockSize)){ // Data to read is larger than provided var
  //  return ISO15693_EC_BLOCK_NOT_AVAILABLE;
  //}

  //uint8_t readMultipleCmd[4] = {0x02, 0x23, blockNo, numBlock-1};
  //                              flags, cmd, uid,             blockNo
  uint8_t readMultipleCmd[12] = { 0x22, 0x23, 1,2,3,4,5,6,7,8, blockNo, (uint8_t)(numBlock-1) }; // UID has LSB first!
  //                                |\- high data rate
  //                                \-- no options, addressed by UID
  for (int i=0; i<8; i++) {
    readMultipleCmd[2+i] = uid[i];
  }

  ESP_LOGD(TAG,"readMultipleBlock: Read Block #%d-%d, size=%d: ", blockNo, blockNo+numBlock-1, blockSize);
  ESP_LOGD(TAG, "readMultipleBlock: Loading RF-Configuration for ISO15693...");
  if(loadRFConfig(0x0D, 0x8D) != ESP_OK){               // 1. Load the ISO15693 protocol into the RF registers
    ESP_LOGE(TAG, "readMultipleBlock: Error loading ISO15693 RF-Configuration");
    reset();
    return ISO15693_EC_UNKNOWN_ERROR;
  }

  ESP_LOGD(TAG, "readMultipleBlock: Turning ON RF field for ISO15693...");
  if(setRF_on() != ESP_OK){                             // 2. Switch the RF field ON.
    ESP_LOGE(TAG, "readMultipleBlock: Error turning on RF for ISO15693");
    reset();
    return ISO15693_EC_UNKNOWN_ERROR;
  }

  clearIRQStatus(0x000FFFFF); // 3. Clear all IRQ_STATUS flags
  sendData(readMultipleCmd, 12, 0); // 4. 5. 6. Idle/StopCom Command, Transceive Command, Inventory command
  vTaskDelay(pdMS_TO_TICKS(15));

  uint32_t irqR = getIRQStatus();
  uint16_t retries = 5;
  while (!(irqR & PN5180_RX_SOF_DET_IRQ_STAT) && retries > 0) {   // 8. wait for RX start of frame (max 50ms)
    vTaskDelay(10 / portTICK_PERIOD_MS);
    irqR = getIRQStatus();
	  retries--;
  }
  if (!(irqR & PN5180_RX_SOF_DET_IRQ_STAT)){ // 7. Check if RX data being sent
    printIRQStatus(TAG, irqR);
    ESP_LOGE(TAG, "readMultipleBlock: No data response.");
    setRF_off();
    return EC_NO_CARD;
  }
  retries = 5;
  while (!(irqR & PN5180_RX_IRQ_STAT) && retries > 0) {   // 8. wait for RX end of frame (max 50ms)
    vTaskDelay(10 / portTICK_PERIOD_MS);
    irqR = getIRQStatus();
	  retries--;
  }
  uint32_t rxStatus;
  readRegister(PN5180_RX_STATUS, &rxStatus);
  uint16_t len = (uint16_t)(rxStatus & 0x000001ff);
  if(!(irqR & PN5180_RX_IRQ_STAT)){
    ESP_LOGE(TAG, "readMultipleBlock: Timeout waiting for response to end. Datalen=%d", len);
    setRF_off();
    reset();
    return EC_NO_CARD;
  }
  if(!len){
    ESP_LOGE(TAG, "readMultipleBlock: Length is 0!!");
    setRF_off();
    return EC_NO_CARD;
  }

  uint8_t *resultPtr = readBuffer;
  uint8_t startAddr = blockNo * blockSize;
  resultPtr = readData(len);
  for (int i=0; i<numBlock * blockSize; i++) {
    ESP_LOGD(TAG,"readMultipleBlock: resultPtr=%d", resultPtr[1+i]);
    blockData[startAddr + i] = resultPtr[1+i];
  }

  ESP_LOGD(TAG,"readMultipleBlock: Value=");
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, blockData, numBlock * blockSize, ESP_LOG_DEBUG);

  setRF_off();
  return ISO15693_EC_OK;
}

/*
 * ISO 15693 - Protocol
 *
 * General Request Format:
 *  SOF, Req.Flags, Command code, Parameters, Data, CRC16, EOF
 *
 *  Request Flags:
 *    xxxx.3210
 *         |||\_ Subcarrier flag: 0=single sub-carrier, 1=two sub-carrier
 *         ||\__ Datarate flag: 0=low data rate, 1=high data rate
 *         |\___ Inventory flag: 0=no inventory, 1=inventory
 *         \____ Protocol extension flag: 0=no extension, 1=protocol format is extended
 *
 *  If Inventory flag is set:
 *    7654.xxxx
 *     ||\_ AFI flag: 0=no AFI field present, 1=AFI field is present
 *     |\__ Number of slots flag: 0=16 slots, 1=1 slot
 *     \___ Option flag: 0=default, 1=meaning is defined by command description
 *
 *  If Inventory flag is NOT set:
 *    7654.xxxx
 *     ||\_ Select flag: 0=request shall be executed by any VICC according to Address_flag
 *     ||                1=request shall be executed only by VICC in selected state
 *     |\__ Address flag: 0=request is not addressed. UID field is not present.
 *     |                  1=request is addressed. UID field is present. Only VICC with UID shall answer
 *     \___ Option flag: 0=default, 1=meaning is defined by command description
 *
 * General Response Format:
 *  SOF, Resp.Flags, Parameters, Data, CRC16, EOF
 *
 *  Response Flags:
 *    xxxx.3210
 *         |||\_ Error flag: 0=no error, 1=error detected, see error field
 *         ||\__ RFU: 0
 *         |\___ RFU: 0
 *         \____ Extension flag: 0=no extension, 1=protocol format is extended
 *
 *  If Error flag is set, the following error codes are defined:
 *    01 = The command is not supported, i.e. the request code is not recognized.
 *    02 = The command is not recognized, i.e. a format error occurred.
 *    03 = The option is not supported.
 *    0F = Unknown error.
 *    10 = The specific block is not available.
 *    11 = The specific block is already locked and cannot be locked again.
 *    12 = The specific block is locked and cannot be changed.
 *    13 = The specific block was not successfully programmed.
 *    14 = The specific block was not successfully locked.
 *    A0-DF = Custom command error codes
 *
 *  Function return values:
 *    0 = OK
 *   -1 = No card detected
 *   >0 = Error code
 */
ISO15693ErrorCode_t pn5180::ISO15693Command(uint8_t *cmd, uint16_t cmdLen, uint8_t **resultPtr) {
  ESP_LOGD(TAG,"ISO5693Command: Issue Command 0x%X...", cmd[1]);
  sendData(cmd, cmdLen, 0);
  vTaskDelay(10 / portTICK_PERIOD_MS);

  uint16_t retries = 50;
  uint32_t irqR = getIRQStatus();
  while (!(irqR & PN5180_RX_SOF_DET_IRQ_STAT) && retries > 0) {   // wait for RF field to set up (max 500ms)
    vTaskDelay(10 / portTICK_PERIOD_MS);
	  irqR = getIRQStatus();
    retries--;
  }
  if (0 == (irqR & PN5180_RX_SOF_DET_IRQ_STAT)){
    printIRQStatus(TAG, irqR);
    ESP_LOGE(TAG, "ISO15693Command: No RX_SOF_DET IRQ. State=%ld", irqR);
    //return EC_NO_CARD;
  }
  retries = 50;
  while (!(irqR & PN5180_RX_IRQ_STAT) && retries > 0) {   // wait for RX end of frame (max 500ms)
    vTaskDelay(10 / portTICK_PERIOD_MS);
    irqR = getIRQStatus();
	  retries--;
  }
  uint32_t rxStatus;
  readRegister(PN5180_RX_STATUS, &rxStatus);
  uint16_t len = (uint16_t)(rxStatus & 0x000001ff);
  if(!(irqR & PN5180_RX_IRQ_STAT) && !len){
    printIRQStatus(TAG, irqR);
    ESP_LOGE(TAG, "ISO15693Command: No EOF_RX IRQ and RX_STATUS: length = 0. State=%ld", irqR);
    clearIRQStatus(PN5180_TX_IRQ_STAT | PN5180_IDLE_IRQ_STAT);
    return EC_NO_CARD;
  }
  
  ESP_LOGD(TAG,"ISO5693Command: RX-Status=0x%lX, len=%d", rxStatus, len);

  *resultPtr = readData(len);
  if (0L == *resultPtr) {
    ESP_LOGE(TAG,"ISO5693Command: ERROR in readData!");
    return ISO15693_EC_UNKNOWN_ERROR;
  }

  uint32_t irqStatus = getIRQStatus();
  if (0 == (PN5180_RX_SOF_DET_IRQ_STAT & irqStatus)) { // no card detected
    printIRQStatus(TAG, irqR);
    clearIRQStatus(PN5180_TX_IRQ_STAT | PN5180_IDLE_IRQ_STAT);
    return EC_NO_CARD;
  }

  uint8_t responseFlags = (*resultPtr)[0];
  if (responseFlags & (1<<0)) { // error flag
    uint8_t errorCode = (*resultPtr)[1];
    ESP_LOGE(TAG,"ISO5693Command: ERROR code=%X",errorCode);
    //printError(errorCode);
    if (errorCode >= 0xA0) { // custom command error codes
      return ISO15693_EC_CUSTOM_CMD_ERROR;
    }
    else return (ISO15693ErrorCode_t)errorCode;
  }

  ESP_LOGD(TAG,"ISO5693Command: Extension flag: %d", (responseFlags & (1<<3)));

  clearIRQStatus(PN5180_RX_SOF_DET_IRQ_STAT | PN5180_IDLE_IRQ_STAT | PN5180_TX_IRQ_STAT | PN5180_RX_IRQ_STAT);
  return ISO15693_EC_OK;
}

void pn5180::printError(uint8_t err) {
  char strErr[50] = "";
  switch (err) {
    case EC_NO_CARD: strcat(strErr,"No card detected!"); break;
    case ISO15693_EC_OK: strcat(strErr,"OK!"); break;
    case ISO15693_EC_NOT_SUPPORTED: strcat(strErr,"Command is not supported!"); break;
    case ISO15693_EC_NOT_RECOGNIZED: strcat(strErr,"Command is not recognized!"); break;
    case ISO15693_EC_OPTION_NOT_SUPPORTED: strcat(strErr,"Option is not supported!"); break;
    case ISO15693_EC_UNKNOWN_ERROR: strcat(strErr,"Unknown error!"); break;
    case ISO15693_EC_BLOCK_NOT_AVAILABLE: strcat(strErr,"Specified block is not available!"); break;
    case ISO15693_EC_BLOCK_ALREADY_LOCKED: strcat(strErr,"Specified block is already locked!"); break;
    case ISO15693_EC_BLOCK_IS_LOCKED: strcat(strErr,"Specified block is locked and cannot be changed!"); break;
    case ISO15693_EC_BLOCK_NOT_PROGRAMMED: strcat(strErr,"Specified block was not successfully programmed!"); break;
    case ISO15693_EC_BLOCK_NOT_LOCKED: strcat(strErr,"Specified block was not successfully locked!"); break;
    default:
      if ((err >= 0xA0) && (err <= 0xDF)) {
        strcat(strErr,"Custom command error code!");
      }
      else strcat(strErr,"Undefined error code in ISO15693!");
  }
  ESP_LOGE(TAG, "ISO15693 Error: %s", strErr);
}

void pn5180::printUID(){
  printf("\033[32mI (%ld) %s: UID=", esp_log_timestamp(), TAG);
  for(int i=uidLength-1; i>=0; i--){
    if(uid[i] < 16) printf("0");
    printf("%X", uid[i]);
    if(i > 0) printf(":");
  }
  printf("\n\033[0m");
}

void pn5180::printSingleBlock(uint8_t blockNum){
  if(blockNum + startBlock > endBlock){
    ESP_LOGE(TAG, "Chosen block is not a valid block");
  }
  else{
    if(ESP_LOG_INFO <= esp_log_level_get(TAG)){
      uint16_t startAddr = blockSize * blockNum;
      // Hex print
      ESP_LOGD(TAG, "startAddr=%d", startAddr);
      printf("\033[32mI (%ld) %s: ", esp_log_timestamp(), TAG);
      for (int i=0; i<blockSize; i++) {
        if(blockData[startAddr + i] < 16) printf("0");
        printf("%X", blockData[startAddr + i]);
        if(i < blockSize - 1) printf(":");
      }
      
      printf(" ");
      
      // String print
      for (int i=0; i<blockSize; i++) {
        char c = blockData[startAddr + i];
        if (isprint(c)) {
          printf("%c",c);
        }
        else printf(".");
      }
      printf("\n\033[0m");
    }
  }
}

void pn5180::printAllBlockData(){
  if(ESP_LOG_INFO <= esp_log_level_get(TAG)){
    // Hex print
    for(int block=0; block<numBlocks; block++){
      uint16_t startAddr = (block * blockSize);
      printf("\033[32mI (%ld) %s: ", esp_log_timestamp(), TAG);
      for (int i=0; i<blockSize; i++) {
        if(blockData[startAddr + i] < 16) printf("0");
        printf("%X", blockData[startAddr + i]);
        if(i < blockSize - 1) printf(":");
      }
      
      printf(" ");
      
      // String print
      for (int i=0; i<blockSize; i++) {
        char c = blockData[startAddr + i];
        if (isprint(c)) {
          printf("%c",c);
        }
        else printf(".");
      }
      printf("\n\033[0m");
    }
  }
}

esp_err_t pn5180::transceiveCommand(uint8_t *sendBuffer, size_t sendBufferLen, uint8_t *recvBuffer, size_t recvBufferLen) {
  esp_err_t ret;
  ////////////////////
  // Initialization //
  ////////////////////
  ESP_LOGD(TAG, "transceiveCommand: Write, wait for busy...");
  ret = busyWait(1000); // 1.
  if(ret == ESP_ERR_TIMEOUT){
    ESP_LOGE(TAG, "transceiveCommand: BUSY signal line timeout");
    return ret;
  }
  ESP_LOGD(TAG, "transceiveCommand: SPI transaction: write %d read %d", sendBufferLen, recvBufferLen);
  
  //////////////////
  // Send command //
  //////////////////
  ESP_LOGD(TAG, "transceiveCommand: Write data:");
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, sendBuffer, sendBufferLen, ESP_LOG_DEBUG);
  ret = spi_txn(dev, sendBuffer, sendBufferLen, NULL, 0); // 2. 3. 4.
  if(ret != ESP_OK){
    ESP_LOGE(TAG, "pn5180_command: SPI transaction write failed");
    return ret;
  }

  // Finish if write-only command
  if ((0 == recvBuffer) || (0 == recvBufferLen)) return ret;

  ESP_LOGD(TAG, "command: Read, wait for busy...");
  ret = busyWait(1000); // 5.
  if(ret == ESP_ERR_TIMEOUT){
    ESP_LOGE(TAG, "command: BUSY signal line timeout");
    return ret;
  }
  memset(recvBuffer, 0xFF, recvBufferLen);

  //////////////////////
  // Receive Response //
  //////////////////////
  ret = spi_txn(dev, recvBuffer, recvBufferLen, recvBuffer, recvBufferLen); // 6. 7. 8.
  if(ret != ESP_OK){
    ESP_LOGE(TAG, "pn5180_command: SPI transaction read failed");
    reset();
    return ret;
  }

  ESP_LOGD(TAG, "command: Read data:");
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, recvBuffer, recvBufferLen, ESP_LOG_DEBUG);
  
  return ret;
}

esp_err_t pn5180::spi_txn(spi_device_handle_t dev, const void *tx, int txLen, void *rx, int rxLen) {
    spi_transaction_t txn = {
        .length = (size_t)(txLen * 8),
        .rxlength = (size_t)(rxLen * 8),
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(dev, &txn);
}

/*
 * The BUSY signal is used to indicate that the PN5180 is not able to send or receive data
 * over the SPI interface
 */
esp_err_t pn5180::busyWait(uint32_t timeout){
  while (gpio_get_level(PN5180_BUSY) && timeout > 0){
    vTaskDelay(pdMS_TO_TICKS(1));
    timeout--;
  }
  vTaskDelay(pdMS_TO_TICKS(1));
  if(gpio_get_level(PN5180_BUSY)){
    ESP_LOGE(TAG, "busy_wait: Timeout waiting for BUSY pin LOW");
    reset();
    return ESP_ERR_TIMEOUT;
  }
  return ESP_OK;
}