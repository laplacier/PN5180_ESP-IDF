// NAME: PN5180.h
//
// DESC: NFC Communication with NXP Semiconductors PN5180 module for Arduino.
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
#ifndef pn5180_h
#define pn5180_h

#define GENERIC_TASK_PRIO 1

// PN5180 Commands
#define PN5180_WRITE_REGISTER                     (0x00)
#define PN5180_WRITE_REGISTER_OR_MASK             (0x01)
#define PN5180_WRITE_REGISTER_AND_MASK            (0x02)
#define PN5180_WRITE_REGISTER_MULTIPLE            (0x03)
#define PN5180_READ_REGISTER                      (0x04)
#define PN5180_READ_REGISTER_MULTIPLE             (0x05)
#define PN5180_WRITE_EEPROM                       (0x06)
#define PN5180_READ_EEPROM                        (0x07)
#define PN5180_WRITE_TX_DATA                      (0x08)
#define PN5180_SEND_DATA                          (0x09)
#define PN5180_READ_DATA                          (0x0A)
#define PN5180_SWITCH_MODE                        (0x0B)
#define PN5180_MIFARE_AUTHENTICATE                (0x0C)
#define PN5180_EPC_INVENTORY                      (0x0D)
#define PN5180_EPC_RESUME_INVENTORY               (0x0E)
#define PN5180_EPC_RETRIEVE_INVENTORY_RESULT_SIZE (0x0F)
#define PN5180_EPC_RETRIEVE_INVENTORY_RESULT      (0x10)
#define PN5180_LOAD_RF_CONFIG                     (0x11)
#define PN5180_UPDATE_RF_CONFIG                   (0x12)
#define PN5180_RETRIEVE_RF_CONFIG_SIZE            (0x13)
#define PN5180_RETRIEVE_RF_CONFIG                 (0x14)
#define PN5180_RF_ON                              (0x16)
#define PN5180_RF_OFF                             (0x17)
#define PN5180_CONFIGURE_TESTBUS_DIGITAL          (0x18)
#define PN5180_CONFIGURE_TESTBUS_ANALOG           (0x19)

// PN5180 Registers
#define PN5180_SYSTEM_CONFIG       (0x00)
#define PN5180_IRQ_ENABLE          (0x01)
#define PN5180_IRQ_STATUS          (0x02)
#define PN5180_IRQ_CLEAR           (0x03)
#define PN5180_TRANSCEIVE_CONTROL  (0x04)
#define PN5180_TIMER1_RELOAD       (0x0C)
#define PN5180_TIMER1_CONFIG       (0x0F)
#define PN5180_RX_WAIT_CONFIG      (0x11)
#define PN5180_CRC_RX_CONFIG       (0x12)
#define PN5180_RX_STATUS           (0x13)
#define PN5180_TX_WAIT_CONFIG	     (0x17)
#define PN5180_TX_CONFIG			     (0x18)
#define PN5180_CRC_TX_CONFIG       (0x19)
#define PN5180_RF_STATUS           (0x1D)
#define PN5180_SYSTEM_STATUS       (0x24)
#define PN5180_TEMP_CONTROL        (0x25)
#define PN5180_AGC_REF_CONFIG	     (0x26)


// PN5180 EEPROM Addresses
#define PN5180_DIE_IDENTIFIER      (0x00)
#define PN5180_PRODUCT_VERSION     (0x10)
#define PN5180_FIRMWARE_VERSION    (0x12)
#define PN5180_EEPROM_VERSION      (0x14)
#define PN5180_IRQ_PIN_CONFIG      (0x1A)

// PN5180 IRQ_STATUS
#define PN5180_RX_IRQ_STAT         	  (1<<0)  // End of RF receiption IRQ
#define PN5180_TX_IRQ_STAT         	  (1<<1)  // End of RF transmission IRQ
#define PN5180_IDLE_IRQ_STAT       	  (1<<2)  // IDLE IRQ
#define PN5180_RFOFF_DET_IRQ_STAT  	  (1<<6)  // RF Field OFF detection IRQ
#define PN5180_RFON_DET_IRQ_STAT   	  (1<<7)  // RF Field ON detection IRQ
#define PN5180_TX_RFOFF_IRQ_STAT   	  (1<<8)  // RF Field OFF in PCD IRQ
#define PN5180_TX_RFON_IRQ_STAT    	  (1<<9)  // RF Field ON in PCD IRQ
#define PN5180_RX_SOF_DET_IRQ_STAT 	  (1<<14) // RF SOF Detection IRQ
#define PN5180_GENERAL_ERROR_IRQ_STAT (1<<17) // General error IRQ
#define PN5180_LPCD_IRQ_STAT 			    (1<<19) // LPCD Detection IRQ

typedef enum {
  PN5180_TS_Idle = 0,
  PN5180_TS_WaitTransmit = 1,
  PN5180_TS_Transmitting = 2,
  PN5180_TS_WaitReceive = 3,
  PN5180_TS_WaitForData = 4,
  PN5180_TS_Receiving = 5,
  PN5180_TS_LoopBack = 6,
  PN5180_TS_RESERVED = 7
} PN5180TransceiveState_t;

typedef enum {
  NFC_WRITE_DATA,
  NFC_SEND_DATA
} nfc_task_action_t;

extern const char manufacturerCode[110][100];

/**
 * @brief  Initialize PN5180 device
 *
 * @note   If SPI fails to initialize, program will be aborted.
 */
void pn5180_init(void);

/**
 * @brief  Reset PN5180 device
 *
 * @return
 *     - ESP_OK = Success
 *     - ESP_TIMEOUT = IRQ_Status did not return IDLE
 * 
 * @note   A constant low level of at least 10 μs at the RESET_N pin starts the internal reset procedure. When the PN5180 has finished the start_up, a IDLE_IRQ is raised and the IC is ready to receive commands on the host interface.
 */
esp_err_t pn5180_reset(void);
esp_err_t pn5180_command(uint8_t *sendBuffer, size_t sendBufferLen, uint8_t *recvBuffer, size_t recvBufferLen);

////////////////////////////////////////////////
// PN5180 direct commands with host interface //
////////////////////////////////////////////////

/**
 * @brief  This command, 0x00, is used to write a 32-bit value (little endian) to a configuration register.
 *
 * @attention The address of the register must exist. If the condition is not fulfilled, an exception is raised.
 * 
 * @param  reg Register addess.
 * @param  value Register content. 
 *
 * @return
 *     - ESP_OK  Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *
 */
esp_err_t pn5180_writeRegister(uint8_t reg, uint32_t value);

/**
 * @brief  This command, 0x01, modifies the content of a register using a logical OR operation. The
content of the register is read and a logical OR operation is performed with the provided
mask. The modified content is written back to the register.
 *
 * @attention The address of the register must exist. If the condition is not fulfilled, an exception is raised.
 * 
 * @param  addr Register addess.
 * @param  mask OR_MASK 
 *
 * @return
 *     - ESP_OK  Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *
 */
esp_err_t pn5180_writeRegisterWithOrMask(uint8_t addr, uint32_t mask);

/**
 * @brief  This command, 0x02, modifies the content of a register using a logical AND operation. The
content of the register is read and a logical AND operation is performed with the provided
mask. The modified content is written back to the register.
 *
 * @attention The address of the register must exist. If the condition is not fulfilled, an exception is raised.
 * 
 * @param  addr Register addess.
 * @param  mask AND_MASK 
 *
 * @return
 *     - ESP_OK  Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *
 */
esp_err_t pn5180_writeRegisterWithAndMask(uint8_t addr, uint32_t mask);

/**
 * @brief  This command, 0x04, is used to read the content of a configuration register. The content of the register is returned in the 4 byte response.
 *
 * @attention The address of the register must exist. If the condition is not fulfilled, an exception is raised.
 * 
 * @param  reg Register address.
 * @param  value Register content. 
 *
 * @return
 *     - ESP_OK  Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *
 */
esp_err_t pn5180_readRegister(uint8_t reg, uint32_t *value);

/**
 * @brief  This command, 0x06, is used to write up to 255 bytes to the EEPROM. The field ‘EEPROM content’ contains the data to be written to EEPROM starting at the address given by byte ‘EEPROM Address’. The data is written in sequential order.
 *
 * @attention The EEPROM Address field must be in the range from 0 – 254, inclusive. The number of bytes within ‘Values’ field must be in the range from 1 – 255, inclusive. If the condition is not fulfilled, an exception is raised.
 * 
 * @param  addr Address in EEPROM from which write operation starts {EEPROM Address}
 * @param  buffer EEPROM content.
 *
 * @return
 *     - ESP_OK  Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *
 */
esp_err_t pn5180_writeEEprom(uint8_t addr, uint8_t *buffer, uint16_t len);

/**
 * @brief  This command, 0x07, is used to read data from EEPROM memory area. The field 'Address" indicates the start address of the read operation. The field Length indicates the number of bytes to read. The response contains the data read from EEPROM (content of the EEPROM); The data is read in sequentially increasing order starting with the given address.
 *
 * @param  addr Address in EEPROM from which write operation starts {EEPROM Address}.
 * @param  buffer EEPROM content.
 *
 * @return
 *     - ESP_OK  Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *
 */
esp_err_t pn5180_readEEprom(uint8_t addr, uint8_t *buffer, uint16_t len);

/**
 * @brief  This command, 0x09, writes data to the RF transmission buffer and starts the RF transmission. The parameter ‘Number of valid bits in last Byte’ indicates the exact number of bits to be transmitted for the last byte (for non-byte aligned frames).
 *
 * @attention Precondition: Host shall configure the Transceiver by setting the register SYSTEM_CONFIG.COMMAND to 0x3 before using the SEND_DATA command, as the command SEND_DATA is only writing data to the transmission buffer and starts the transmission but does not perform any configuration. The size of ‘Tx Data’ field must be in the range from 0 to 260, inclusive (the 0 byte length allows a symbol only transmission when the TX_DATA_ENABLE is cleared). ‘Number of valid bits in last Byte’ field must be in the range from 0 to 7. The command must not be called during an ongoing RF transmission. Transceiver must be in ‘WaitTransmit’ state with ‘Transceive’ command set. If the condition is not fulfilled, an exception is raised.
 * 
 * @param  data Array of up to 260 elements {Transmit data}.
 * @param  len Length of received data in bytes.
 * @param  validBits Number of valid bits in last Byte.
 * 
 * @return
 *     - ESP_OK  Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *
 */
esp_err_t pn5180_sendData(uint8_t *data, uint16_t len, uint8_t validBits);

/**
 * @brief  This command, 0x0A, reads data from the RF reception buffer, after a successful reception. The RX_STATUS register contains the information to verify if the reception had been successful. The data is available within the response of the command. The host controls the number of bytes to be read via the SPI interface.
 *
 * @attention The RF data had been successfully received. In case the instruction is executed without preceding an RF data reception, no exception is raised but the data read back from the reception buffer is invalid. If the condition is not fulfilled, an exception is raised.
 * 
 * @param  len Address in EEPROM from which write operation starts {EEPROM Address}
 * @param  buffer EEPROM content.
 * 
 * @return
 *     - ESP_OK  Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *
 */
uint8_t* pn5180_readData(int len);

////////////////////////////
// Prepare LPCD registers //
////////////////////////////

esp_err_t pn5180_prepareLPCD();

/**
 * @brief  This instruction, 0x0B, is used to switch the mode. It is only possible to switch from normal mode to Standby, LPCD or Autocoll mode. Switching back to normal mode is not possible using this instruction. The modes Standby, LPCD and Autocoll terminate on specific conditions. Once a configured mode (Standby, LPCD, Autocoll) terminates, normal mode is entered again. To force an exit from Standby, LPCD or Autocoll mode to normal mode, the host controller has to reset the PN5180.
 *
 * @attention Parameter ‘mode’ has to be in the range from 0 to 2, inclusive. Dependent on the selected mode, different parameters have to be passed:

 * 
 * @param  len Address in EEPROM from which write operation starts {EEPROM Address}
 * @param  buffer EEPROM content.
 * 
 * @return
 *     - ESP_OK  Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *
 */
esp_err_t pn5180_switchToLPCD(uint16_t wakeupCounterInMs);

/**
 * @brief  This instruction, 0x11, is used to load the RF configuration from EEPROM into the configuration registers. The configuration refers to a unique combination of "mode" (target/initiator) and "baud rate". The configurations can be loaded separately for the receiver (Receiver configuration) and transmitter (Transmitter configuration).
 *
 * @attention Parameter 'Transmitter Configuration' must be in the range from 0x0 - 0x1C, inclusive. If the transmitter parameter is 0xFF, transmitter configuration is not changed. Field 'Receiver Configuration' must be in the range from 0x80 - 0x9C, inclusive.
 * 
 * @param  len Address in EEPROM from which write operation starts {EEPROM Address}
 * @param  buffer EEPROM content.
 * 
 * @return
 *     - ESP_OK  Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *
 */
esp_err_t pn5180_loadRFConfig(uint8_t txConf, uint8_t rxConf);

/**
 * @brief This command, 0x16, is used to switch on the internal RF field. If enabled the TX_RFON_IRQ is set after the field is switched on.
 */
esp_err_t pn5180_setRF_on();

/**
 * @brief This command, 0x17, is used to switch off the internal RF field. If enabled, the TX_RFOFF_IRQ is set after the field is switched off.
 */
esp_err_t pn5180_setRF_off();

uint32_t pn5180_getIRQStatus();
esp_err_t pn5180_clearIRQStatus(uint32_t irqMask);

PN5180TransceiveState_t getTransceiveState();

void printIRQStatus(const char* tag, uint32_t irqStatus);

#endif /* PN5180_H */