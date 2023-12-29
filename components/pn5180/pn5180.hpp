#ifndef pn5180_h
#define pn5180_h

// PN5180 Commands
#define PN5180_WRITE_REGISTER                     (0x00)
#define PN5180_WRITE_REGISTER_OR_MASK             (0x01)
#define PN5180_WRITE_REGISTER_AND_MASK            (0x02)
#define PN5180_WRITE_REGISTER_MULTIPLE            (0x03)
#define PN5180_READ_REGISTER                      (0x04)
#define PN5180_READ_REGISTER_MULTIPLE             (0x05)
#define PN5180_WRITE_EEPROM                       (0x06)
#define PN5180_READ_EEPROM                        (0x07)
#define PN5180_WRITE_DATA                         (0x08)
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

// ISO14443 Select Commands
#define ISO14443_REQA (0x26)
#define ISO14443_WUPA (0x52)
#define ISO14443_REQB (0x26)
#define ISO14443_CASCADE1 (0x93)
#define ISO14443_CASCADE2 (0x95)
#define ISO14443_CASCADE3 (0x97)
#define MIFARE_READ (0x30)
#define MIFARE_WRITE (0xA0)

// ISO Protocol choices
#define ISO14443          	(1)
#define ISO15693         	  (2)
#define ISO18003         	  (3)

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
  EC_NO_CARD = -1,
  ISO15693_EC_OK = 0,
  ISO15693_EC_NOT_SUPPORTED = 0x01,
  ISO15693_EC_NOT_RECOGNIZED = 0x02,
  ISO15693_EC_OPTION_NOT_SUPPORTED = 0x03,
  ISO15693_EC_UNKNOWN_ERROR = 0x0F,
  ISO15693_EC_BLOCK_NOT_AVAILABLE = 0x10,
  ISO15693_EC_BLOCK_ALREADY_LOCKED = 0x11,
  ISO15693_EC_BLOCK_IS_LOCKED = 0x12,
  ISO15693_EC_BLOCK_NOT_PROGRAMMED = 0x13,
  ISO15693_EC_BLOCK_NOT_LOCKED = 0x14,
  ISO15693_EC_CUSTOM_CMD_ERROR = 0xA0
} ISO15693ErrorCode_t;

typedef enum {
  MIFARE_AUTH_SUCCESS = 0x00,
  MIFARE_AUTH_FAILED = 0x01,
  MIFARE_TIMEOUT = 0x02
} MifareAuth_t;

typedef enum {
  PN5180_OK = 0x00,
  PN5180_ERR_NO_CARD = 0x01,
  PN5180_ERR_NO_RESP = 0x02,
  PN5180_ERR_TIMEOUT = 0x03,
  PN5180_ERR_COLLISION = 0x04,
  PN5180_ERR_REGISTER = 0x05,
  PN5180_ERR_INVALID_PARAM = 0x06,
  PN5180_ERR_UNKNOWN = 0xFF
} PN5180Error_t;

typedef struct {
  uint8_t uid[16][10]; // MSBFIRST (ex: uid{8,7,6,5,4,3,2,1} = 1:2:3:4:5:6:7:8)
  uint8_t uidLength[16];
  uint8_t type[16];
  uint8_t manufacturer[16];
  uint8_t dsfid[16];
  uint8_t afi[16];
  uint8_t ic_ref[16];
  // The physical memory of an ISO15693 VICC is organized in the form of blocks or pages of fixed size. Up to 256 blocks can be addressed and a block size can be up to 32 bytes.
  uint16_t numBlocks[16];
  uint16_t blockSize[16];
  uint16_t startBlock[16];
  uint16_t endBlock[16];
  uint8_t blocksRead[16];
  uint8_t data[16][16];
} RFIDCard_t;

extern const char manufacturerCode[110][100];
extern const char afi_string[14][18];

class pn5180 {
private:
  gpio_num_t PN5180_MISO;
  gpio_num_t PN5180_MOSI;
  gpio_num_t PN5180_CLK;
  gpio_num_t PN5180_NSS;   // active low
  gpio_num_t PN5180_BUSY;
  gpio_num_t PN5180_RST;
  spi_bus_config_t pn5180_buscfg;
  spi_device_interface_config_t pn5180_devcfg;
  spi_device_handle_t dev;
  spi_host_device_t spi_host;

  uint8_t product[2]; // {minor ver, major ver}
  uint8_t firmware[2]; // {minor ver, major ver}
  uint8_t eeprom[2]; // {minor ver, major ver}
  //uint8_t manufacturer;
  uint8_t numCard;
  //uint8_t type;
  //uint8_t uid[160]; // MSBFIRST (ex: uid{8,7,6,5,4,3,2,1} = 1:2:3:4:5:6:7:8)
  //uint8_t uidLength;
  //uint8_t dsfid;
  //uint8_t afi;
  //uint8_t ic_ref;
  // The physical memory of an ISO15693 VICC is organized in the form of blocks or pages of fixed size. Up to 256 blocks can be addressed and a block size can be up to 32 bytes.
  //uint16_t numBlocks;
  //uint16_t blockSize;
  //uint16_t startBlock;
  //uint16_t endBlock;
  //uint8_t blockData[508];
  uint8_t readBuffer[508];
  uint8_t sak[10];
  RFIDCard_t card;

public:
  pn5180(uint8_t MISOpin, uint8_t MOSIpin, uint8_t CLKpin, uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin, spi_host_device_t host);
  uint8_t getNumCard(void) const;
  uint8_t getManufacturer(uint8_t cardNum);
  uint8_t getType(uint8_t cardNum);
  uint8_t getDsfid(uint8_t cardNum);
  uint8_t getAfi(uint8_t cardNum);
  uint8_t getICRef(uint8_t cardNum);
  uint8_t getBlockSize(uint8_t cardNum);
  uint8_t getNumBlocks(uint8_t cardNum);
  //uint8_t *getUID(void) const { return uid };

  void begin(void);
  void end(void);
  esp_err_t reset(void);
  esp_err_t readEEprom(uint8_t addr, uint8_t *buffer, uint16_t len);

  
  PN5180Error_t getInventory();
  PN5180Error_t getInventoryMultiple();
  void printInfo(uint8_t cardNum);
  void clearInfo(uint8_t cardNum);
  PN5180Error_t getData(uint8_t blockNo);
  PN5180Error_t getData(uint8_t blockNo, uint8_t numBlock);
  PN5180Error_t readData(void);
  PN5180Error_t writeData(const char* data);
  PN5180Error_t readSingleBlock(uint8_t protocol, uint8_t blockNo);
  PN5180Error_t writeSingleBlock(uint8_t blockNo, uint8_t* blockData, uint8_t len);
  PN5180Error_t readMultipleBlock(uint8_t protocol, uint8_t blockNo, uint8_t numBlock);

  void printIRQStatus(const char* tag, uint32_t irqStatus);
  void printRfStatus(const char* tag, uint32_t rfStatus);
  void printError(uint8_t err);
  void printUID(uint8_t cardNum);
  void printSingleBlock(uint8_t cardNum, uint8_t blockNum);
  void printMultipleBlock(uint8_t cardNum, uint8_t blockNum, uint8_t numBlock);
  void printData(uint8_t cardNum);

private:
  esp_err_t writeRegister(uint8_t reg, uint32_t value);
  esp_err_t writeRegisterOrMask(uint8_t addr, uint32_t mask);
  esp_err_t writeRegisterAndMask(uint8_t addr, uint32_t mask);
  esp_err_t writeRegisterMultiple(uint8_t *reg, uint8_t *action, uint8_t len, uint32_t *value);
  esp_err_t readRegister(uint8_t reg, uint32_t *value);
  esp_err_t readRegisterMultiple(uint8_t *reg, uint8_t len, uint32_t *value);
  esp_err_t writeEEprom(uint8_t addr, uint8_t *buffer, uint16_t len);
  esp_err_t sendData(uint8_t *data, uint16_t len, uint8_t validBits);
  PN5180Error_t sendData(const char *data);
  uint8_t* readData(int len);
  esp_err_t prepareLPCD();
  esp_err_t switchToLPCD(uint16_t wakeupCounterInMs);
  esp_err_t setupRF(uint8_t protocol);
  esp_err_t updateRF(uint8_t protocol);
  esp_err_t clearIRQStatus(uint32_t irqMask);
  esp_err_t loadRFConfig(uint8_t txConf, uint8_t rxConf);
  esp_err_t setRF_on();
  esp_err_t setRF_off();
  uint32_t getIRQStatus();
  PN5180TransceiveState_t getTransceiveState();

  // Low level communication functions that handle SPI
  esp_err_t transceiveCommand(uint8_t *sendBuffer, size_t sendBufferLen, uint8_t *recvBuffer, size_t recvBufferLen);
  esp_err_t spi_txn(spi_device_handle_t dev, const void *tx, int txLen, void *rx, int rxLen);
  esp_err_t busyWait(uint32_t timeout);

  // Direct inventory commands related to each spec
  PN5180Error_t iso14443poll(uint8_t atqaCmd);
  void iso14443GetSystemInfo(uint8_t ats);
  bool mifareAuthenticate(uint8_t blockNo);
  bool mifareReadBlock(uint8_t cardNum, uint8_t blockNo);
  bool mifareReadMultipleBlock(uint8_t cardNum, uint8_t blockNo, uint8_t numBlock);
  bool mifareWriteBlock(const char *blockData, uint8_t blockNo);
  void mifareHalt(void);
  ISO15693ErrorCode_t iso15693Poll(void);
  PN5180Error_t iso15693PollSingle(uint8_t cardNum);
  PN5180Error_t iso15693GetSystemInfo(uint8_t cardNum);
  bool iso15693ReadBlock(uint8_t cardNum, uint8_t blockNo);
  bool iso15693WriteBlock(uint8_t cardNum, uint8_t blockNo, uint8_t* blockData, uint8_t len);
  bool iso15693WriteBlock(const char *blockData, uint8_t blockNo);
  bool iso15693ReadMultipleBlock(uint8_t cardNum, uint8_t blockNo, uint8_t numBlock);
};

#endif /* PN5180_H */