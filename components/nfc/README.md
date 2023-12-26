| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- |

# nfc component
Utilizes a pn5180 NFC reader from NXP to detect ISO15693 tags.

## How to configure
This component can be configured by opening the ESP-IDF Configuration editor and navigating to "NFC Options".

## Configuration Options

| Setting                                       | Type            | Default |
|-----------------------------------------------|-----------------|---------|
| [Enable NFC](#enable-nfc)                     | bool            | true    |
| [Detect Multiple Tags](#detect-multiple-tags) | bool            | false   |

### Enable NFC
This will enable the NFC capabilities of the prop.

### Detect multiple tags
Multiple NFC tags can be placed on top of the NFC reader and read if enabled. Returns one NFC tag if only one present when false.

## API reference
The following functions and global variables can be called to utilize this component:

### ISO15693NFC_t nfc
NFC struct containing all of the data from the detected NFC tag. 

| Member       | Type | Description                                        |
|--------------|------|----------------------------------------------------|
| manufacturer | int  | The manufacturer of the tag                        |
| type         | int  | The type of tag, as a subclass of the manufacturer |
| uid[8]       | int  | Unique identifier of the tag                       |
| dsfid        | int  | Data structure format identifier                   |
| afi          | int  | Application family identifier                      |
| ic_ref       | int  | IC specific information                            |
| numBlocks    | int  | Number of programmable data blocks                 |
| blockSize    | int  | Number of bytes in each programmable data block    |
| blockData    | int* | Pointer to data from tag                           |

## Example usage:
```
ISO15693NFC_t correct_nfc;
// Fill struct with data for expected nfc...

bool isMatch = 1;
if(nfc.numBlock != nfc_correct.numBlock) isMatch = 0;
if(nfc.blockSize != nfc_correct.blockSize) isMatch = 0;
for(int i=0; i<nfc.numBlocks; i++){
    if(!isMatch) break;
    for(int j=0; j<nfc.blockSize; j++){
        uint8_t nfc_byte = nfc.blockData[i*nfc.blockSize + j];
        uint8_t correct_nfc_byte = nfc_correct.blockData[i*nfc.blockSize + j];
        if(nfc_byte != correct_nfc_byte){
            isMatch = 0;
            break;
        }
    }
}

if(isMatch){
    // Do a thing for the correct match
}
```