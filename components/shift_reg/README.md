| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- |

# shift_reg component
Utilizes SN74HC165n and SN74HC595n shift registers to expand the IO capabilities of the prop.

## How to configure
This component can be configured by opening the ESP-IDF Configuration editor and navigating to "Prop Shift Register Options".

## Configuration Options

| Setting                                           | Type | Default |
|---------------------------------------------------|------|---------|
| [Enable shift registers](#enable-shift-registers) | bool | true    |
| [Clock pin](#clock-pin)                           | int  | 15      |
| [SN74HC165n Load pin](#sn74hc165n-load-pin)       | int  | 2       |
| [SN74HC165n Data pin](#sn74hc165n-data-pin)       | int  | 13      |
| [SN74HC595n Latch pin](#sn74hc595n-latch-pin)     | int  | 14      |
| [SN74HC595n Data pin](#sn74hc595n-data-pin)       | int  | 12      |

### Enable shift registers
This will allow the prop to utilize additional inputs and outputs provided by the shift registers. 8 per register.

### Clock pin
Can be any unused output capable pin.

### SN74HC165n Load pin
Can be any unused output capable pin.

### SN74HC165n Data pin
Can be any unused output capable pin.

### SN74HC595n Latch pin
Can be any unused output capable pin.

### SN74HC595n Data pin
Can be any unused output capable pin.

## API reference
For simplification, a SN74HC165n will be referred to as an input shift register and a SN74HC595n will be an output shift register. The following functions can be called to utilize this component:

### bool shift_read(uint8_t pin)
Returns the value read from the specified pin on the input shift registers. Pins are specified in order from 0 *(reg#0, pinA)* to **NUM_SN74HC165n** - 1 *(reg#NUM_PISO, pinH)*.

### void shift_write(uint8_t pin, bool val)
Writes val to the specified pin on the output shift registers. Pins are specified in order from 0 *(reg#0, pinA)* to **NUM_SN74HC595n** - 1 *(reg#NUM_SIPO, pinH)*. Note that the changes will not be reflected until **shift_show()** is called.

### void shift_show(void)
Shows the changes on the output shift register pins written via **shift_write()**

## Example usage:
```
if(shift_read(12)){   // If input shift register 1, pin 4 is HIGH...
    shift_write(3,1); // Write HIGH to output shift register 0, pin 3
}
else{                 // Otherwise...
    shift_write(3,0); // Write LOW to output shift register 0, pin 3
}
shift_show();         // Show the state changes on output shift registers
```