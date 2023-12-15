# UART-based firmware update application 

## Description

The application will update the LR1110 firmware using fragments received from the UART.

The companion `lr1110_firmware_update.py` script will read firmware data from a given file and send fragments to the application through the specified serial interface.

## Protocol

The application defines a basic TLV-based protocol to transfer the fragments. The protocol also defines a couple of auxiliary commands to check the updated firmware version.

### Message format

All commands start with a specific tag value, coded as a single byte, followed by the length of optional arguments, given in bytes and coded as a 16-bit value. Finally, the message may include optional arguments. All multi-byte values are little-endian (LSB first).

```
            +-----+-----+-----+-----+...+-----+
byte offset |  0  |  1  |  2  |               |
            +-----+-----+-----+-----+...+-----+
value       | tag |   length  |      args     |
            +-----+-----+-----+-----+...+-----+
```

### GET_VERSION

This command returns the firmware version. It only works if the LR1110 is either in bootloader mode or runs the Transceiver (TRX) firmware.

#### Request format

```
            +-----+-----+-----+
byte offset |  0  |  1  |  2  |
            +-----+-----+-----+
value       |  0  |     0     |
            +-----+-----+-----+
```

#### Reply format

```
            +-----+-----+-----+-----+-----+-----+-----+
byte offset |  0  |  1  |  2  |  3  |  4  |  5  |  6  |
            +-----+-----+-----+-----+-----+-----+-----+
value       |  0  |     4     | hw  |type |    fw     |
            +-----+-----+-----+-----+-----+-----+-----+
```

The reply message arguments are copied from the `lr11xx_bootloader_version_t` structure initialized by `lr11xx_bootloader_get_version()`. See the documentation for this structure and function for more information.

### WRITE_LR1110_UPDATE

This command transfers a fragment of firmware from the UART and installs it.

When the application receives this command for the first firmware fragment, it will automatically restart the LR1110 in bootloader mode and erase its flash memory.

When the application receives this command for the last firmware fragment, it will automatically restart the LR1110 and run the new firmware.

#### Request format

```
            +-----+-----+-----+-----+-----+-----+-----+.....+-----+
byte offset |  0  |  1  |  2  |  3  |  4  |  5  |  6  |     | N+5 |
            +-----+-----+-----+-----+-----+-----+-----+.....+-----+
value       |  2  |   2 + N   |fragment_id|     fragment bytes    |
            +-----+-----+-----+-----+-----+-----+-----+.....+-----+
```

`fragment_id` is an integer value. It starts at zero and is incremented for each new fragment sent.

N is the number of fragments bytes in the message. It must to be equal to 256 except for the last fragment which will be shorter.

#### 2.4.2. Reply format

The application will reply with the same fragment ID to acknowledge the request.

```
            +-----+-----+-----+-----+-----+
byte offset |  0  |  1  |  2  |  3  |  4  |
            +-----+-----+-----+-----+-----+
value       |  2  |     2     |fragment_id|
            +-----+-----+-----+-----+-----+
```

## Usage

### STM32 application

The application will display a status using the user LEDs of the LoRa Edge Tracker Reference design.

At startup all three LEDs (TX, RX) will blink twice to indicate that the application is ready.

The application will turn the RX LED on when it is executing a single UART command.

The application will turn the TX LED on when it receives the first firmware fragment and turn it off when it receives the last one.

### 3.2. Python script

The `lr1110_firmware_update.py` script requires that the PySerial module is installed on the host computer. See https://pypi.org/project/pyserial/ for mode information.

You can get binary firmware image files from this [GitHub repository](https://github.com/Lora-net/radio_firmware_images/tree/master/lr1110).

This script will request and display the new firmware version after installation, for this reason and because different firmware types have incompatible ways of indicating their version, the script must know which kind of firmware it is installing. The type of firmware is given as a command line argument along with the serial interface and firmware binary file.

Generic invocation syntax for the script:

```
lr1110_firmware_update.py <EVK_COM_PORT> <FILE> [<TYPE>]
```

For more information, call the script with the `-h` option:

```
$ python lr1110_firmware_update.py -h
usage: lr1110_firmware_update.py [-h] [--port PORT] [--file FILE]
                     
Update the LR1110 firmware of a device over a serial link

optional arguments:
  -h, --help           show this help message and exit
  --port PORT          Device name of the serial port
  --file FILE          Name of the LR1110 firmware binary file
```

Example of updating to a new Transceiver firmware:

```
$ python lr1110_firmware_update.py --port /dev/ttyACM0 --file lr1110_transceiver_0307.bin
Sending fragment 0... done.
Sending fragment 1... done.
Sending fragment 2... done.
...
Sending fragment 957... done.
Sending fragment 958... done.
Firmware version:
  HW: 0x22
  Type: 0x1
  FW: 0x307
```
