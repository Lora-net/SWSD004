# LoRa Basics Modem Geolocation SDK - SWSD004

The LoRa Basics Modem Geolocation SDK contains several examples to demonstrate the usage of LoRa Basic Modem features with geolocation operations.

This project contains a demonstration for a full-feature tracker application, as well as simple Wi-Fi and GNSS scanning examples, based on LoRa Basics Modem and geolocation middlewares which provide abstraction on top of LR11xx transceiver

## Changelog and current version

Here is the different components changelogs and current version included in this SDK :

-   [SWSD004 v1.2.0 Changelog](geolocation_middleware/CHANGELOG.md)
-   [Tracker application v2.2.0 Changelog](apps/demonstrations/tracker_application/CHANGELOG.md)
-   [Geolocation middleware v2.0.0 Changelog](geolocation_middleware/CHANGELOG.md)
-   [LoRa Basics Modem v3.2.4 Changelog](lora_basics_modem/lora_basics_modem/CHANGELOG.md)

## Examples

| Name                       | Description                                                                              | Documentation                                           |
| -------------------------- | ---------------------------------------------------------------------------------------- | ------------------------------------------------------- |
| Tracker application        | Tracker demo application running on LoRa Edge Tracker ref design                         | [README](apps/demonstrations/tracker_application/README.md)        |
| Geolocation - GNSS         | Perform GNSS scans and send results over LoRaWAN using GNSS middleware                   | [README](apps/examples/geolocation_gnss/README.md)      |
| Geolocation - Wi-Fi        | Perform Wi-Fi scans and send results over LoRaWAN using Wi-Fi middleware                 | [README](apps/examples/geolocation_wifi/README.md)      |
| Geolocation - GNSS & Wi-Fi | Perform GNSS and Wi-Fi scans and send results over LoRaWAN using geolocation middlewares | [README](apps/examples/geolocation_gnss_wifi/README.md) |
| Full almanac update        | Perform a full almanac update                                                            | [README](apps/examples/full_almanac_update/README.md)   |
| TX/RX continuous           | Generate a TX/RX continuous thanks to the test mode                                      | [README](apps/examples/tx_rx_continuous/README.md)      |


## Configuration

Each example has its own set of parameters - see `apps/examples/<example>/main_<example>.h`.

There is also [a common configuration file](apps/common/lorawan_key_config.h) where LoRaWAN parameters can be set:

* DevEUI
* JoinEUI
* AppKey
* Region

## Convert LoRa Basics Modem Edge application code in LoRa Basics Modem application code

In order to help a LoRa Basics Modem Edge application code convertion to a LoRa Basics Modem application code please refer to [ModemE_to_LBM](doc/ModemE_to_LBM.md) 

## Requirements

### Supported platforms

LoRa Basics Modem is platform independent and can be used with any MCU that fulfills the requirements.

This SDK is developed on the STMicroeletronics [NUCLEO-L476RG development board](https://www.st.com/en/evaluation-tools/nucleo-l476rg.html).

Different Semtech shields can configured at compile time:

* LR1110 / LR1120 see [README](shields/LR11XX/smtc_shield_lr11xx/README.md)
* Semtech [LR1110TRK1BKS](https://fr.semtech.com/products/wireless-rf/lora-edge/lr1110trk1bks) board and [LR1110TRK1CKS](https://fr.semtech.com/products/wireless-rf/lora-edge/lr1110trk1cks) board

### Toolchain

* [Keil MDK ARM](https://www2.keil.com/mdk5)
* [GNU Arm Embedded toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm)

The projects are known to compile with GCC arm-none-eabi toolchain v10.3.1.

### Firmware

The LoRa Basics Modem library requires the following firmware:

* LR1110 firmware version 0x0307 ([available here](https://github.com/Lora-net/radio_firmware_images/tree/master/lr1110/transceiver))
* LR1120 firmware version 0x0101 ([available here](https://github.com/Lora-net/radio_firmware_images/tree/master/lr1120/transceiver))

To update the transceiver with the desired firmware version, please use [the updater tool application](https://github.com/Lora-net/SWTL001).

## Getting started

### Configure

Before starting to build an example, check the parameters in [the common configuration file](apps/common/lorawan_key_config.h). Make sure that the LoRaWAN configuration is consistent with what is configured in your network server and in your LoRa Cloud account.

Parameters can be seen in the [debug output](#view-debug-output).

### Build

#### Keil MDK ARM

Each example and demonstration are delivered with a Keil project file - see `apps/examples/<example>/MDK-ARM/lbm_example_<example>.uvprojx` or `demonstrations/tracker_application/MDK-ARM/tracker_application.uvprojx`.

Launch Keil IDE, open the project file and compile it.

Each project has 3 targets ([Keil manual](https://www.keil.com/support/man/docs/uv4/uv4_ca_projtargfilegr.htm)), each one allowing to chose how cryptographic operations are performed and where the LoRaWAN parameters are:

| Target name                                | Cryptographic operations    | LoRaWAN parameters |
| ------------------------------------------ | --------------------------- | ------------------ |
| `<example>_crypto_sw`                      | Software                    | User-defined       |
| `<example>_crypto_lr1110`                  | LR11XX cryptographic engine | User-defined       |
| `<example>_crypto_lr1110_with_credentials` | LR11XX cryptographic engine | LR11XX-based       |

#### GNU Arm embedded toolchain

Examples are built from their respective subfolder in the `apps` directory. For instance, the makefile for the tracker application example is available in `$SDK_FOLDER/apps/demonstrations/tracker_application/Makefile`.

Build settings, compile time and configuration options are specified in the project's Makefile.

To build a project, simply run make

```shell
$ cd $SDK_FOLDER/apps/demonstrations/tracker_application/makefile
$ make -j

$ cd $SDK_FOLDER/apps/examples/geolocation_gnss/makefile
$ make -j
```

The output files of the build process are stored in the `build` folder with firmware binary file having the same name as the project with a .bin extension.

It is possible to choose the cryptographic mode with the CRYPTO variable:

| CRYPTO value            | Cryptographic operations    | LoRaWAN parameters | Default value for demonstration ans examples makefiles |
| ----------------------- | --------------------------- | ------------------ | ------------------------------------------------------ |
| SOFT                    | Software                    | User-defined       |                                                        |
| LR11XX                  | LR11XX cryptographic engine | User-defined       |                                                        |
| LR11XX_WITH_CREDENTIALS | LR11XX cryptographic engine | LR11XX-based       | :heavy_check_mark:                                     |

For instance, to build the project `geolocation_gnss` with software-based cryptographic operations and user-defined LoRaWAN parameters:

To build a project, simply run make

```shell
$ cd $SDK_FOLDER/apps/examples/geolocation_gnss/makefile
$ make CRYPTO=SOFT -j
```

The default value for all examples is set to `LR11XX`.

It is possible to select the platform mode with the MCU_BOARD & RADIO_BOARD variable:

It is possible to choose the shield with the `RADIO_BOARD` variable. The possible values are listed in the correspondind `README` file:

* [LR11xx shields](shields/LR11XX/smtc_shield_lr11xx/README.md)


by default for examples in `apps` folder the MCU_BOARD is NUCLEO_L476RG and the RADIO_BOARD is LR1110MB1DIS
by default for tracker application in `demonstrations/tracker_application` folder the MCU_BOARD is LR1110TRK1XKS and the RADIO_BOARD is LR1110TRK1XKS

For instance, to build the project `geolocation_gnss` on the LR1110TRK1XKS platform board

To build a project, simply run make

```shell
$ cd $SDK_FOLDER/apps/examples/geolocation_gnss/makefile
$ make RADIO_BOARD=LR1110TRK1XKS MCU_BOARD=LR1110TRK1XKS -j
```
| :exclamation:  The value LR1110TRK1XKS is only valid for the combination RADIO_BOARD=LR1110TRK1XKS with MCU_BOARD=LR1110TRK1XKS |
| ------------------------------------------------------------------------------------------------------------------------------- |

Compatibility matrix :

| Application                | NUCLEO_L476RG + LR11XX shields | LR1110TRK1XKS      |
| -------------------------- | ------------------------------ | ------------------ |
| Tracker application        | :x:                            | :heavy_check_mark: |
| Geolocation - GNSS         | :heavy_check_mark:             | :heavy_check_mark: |
| Geolocation - Wi-Fi        | :heavy_check_mark:             | :heavy_check_mark: |
| Geolocation - GNSS & Wi-Fi | :heavy_check_mark:             | :heavy_check_mark: |
| Full almanac update        | :heavy_check_mark:             | :heavy_check_mark: |

For instance, to build the project `Geolocation - GNSS` on the LR1120MB1DJS platform board with the software cryptographic mode, call the following commands:

```shell
$ cd $SDK_FOLDER/apps/examples/geolocation_gnss/makefile
$ make RADIO_BOARD=LR1120MB1DJS CRYPTO=SOFT
```

##### Note on re-build

When re-building an example after changing a compile-time parameter that affects LoRa Basics Modem library (like *CRYPTO*, *RADIO_BOARD* or *MIDDLEWARE*), it is important to force re-build of the LoRa Basics Modem library.

This can be achieved by cleaning prior the re-build of the example with:

```shell
$ make clean_lbm
```

##### Command line configuration

Additionnal configuration flags can be passed from command line to compiler with `EXTRAFLAGS` argument.
This is dedicated to define macros that can be defined like the following:

```bash
$ make EXTRAFLAGS='-D<MACRO>=<VALUE>'
```

Where `<MACRO>` is the macro name to set and `<VALUE>` is the value to set for this macro.
Not all macro can be redefined through this way. Refer to the readme of examples for the list of macro that can be defined.

Note that when using the configuration on command line, `make` cannot detect a change in configuration on next build.
Therefore `make clean` must be invoked when building after a build where configuration was provided on command line.

### Load

After a project is built, it can be loaded onto a device.

There are multiple ways to do it:

* Drag and drop the binary file to the USB drive listed by our OS - usually shows up as `NODE_L476RG`.
* Load it through the Keil IDE
* Use FOTA capability for `LR1110TRK1XKS` thought to LoRa Edge Config Android smartphone application (see user guide).

### View debug output

On the NUCLEO-L476RG development board, the firmware prints debug information to the UART that is connected via the ST-LINK to the host computer. The configuration is 921600/8-N-1:

* On Linux, this device usually shows up as `/dev/ttyACM0`
* On Windows, the port can be obtained from the device manager

For instance, using stty on Linux:

```shell
$ stty -echo raw speed 921600 < /dev/ttyACM0 && cat /dev/ttyACM0

INFO: Modem Initialization

INFO:######  ===== LoRa Basics Modem GNSS Geolocation example ===== ###### 

INFO: LoRaWAN version: 01.00.04.01
INFO: LoRa Basics Modem version: 03.01.08
INFO: ###### ===== BASICS MODEM RESET EVENT ==== ######
Reset count : 95
INFO: Application parameters:
INFO:   - LoRaWAN uplink Fport = 2
INFO:   - DM report interval   = 60
INFO:   - Confirmed uplink     = No
INFO: LoRaWAN parameters:
DevEUI - (8 bytes):
 00 16 C0 00 00 00 00 00
JoinEUI - (8 bytes):
 00 16 C0 01 FF FE 00 01
Class: A
Region: EU868

INFO: ###### ===== JOINED EVENT ==== ######
```
