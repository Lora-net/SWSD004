# How to flash BLE stack in the tracker
Here is the process to program the BLE stack into the STM32WB55CG M0 core 

- Install STM32CubeProgrammer available here : https://www.st.com/en/development-tools/stm32cubeprog.html
- copy FUS and Stack binaries in PATH\STM32CubeProgrammer\bin
- Put STM boot0 to VCC, to put STM32 in bootloader mode
- Open a CMD
	- Go in the folder C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin
	- Run the following command lines :
		- `STM32_Programmer_CLI.exe -c port=usb1 -fwdelete`
		- `STM32_Programmer_CLI.exe -c port=usb1 -r32 0x20030030 1`: read the FUSversion
			- if @0x20030030: 00050300: FUSv0.5.3:
				- `STM32_Programmer_CLI.exe -c port=usb1 -fwupgrade stm32wb5x_FUS_fw_for_fus_0_5_3.bin 0x080EC000 firstinstall=0.`
			- if @0x20030030: 010X0Y00: FUSv1.x.y: 
				- `STM32_Programmer_CLI.exe -c port=usb1 -fwupgrade stm32wb5x_FUS_fw.bin 0x080EC000 firstinstall=0`
			- if @0x20030030: 01020X00: FUSv1.2.0 => We’re done
	- Update the BLE stack : 
		- `STM32_Programmer_CLI.exe -c port=usb1 -fwupgrade stm32wb5x_BLE_Stack_full_fw.bin 0x080CB000 firstinstall=1`
-	Put STM boot0 to GND, to put STM32 in run mode