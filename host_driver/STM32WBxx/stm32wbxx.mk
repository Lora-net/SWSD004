# --- The Clear BSD License ---
# Copyright Semtech Corporation 2021. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Semtech corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
# THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
# NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


######################################
# target
######################################

######################################
# source
######################################

# C sources

C_SOURCES +=  \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rng.c\
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_i2c.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_i2c_ex.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_lptim.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_ll_rtc.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rtc.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rtc_ex.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_ll_spi.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_spi.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_spi_ex.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_tim.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_tim_ex.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_uart.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_uart_ex.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_wwdg.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_iwdg.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_exti.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rcc.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_rcc_ex.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_flash.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_flash_ex.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_gpio.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_dma.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_dma_ex.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pwr.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_pwr_ex.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_cortex.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_ll_adc.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_adc.c \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_adc_ex.c\
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Src/stm32wbxx_hal_hsem.c\
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/CMSIS/Device/ST/STM32WBxx/Source/Templates/system_stm32wbxx.c \

# ASM sources

ASM_SOURCES +=  \
$(TOP_DIR)/host_driver/STM32WBxx/Drivers/CMSIS/Device/ST/STM32WBxx/Source/Templates/gcc/startup_stm32wb55xx_cm4.s

#######################################
# CFLAGS
#######################################

# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

C_DEFS +=  \
-DUSE_FULL_LL_DRIVER \
-DUSE_HAL_DRIVER \
-DSTM32WB55xx \

# C includes
C_INCLUDES +=  \
-I$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Inc\
-I$(TOP_DIR)/host_driver/STM32WBxx/Drivers/STM32WBxx_HAL_Driver/Inc/Legacy\
-I$(TOP_DIR)/host_driver/STM32WBxx/Drivers/CMSIS/Include \
-I$(TOP_DIR)/host_driver/STM32WBxx/Drivers/CMSIS/Device/ST/STM32WBxx/Include \
-I$(TOP_DIR)/host_driver/STM32WBxx/hal_config \

#######################################
# LDFLAGS
#######################################

# link script

ifeq ($(TARGET_MCU),STM32WB55xx)
LDSCRIPT = $(TOP_DIR)/host_driver/STMicroelectronics/gcc/stm32wb55xx_flash_cm4.ld
else
$(error Invalid target, must be STM32WB55xx or please modify makefile to add right .ld file)
endif
