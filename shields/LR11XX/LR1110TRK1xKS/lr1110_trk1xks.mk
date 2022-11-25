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

RADIO = lr1110

RP_MARGIN_DELAY = 33

######################################
# source
######################################

# C sources

C_SOURCES +=  \
$(TOP_DIR)/shields/LR11XX/radio_drivers_hal/lr11xx_hal.c\
$(TOP_DIR)/shields/LR11XX/LR1110TRK1xKS/BSP/peripherals/Leds/leds.c\
$(TOP_DIR)/shields/LR11XX/LR1110TRK1xKS/BSP/peripherals/lis2de12/lis2de12.c\
$(TOP_DIR)/shields/LR11XX/LR1110TRK1xKS/BSP/peripherals/usr_button/usr_button.c\
$(TOP_DIR)/shields/LR11XX/LR1110TRK1xKS/BSP/peripherals/hall_effect/hall_effect.c\
$(TOP_DIR)/shields/LR11XX/LR1110TRK1xKS/BSP/ral_bsp/ral_lr11xx_bsp.c\
$(TOP_DIR)/shields/LR11XX/LR1110TRK1xKS/BSP/board/lr1110_trk1xks_board.c

ifeq ($(MIDDLEWARE),yes)
C_SOURCES +=  \
$(TOP_DIR)/shields/LR11XX/LR1110TRK1xKS/BSP/mw_bsp/mw_bsp.c
endif

#######################################
# CFLAGS
#######################################

# C includes
C_INCLUDES +=  \
-I$(LORA_BASICS_MODEM)/smtc_modem_core/radio_drivers/lr11xx_driver/src \
-I$(LORA_BASICS_MODEM)/smtc_modem_core/smtc_ralf/src \
-I$(LORA_BASICS_MODEM)/smtc_modem_core/smtc_ral/src \
-I$(TOP_DIR)/shields/LR11XX/radio_drivers_hal \
-I$(TOP_DIR)/shields/LR11XX/smtc_lr11xx_board \
-I$(TOP_DIR)/shields/LR11XX/LR1110TRK1xKS/BSP/peripherals/lis2de12/ \
-I$(TOP_DIR)/shields/LR11XX/LR1110TRK1xKS/BSP/peripherals/Leds/ \
-I$(TOP_DIR)/shields/LR11XX/LR1110TRK1xKS/BSP/peripherals/usr_button/ \
-I$(TOP_DIR)/shields/LR11XX/LR1110TRK1xKS/BSP/peripherals/hall_effect/ \
-I$(TOP_DIR)/shields/LR11XX/LR1110TRK1xKS/BSP/board/ \
-I$(TOP_DIR)/shields/interface

