#-----------------------------------------------------------------------------
# Indicates library location
#-----------------------------------------------------------------------------

C_INCLUDES +=  \
-I$(LORA_BASICS_MODEM)/smtc_modem_api/ \
-I$(LORA_BASICS_MODEM)/smtc_modem_core/modem_config/ \
-I$(LORA_BASICS_MODEM)/smtc_modem_hal/ \
-I$(LORA_BASICS_MODEM)/smtc_modem_core/device_management/ \
-I$(LORA_BASICS_MODEM)/smtc_modem_core/modem_core/ \
-I$(LORA_BASICS_MODEM)/smtc_modem_core/lr1mac/src/ \
-I$(LORA_BASICS_MODEM)/smtc_modem_core/smtc_modem_services/headers/ \
-I$(LORA_BASICS_MODEM)/smtc_modem_core/radio_planner/src/ \


# Add printers
C_INCLUDES += -I$(TOP_DIR)/lora_basics_modem/printers/\


C_SOURCES +=  \
$(TOP_DIR)/lora_basics_modem/printers/smtc_modem_api_str.c

#-----------------------------------------------------------------------------
# Buid targets
#-----------------------------------------------------------------------------
.PHONY: build_basic_modem

build_basic_modem:
	$(MAKE) -C $(LORA_BASICS_MODEM) basic_modem RADIO=$(RADIO) RP_VERSION=$(RP_VERSION) MODEM_TRACE=$(MODEM_TRACE) CRYPTO=$(CRYPTO) MIDDLEWARE=$(MIDDLEWARE) DEBUG=$(DEBUG)

#-----------------------------------------------------------------------------
# Clean targets
#-----------------------------------------------------------------------------

.PHONY: clean_lbm
clean_lbm:
	$(MAKE) -C $(LORA_BASICS_MODEM) clean_all
