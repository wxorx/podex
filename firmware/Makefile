TARGETS = podex attiny_podex

AS = avr-as
LD = avr-ld
OBJCOPY = avr-objcopy

AVRDUDE = avrdude
PROGRAMMER = usbasp
MCU = t2313

# Fuse values for ATTiny2313-20PU with 18.432MHz full-swing crystal:
#   LFUSE 0xEF: CKSEL=1111 (crystal 16.38-20.2MHz), SUT=10 (65ms slow start-up),
#               CKOUT off, CKDIV8 off (CPU clock = 18.432MHz)
#   HFUSE 0x9F: SPIEN programmed, RESET pin enabled, BOD disabled, WDT off
#   EFUSE 0xFF: SELFPRGEN off (flash self-programming not used by podex)
LFUSE = 0xEF
HFUSE = 0x9F
EFUSE = 0xFF

FLASH_FILE = attiny_podex.hex
FLASH_BIN = attiny_podex.bin

all: $(addsuffix .hex,$(TARGETS))

define TARGET_rules
$(1).o: $(1).s
	$(AS) -o $$@ $$<

$(1).elf: $(1).o
	$(LD) -o $$@ $$^

$(1).hex: $(1).elf
	$(OBJCOPY) -R .eeprom -O ihex $$< $$@
endef

$(foreach t,$(TARGETS),$(eval $(call TARGET_rules,$(t))))

write_flash: $(FLASH_FILE)
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U flash:w:$(FLASH_FILE):i

read_flash:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U flash:r:$(FLASH_BIN):r

erase:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -e

set_fuses:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m -U efuse:w:$(EFUSE):m

get_fuses:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -U lfuse:r:-:i -U hfuse:r:-:i -U efuse:r:-:i

test:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -v

help:
	@echo "Targets: write_flash, read_flash, erase, set_fuses, get_fuses, test, help, clean"

clean:
	rm -f $(addsuffix .o,$(TARGETS)) $(addsuffix .elf,$(TARGETS)) $(addsuffix .hex,$(TARGETS)) $(FLASH_BIN)

.PHONY: all clean write_flash read_flash erase set_fuses get_fuses
