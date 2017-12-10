FIRMWARE = firmware.bin
deps:
	@echo Installing deps
	yarn global add particle-cli

$(FIRMWARE):
	particle compile photon . --saveTo $(FIRMWARE)

build: $(FIRMWARE)

flash: build $(FIRMWARE)
	@echo Flashing firmware
	particle flash --usb $(FIRMWARE)
