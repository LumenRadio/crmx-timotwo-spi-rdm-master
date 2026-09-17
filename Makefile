BUILD_DIR = build
ROOT_DIR = spi_rdm_master
SKETCH = $(ROOT_DIR)/spi_rdm_master.ino

PORT ?= /dev/ttyACM0
FQBN ?= arduino:sam:arduino_due_x_dbg
BAUD = 115200

.PHONY: all compile upload clean monitor

all: compile upload

monitor:
	@echo "Monitoring device: $(PORT)"
	arduino-cli monitor \
		--port $(PORT) \
		--config $(BAUD) \
		--timestamp

compile:
	@echo "Compiling sketch: $(SKETCH)"
	arduino-cli compile \
		--fqbn $(FQBN) \
		--build-path $(BUILD_DIR) \
		$(SKETCH)

upload: compile
	@echo "Uploading sketch: $(SKETCH)"
	arduino-cli upload \
		--port $(PORT) \
		--fqbn $(FQBN) \
		--input-dir $(BUILD_DIR) \
		$(SKETCH)

clean:
	rm -rf $(BUILD_DIR)
