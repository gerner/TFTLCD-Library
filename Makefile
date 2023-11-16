SHELL=/bin/bash
SOURCES=$(shell find . -name '*.cpp' -o -name '*.h')

LIBNAME=Adafruit_TFTLCD_Library

all: $(SOURCES)
	arduino-cli compile --fqbn arduino:renesas_uno:minima --library ./ examples/simple/

install: all
	./create_zip
	arduino-cli lib install --zip-path $(LIBNAME).zip
