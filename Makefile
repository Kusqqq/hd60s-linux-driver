CC ?= gcc
CFLAGS ?= -O2 -Wall
LIBUSB_CFLAGS := $(shell pkg-config --cflags libusb-1.0)
LIBUSB_LIBS := $(shell pkg-config --libs libusb-1.0)
ALSA_LIBS := -lasound
PIPEWIRE_CFLAGS := $(shell pkg-config --cflags libpipewire-0.3)
PIPEWIRE_LIBS := $(shell pkg-config --libs libpipewire-0.3)

BINS = iso_capture audio_extract offline_parser ep81_probe spi_dump

all: $(BINS)

iso_capture: iso_capture.c
	$(CC) $(CFLAGS) $(LIBUSB_CFLAGS) $(PIPEWIRE_CFLAGS) $< -o $@ $(LIBUSB_LIBS) $(ALSA_LIBS) $(PIPEWIRE_LIBS) -lpthread -lsamplerate

audio_extract: audio_extract.c
	$(CC) $(CFLAGS) $< -o $@

offline_parser: offline_parser.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(BINS)

install-symlink:
	ln -sf $(CURDIR)/hd60s /usr/local/bin/hd60s

.PHONY: all clean install-symlink

ep81_probe: ep81_probe.c
	$(CC) $(CFLAGS) $(LIBUSB_CFLAGS) $< -o $@ $(LIBUSB_LIBS)

spi_dump: spi_dump.c
	$(CC) $(CFLAGS) $(LIBUSB_CFLAGS) $< -o $@ $(LIBUSB_LIBS)
