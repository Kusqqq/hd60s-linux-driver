// HD60S SPI Flash Dump Tool
// Based on Opus 4.8 static analysis of Elgato yPushFile3.dll::updateSpiRom
//
// Uses vendor commands bRequest=0x13 (OUT) + bRequest=0x64 (IN)
// SPI READ header: [0x66, 0xC3, 0x10, addr_lo, addr_mid, addr_hi]
// Returns 64 bytes per transaction from W25Q32JV SPI flash
//
// Usage: sudo ./spi_dump [start_addr=0] [length=1024]
//
// CAVEATS (per Opus 4.8 analysis):
// - bRequest 0x13/0x64 verified in DLL but not traced to actual libusb call
// - MCU may only honor 0xC366 in "update mode" entered by prior resetUb658/recoveryFromSuspend
// - PI5C3257 mux switching state unknown outside firmware-update mode

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>

#define VID 0x0fd9
#define PID 0x0074

static int hd60s_read_spi(libusb_device_handle* h, uint32_t addr, uint8_t out[64]) {
    uint8_t setup[6] = {
        0x66, 0xC3, 0x10,
        (uint8_t)(addr & 0xff),
        (uint8_t)((addr >> 8) & 0xff),
        (uint8_t)((addr >> 16) & 0xff)
    };
    int r = libusb_control_transfer(h, 0x40, 0x13, 0, 0, setup, sizeof(setup), 1000);
    if (r < 0) {
        fprintf(stderr, "  OUT bReq=0x13 failed: %d (%s)\n", r, libusb_error_name(r));
        return r;
    }
    usleep(2000);
    r = libusb_control_transfer(h, 0xC0, 0x64, 0, 0, out, 64, 1000);
    if (r < 0) {
        fprintf(stderr, "  IN bReq=0x64 failed: %d (%s)\n", r, libusb_error_name(r));
        return r;
    }
    return r;
}

int main(int argc, char** argv) {
    uint32_t start_addr = 0;
    uint32_t length = 1024;

    if (argc >= 2) start_addr = (uint32_t)strtoul(argv[1], NULL, 0);
    if (argc >= 3) length = (uint32_t)strtoul(argv[2], NULL, 0);
    length = (length + 63) & ~63u;

    fprintf(stderr, "HD60S SPI Flash Dump (Opus 4.8 RE hypothesis)\n");
    fprintf(stderr, "  Start addr: 0x%06x\n", start_addr);
    fprintf(stderr, "  Length:     %u bytes (rounded to 64B)\n", length);

    libusb_init(NULL);
    libusb_device_handle* h = libusb_open_device_with_vid_pid(NULL, VID, PID);
    if (!h) {
        fprintf(stderr, "HD60S (0fd9:0074) not found\n");
        libusb_exit(NULL);
        return 1;
    }

    libusb_detach_kernel_driver(h, 0);
    int r = libusb_claim_interface(h, 0);
    if (r < 0) {
        fprintf(stderr, "claim_interface(0) failed: %s\n", libusb_error_name(r));
        libusb_close(h);
        libusb_exit(NULL);
        return 1;
    }

    fprintf(stderr, "Reading %u bytes from 0x%06x...\n", length, start_addr);
    uint8_t buf[64];
    uint32_t read_ok = 0;
    for (uint32_t i = 0; i < length; i += 64) {
        int rr = hd60s_read_spi(h, start_addr + i, buf);
        if (rr < 0) {
            fprintf(stderr, "SPI read failed at 0x%06x (aborting)\n", start_addr + i);
            break;
        }
        fwrite(buf, 1, rr, stdout);
        read_ok += rr;
        if ((i % 4096) == 0 && length >= 4096) {
            fprintf(stderr, "  progress: 0x%06x (%d%%)\n", start_addr + i, (int)(100 * i / length));
        }
    }
    fflush(stdout);
    fprintf(stderr, "Done. Read %u bytes.\n", read_ok);

    libusb_release_interface(h, 0);
    libusb_close(h);
    libusb_exit(NULL);
    return 0;
}
