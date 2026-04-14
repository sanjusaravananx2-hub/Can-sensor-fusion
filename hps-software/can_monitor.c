/**
 * @file    can_monitor.c
 * @brief   HPS application for monitoring CAN sensor data from FPGA
 * @details Reads processed sensor data via memory-mapped FPGA registers,
 *          displays real-time data on terminal, and logs to CSV file.
 *
 * Build:   arm-linux-gnueabihf-gcc -o can_monitor can_monitor.c -O2
 *          (or natively on DE1-SoC: gcc -o can_monitor can_monitor.c -O2)
 * Run:     sudo ./can_monitor [-l logfile.csv] [-r rate_hz]
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>
#include <time.h>
#include <getopt.h>

#include "fpga_regs.h"

static volatile int running = 1;

static void signal_handler(int sig)
{
    (void)sig;
    running = 0;
}

static void print_usage(const char *prog)
{
    fprintf(stderr, "Usage: %s [-l logfile.csv] [-r rate_hz]\n", prog);
    fprintf(stderr, "  -l  Log data to CSV file\n");
    fprintf(stderr, "  -r  Display update rate in Hz (default: 50)\n");
}

int main(int argc, char *argv[])
{
    const char *logfile = NULL;
    int rate_hz = 50;
    int opt;

    while ((opt = getopt(argc, argv, "l:r:h")) != -1) {
        switch (opt) {
        case 'l': logfile = optarg; break;
        case 'r': rate_hz = atoi(optarg); break;
        default:  print_usage(argv[0]); return 1;
        }
    }

    if (rate_hz < 1 || rate_hz > 1000) {
        fprintf(stderr, "Error: rate must be 1-1000 Hz\n");
        return 1;
    }

    /* Open /dev/mem for FPGA register access */
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("Error: cannot open /dev/mem (run as root)");
        return 1;
    }

    /* Map Lightweight HPS-to-FPGA bridge */
    void *lw_bridge = mmap(NULL, LW_H2F_SPAN, PROT_READ | PROT_WRITE,
                           MAP_SHARED, fd, LW_H2F_BASE);
    if (lw_bridge == MAP_FAILED) {
        perror("Error: mmap failed");
        close(fd);
        return 1;
    }

    /* FPGA component base (offset 0 within LW bridge — adjust if Qsys assigns different base) */
    void *fpga = lw_bridge;

    /* Open CSV log file if requested */
    FILE *csv = NULL;
    if (logfile) {
        csv = fopen(logfile, "w");
        if (!csv) {
            perror("Error: cannot open log file");
            munmap(lw_bridge, LW_H2F_SPAN);
            close(fd);
            return 1;
        }
        fprintf(csv, "timestamp_us,can_id,raw_x,raw_y,raw_z,filt_x,filt_y,filt_z,frame_count,errors\n");
    }

    /* Install signal handler for clean shutdown */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("╔══════════════════════════════════════════════════════════════════╗\n");
    printf("║           CAN Sensor Fusion Platform — HPS Monitor             ║\n");
    printf("╠══════════════════════════════════════════════════════════════════╣\n");
    printf("║  Update rate: %3d Hz  |  Logging: %-28s ║\n",
           rate_hz, logfile ? logfile : "disabled");
    printf("║  Press Ctrl+C to stop                                          ║\n");
    printf("╚══════════════════════════════════════════════════════════════════╝\n\n");

    uint32_t prev_frame_count = 0;
    int fps_counter = 0;
    int fps_display = 0;
    time_t last_sec = time(NULL);

    unsigned long sleep_us = 1000000 / rate_hz;

    while (running) {
        /* Read all registers */
        uint32_t status      = REG_READ(fpga, REG_STATUS);
        uint32_t can_id      = REG_READ(fpga, REG_CAN_ID);
        uint32_t timestamp   = REG_READ(fpga, REG_TIMESTAMP);
        int16_t  raw_x       = TO_INT16(REG_READ(fpga, REG_ACCEL_RAW_X));
        int16_t  raw_y       = TO_INT16(REG_READ(fpga, REG_ACCEL_RAW_Y));
        int16_t  raw_z       = TO_INT16(REG_READ(fpga, REG_ACCEL_RAW_Z));
        int32_t  filt_x      = (int32_t)REG_READ(fpga, REG_ACCEL_FILT_X);
        int32_t  filt_y      = (int32_t)REG_READ(fpga, REG_ACCEL_FILT_Y);
        int32_t  filt_z      = (int32_t)REG_READ(fpga, REG_ACCEL_FILT_Z);
        uint32_t frame_count = REG_READ(fpga, REG_FRAME_COUNT);
        uint32_t error_count = REG_READ(fpga, REG_ERROR_COUNT);

        /* Calculate frames per second */
        time_t now = time(NULL);
        if (now != last_sec) {
            fps_display = fps_counter;
            fps_counter = 0;
            last_sec = now;
        }
        if (frame_count != prev_frame_count) {
            fps_counter += (frame_count - prev_frame_count);
            prev_frame_count = frame_count;
        }

        /* Terminal output (overwrite same lines) */
        printf("\033[7A");  /* Move cursor up 7 lines */
        printf("┌─ CAN Bus Status ──────────────────────────────────────────────┐\n");
        printf("│  Last CAN ID: 0x%03X    Timestamp: %10u us               │\n",
               can_id & 0x7FF, timestamp);
        printf("│  Frames: %8u      FPS: %4d      Errors: %5u            │\n",
               frame_count, fps_display, error_count);
        printf("├─ Accelerometer ────────────────────────────────────────────────┤\n");
        printf("│  Raw:      X=%+6d   Y=%+6d   Z=%+6d                    │\n",
               raw_x, raw_y, raw_z);
        printf("│  Filtered: X=%+8d Y=%+8d Z=%+8d                │\n",
               filt_x, filt_y, filt_z);
        printf("└────────────────────────────────────────────────────────────────┘\n");

        /* CSV logging */
        if (csv && (status & 0x01)) {
            fprintf(csv, "%u,0x%03X,%d,%d,%d,%d,%d,%d,%u,%u\n",
                    timestamp, can_id & 0x7FF,
                    raw_x, raw_y, raw_z,
                    filt_x, filt_y, filt_z,
                    frame_count, error_count);
            fflush(csv);
        }

        usleep(sleep_us);
    }

    printf("\nShutting down...\n");
    printf("Total frames received: %u\n", REG_READ(fpga, REG_FRAME_COUNT));
    printf("Total errors: %u\n", REG_READ(fpga, REG_ERROR_COUNT));

    if (csv) {
        fclose(csv);
        printf("Log saved to: %s\n", logfile);
    }

    munmap(lw_bridge, LW_H2F_SPAN);
    close(fd);

    return 0;
}
