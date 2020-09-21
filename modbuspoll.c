/*
 * modbuspoll.c - a Modbus master utility 
 *
 * Copyright (c) 2020, Francisco Oliveto <franciscoliveto@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h> /* for sleep() */
#include <getopt.h>
#include <limits.h> /* for CHAR_MIN */
#include <errno.h>
#include <stdbool.h>
#include <stdint.h> /* for int8_t, uint16_t, etc. */
#include <signal.h> /* for sigaction() */
#include <curses.h>

#include <modbus.h>

/* This defines how much overhead is contained in the 'infowin' window
   (box around the window takes two lines). */
#define INFOWIN_OVERHEAD 2

/* This is how many display fields are output in the 'infowin' window.
   Change this value if you add or remove fields from the 'infowin' window. */
#define INFOWIN_FIELDS 4

/* This is the minimum ysize we'll accept for the 'infowin' window. */
#define MIN_INFOWIN_YSIZE (INFOWIN_FIELDS + INFOWIN_OVERHEAD)

/* This is how far over in the 'infowin' window to indent the field
   descriptions. */
#define INFOWIN_DESC_OFFSET 2

/* This is how far over in the 'infowin' window to indent the field
   values. */
#define INFOWIN_VALUE_OFFSET 17

/* This is the width of the 'infowin' window. */
#define INFOWIN_WIDTH 75

/* This is the width of the 'datawin' window. */
#define DATAWIN_WIDTH 75

#define MAXIPv4SIZE 15

#define MIN_SLAVE_ID 1
#define MAX_SLAVE_ID 247

const char *version = "0.1";

/* These enum values cannot possibly conflict with the option values
   ordinarily used by commands, including CHAR_MAX + 1, etc.  Avoid
   CHAR_MIN - 1, as it may equal -1, the getopt end-of-options value. 
   (this has been borrowed from GNU coreutils/system.h) */
enum {
    GETOPT_HELP_CHAR = CHAR_MIN - 2,
    GETOPT_VERSION_CHAR = CHAR_MIN - 3
};

/* Modbus communication backend */
enum {
    TCP,
    UDP,
    RTU,
    ASCII
};

/* Modbus data types */
enum {
    COILS,
    DISCRETE_INPUTS,
    INPUT_REGISTERS,
    HOLDING_REGISTERS
};

static struct option longopts[] = {
    {"help", no_argument, NULL, GETOPT_HELP_CHAR},
    {"version", no_argument, NULL, GETOPT_VERSION_CHAR},
    {NULL, 0, NULL, 0}
};


static WINDOW *datawin, *infowin;

static uint16_t *tab_registers = NULL;
static uint8_t *tab_bits = NULL;
static modbus_t *ctx = NULL;


static void cleanup(void)
{
    if (!isendwin()) {
        /* Done with curses. */
        endwin();
    }
    free(tab_bits);
    free(tab_registers);
    if (ctx) {
        modbus_close(ctx);
        modbus_free(ctx);
    }
}

/* Cleanup and exit. */
static void die(int sig)
{
    cleanup();
    fprintf(stderr, "Caught signal %d\n", sig);
    exit(EXIT_SUCCESS);
}

const char *backend_str(int n)
{
    static char *str[] = {
        "Modbus TCP/IP",
        "Modbus UDP/IP",
        "Modbus RTU",
        "Modbus ASCII"
    };
    
    return str[n];
}

const char *type_str(int n)
{
    static char *str[] = {
        "Coils",
        "Discrete input",
        "16-bit input register",
        "16-bit holding register"
    };
    
    return str[n];
}

/* Validates whether ip is a valid IPv4 numbers-and-dots notation address. */
static bool is_valid_ipv4(const char *ip)
{
    /* TODO: to be implemented. */
    return true;
}

/* Print usage. */
void usage(void)
{
    fputs("Usage: modpoll [options] HOST\n\
\nMandatory arguments to long options are mandatory for short options too.\n\
  -m tcp                      Modbus TCP/IP (default)\n\
  -m udp                      Modbus UDP/IP\n\
  -m rtu                      Modbus RTU (default for serial communication)\n\
  -m ascii                    Modbus ASCII\n\
  -a integer                  Slave address (1-247, 1 is default)\n\
  -r integer                  Start data reference (1-65536, 100 is default)\n\
  -c integer                  Number of data values to read (1-125, 1 is default)\n\
  -t 1                        Coils data type\n\
  -t 2                        Discrete input data type\n\
  -t 3                        16-bit input register data type\n\
  -t 4                        16-bit holding register data type\n\
  -p integer                  TCP port number (502 is default)\n\
  -R integer                  Poll rate in seconds (1 is default)\n\
      --version               Output version information and exit\n\
      --help                  Display this help and exit\n\
", stdout);
}

/* Initialize curses and set up screen windows */
void windows_setup(void)
{
    int xsize, ysize;
    
    initscr();
    cbreak();
    noecho();

    getmaxyx(stdscr, ysize, xsize);
    (void)xsize;
    /* turn off cursor */
    curs_set(0);

    int row = 1;
    
    infowin = newwin(MIN_INFOWIN_YSIZE, INFOWIN_WIDTH, 0, 0);    

    mvwaddstr(infowin, row++, INFOWIN_DESC_OFFSET, "Connection:");
    mvwaddstr(infowin, row++, INFOWIN_DESC_OFFSET, "Slave:");
    mvwaddstr(infowin, row++, INFOWIN_DESC_OFFSET, "Communication:");
    mvwaddstr(infowin, row++, INFOWIN_DESC_OFFSET, "Data type:");
    box(infowin, 0, 0);

    datawin = newwin(ysize - MIN_INFOWIN_YSIZE, DATAWIN_WIDTH, MIN_INFOWIN_YSIZE, 0);
    mvwaddstr(datawin, 1, 2, "Polling slave... (Ctrl-C to stop)");
    box(datawin, 0, 0);

    wrefresh(infowin);
    wrefresh(datawin);
}

int main(int argc, char *argv[])
{
    int option;
    int poll_rate = 1;
    int slave_id = 1;
    int npoints = 1;
    int ref = 100;
    int port = MODBUS_TCP_DEFAULT_PORT;
    int backend = TCP;
    int type = INPUT_REGISTERS;
    char host[MAXIPv4SIZE+1];
    
    while ((option = getopt_long(argc, argv, "m:a:r:c:t:p:R:", longopts, NULL)) != -1) {
        switch (option) {
        case 'm':
            if (strcmp(optarg, "tcp") == 0) {
                backend = TCP;
            } else if (strcmp(optarg, "udp") == 0) {
                backend = UDP;
            } else if (strcmp(optarg, "rtu") == 0) {
                backend = RTU;
            } else if (strcmp(optarg, "ascii") == 0) {
                backend = ASCII;
            } else {
                fprintf(stderr, "Invalid communication mode %s\n", optarg);
                exit(EXIT_FAILURE);
            }
            break;
        case 'a':
            slave_id = atoi(optarg);
            break;
        case 'r':
            ref = atoi(optarg);
            break;
        case 'c':
            npoints = atoi(optarg);
            break;
        case 't':
            type = atoi(optarg);
            if (type < 1 || type > 4) {
                fprintf(stderr, "Invalid data type %d.\n", type);
                exit(EXIT_FAILURE);
            }
            type--; /* data types are indexed to 0 */
            break;
        case 'p':
            port = atoi(optarg);
            break;
        case 'R':
            poll_rate = atoi(optarg);
            break;
        case GETOPT_VERSION_CHAR:
            fprintf(stderr, "Version: %s\n", version);
            exit(EXIT_SUCCESS);
        case GETOPT_HELP_CHAR:
            usage();
            exit(EXIT_SUCCESS);
        case '?':
        default:
            usage();
            exit(EXIT_FAILURE);
        }
    }

    if (argc == optind) {
        fprintf(stderr, "HOST argument is required.\n");
        exit(EXIT_FAILURE);
    }

    if (slave_id < MIN_SLAVE_ID || slave_id > MAX_SLAVE_ID) {
        fprintf(stderr, "Invalid slave address %d.\n", slave_id);
        exit(EXIT_FAILURE);
    }

    /* XXX: e.g. one handler per signal. */
    struct sigaction sa;

    sa.sa_flags = 0;
    sa.sa_handler = die;

    sigfillset(&sa.sa_mask);
    sigaction(SIGHUP, &sa, NULL); /* Hangup detected on controlling terminal */
    sigaction(SIGINT, &sa, NULL); /* Interrupt from keyboard */
    sigaction(SIGTERM, &sa, NULL); /* Termination signal */

    switch (backend) {
    case TCP:
        if (!is_valid_ipv4(argv[optind])) {
            fputs("Invalid IPv4 HOST address\n", stderr);
            exit(EXIT_FAILURE);
        }
        strcpy(host, argv[optind]);
        
        ctx = modbus_new_tcp(host, port);
        break;
    case UDP:
    case RTU:
    case ASCII:
        fprintf(stderr, "%s mode is not yet supported.\n", backend_str(backend));
        exit(EXIT_SUCCESS);
    }

    if (ctx == NULL) {
        fputs("Unable to allocate libmodbus context.\n", stderr);
        exit(EXIT_FAILURE);
    }

    modbus_set_debug(ctx, FALSE);

    if (modbus_set_slave(ctx, slave_id) == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        modbus_free(ctx);
        exit(EXIT_FAILURE);
    }

    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        exit(EXIT_FAILURE);
    }

    
    switch (type) {
    case COILS:
    case DISCRETE_INPUTS:
        tab_bits = (uint8_t *)malloc(npoints * sizeof(uint8_t));
        if (tab_bits == NULL) {
            fputs("Cannot allocate memory for bits\n", stderr);
            modbus_close(ctx);
            modbus_free(ctx);
            exit(EXIT_FAILURE);
        }
        memset(tab_bits, 0, npoints * sizeof(uint8_t));
        break;
    case INPUT_REGISTERS:
    case HOLDING_REGISTERS:
        tab_registers = (uint16_t *)malloc(npoints * sizeof(uint16_t));
        if (tab_registers == NULL) {
            fputs("Cannot allocate memory for registers\n", stderr);
            modbus_close(ctx);
            modbus_free(ctx);
            exit(EXIT_FAILURE);
        }
        memset(tab_registers, 0, npoints * sizeof(uint16_t));
        break;
    default:
        ;
    }

    windows_setup();

    int row = 1;

    mvwprintw(infowin, row++, INFOWIN_VALUE_OFFSET, "%s", backend_str(backend));
    mvwprintw(infowin, row++, INFOWIN_VALUE_OFFSET,
              "address = %d, start reference = %d, count = %d", slave_id, ref, npoints);
    mvwprintw(infowin, row++, INFOWIN_VALUE_OFFSET,
              "%s, port %d, poll rate %d s", host, port, poll_rate);
    mvwprintw(infowin, row++, INFOWIN_VALUE_OFFSET, "%s", type_str(type));
    wrefresh(infowin);

    
    int address = ref - 1;
    for (;;) {
        int rc, i;
        
        switch (type) {
        case COILS:
            rc = modbus_read_bits(ctx, address, npoints, tab_bits);
            if (rc == -1) {
                cleanup();
                fprintf(stderr, "%s\n", modbus_strerror(errno));
                exit(EXIT_FAILURE);
            }

            for (i = 0; i < rc; i++) {
                mvwprintw(datawin, i + 3, 2, "[%d]: %d", ref + i, tab_bits[i]);
            }

            break;
        case DISCRETE_INPUTS:
            rc = modbus_read_input_bits(ctx, address, npoints, tab_bits);
            if (rc == -1) {
                cleanup();
                fprintf(stderr, "%s\n", modbus_strerror(errno));
                exit(EXIT_FAILURE);
            }

            for (i = 0; i < rc; i++) {
                mvwprintw(datawin, i + 3, 2, "[%d]: %d", ref + i, tab_bits[i]);
            }

            break;
        case INPUT_REGISTERS:
            rc = modbus_read_input_registers(ctx, address, npoints, tab_registers);
            if (rc == -1) {
                cleanup();
                fprintf(stderr, "%s\n", modbus_strerror(errno));
                exit(EXIT_FAILURE);
            }

            for (i = 0; i < rc; i++) {
                mvwprintw(datawin, i + 3, 2, "[%d]: %d", ref + i, tab_registers[i]);
            }

            break;
        case HOLDING_REGISTERS:
            rc = modbus_read_registers(ctx, address, npoints, tab_registers);
            if (rc == -1) {
                cleanup();
                fprintf(stderr, "%s\n", modbus_strerror(errno));
                exit(EXIT_FAILURE);
            }
            
            for (i = 0; i < rc; i++) {
                mvwprintw(datawin, i + 3, 2, "[%d]: %d", ref + i, tab_registers[i]);
            }
            
            break;
        default:
            ;
        }

        wrefresh(datawin);

        sleep(poll_rate);
    }
}
