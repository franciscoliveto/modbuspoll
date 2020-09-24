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
#include <getopt.h>
#include <limits.h> /* for CHAR_MIN */
#include <errno.h>
#include <stdbool.h>
#include <stdint.h> /* for int8_t, uint16_t, etc. */
#include <signal.h> /* for sigaction() */
#include <curses.h>
#include <time.h>

#include <modbus.h>

enum {
    INFOWIN_OVERHEAD = 2, /* This defines how much overhead is contained in
                             the 'infowin' window (box around the window
                             takes two lines). */
    INFOWIN_FIELDS = 4, /* This is how many display fields are output in the
                           'infowin' window. Change this value if you add or
                           remove fields from the 'infowin' window. */
    INFOWIN_HEIGHT = (INFOWIN_FIELDS + INFOWIN_OVERHEAD), /* This is the minimum height
                                                             we'll accept for the
                                                             'infowin' window. */
    INFOWIN_OFFSET = 2, /* This is how far over in the 'infowin'
                           window to indent the field descriptions. */
    DATAWIN_OFFSET = 2 /* This is how far over in the 'datawin'
                          window to indent the field values. */
};

enum { MAXIPv4SIZE = 15 };
enum { NSEC_CONVERTION_FACTOR = 1000000 };

enum {
    MIN_SLAVE_ID = 1,
    MAX_SLAVE_ID = 247
};

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


static int poll_rate;
static int slave_id;
static int npoints;
static int ref;
static int port;
static int backend;
static int type;
static char host[MAXIPv4SIZE+1];

static const char *version = "0.1";

static WINDOW *datawin, *infowin;

static uint16_t *tab_registers = NULL;
static uint8_t *tab_bits = NULL;
static modbus_t *ctx = NULL;

void windows_setup();


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

/* Cope with terminal resize. */
static void resize(int sig)
{
    if (!isendwin()) {
        endwin();
        refresh();
        windows_setup();
    }
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
  -R integer                  Poll rate in milliseconds (1000 is default)\n\
      --version               Output version information and exit\n\
      --help                  Display this help and exit\n\
", stdout);
}

/* Initialize curses and set up screen windows */
void windows_setup(void)
{
    initscr();
    cbreak();
    noecho();
    
    curs_set(0); /* turn off cursor */

    infowin = newwin(INFOWIN_HEIGHT, 0, 0, 0);
    mvwprintw(infowin, 1, INFOWIN_OFFSET,
              "Connection:\t\t%s", backend_str(backend));
    mvwprintw(infowin, 2, INFOWIN_OFFSET,
              "Slave:\t\taddress = %d, start reference = %d, count = %d",
              slave_id, ref, npoints);
    mvwprintw(infowin, 3, INFOWIN_OFFSET,
              "Communication:\t%s, port %d, poll rate %d milliseconds",
              host, port, poll_rate);
    mvwprintw(infowin, 4, INFOWIN_OFFSET,
              "Data Type:\t\t%s", type_str(type));
    box(infowin, 0, 0);

    datawin = newwin(0, 0, INFOWIN_HEIGHT, 0);
    mvwaddstr(datawin, 1, DATAWIN_OFFSET, "Polling slave... (Ctrl-C to stop)");
    box(datawin, 0, 0);

    wrefresh(infowin);
    wrefresh(datawin);
}

int main(int argc, char *argv[])
{
    int option;

    poll_rate = 1000;
    slave_id = 1;
    npoints = 1;
    ref = 100;
    port = MODBUS_TCP_DEFAULT_PORT;
    backend = TCP;
    type = INPUT_REGISTERS;
    
    while ((option = getopt_long(argc, argv, "m:a:r:c:t:p:R:",
                                 longopts, NULL)) != -1) {
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

    struct sigaction sa;

    sa.sa_flags = 0;
    sigfillset(&sa.sa_mask);

    sa.sa_handler = die;
    sigaction(SIGHUP, &sa, NULL); /* Hangup detected on controlling terminal */
    sigaction(SIGINT, &sa, NULL); /* Interrupt from keyboard */
    sigaction(SIGTERM, &sa, NULL); /* Termination signal */

    sa.sa_handler = resize;
    sigaction(SIGWINCH, &sa, NULL); /* Window resizing signal */

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

    struct timespec ts;
    /* poll_rate is in milliseconds */
    ts.tv_sec = poll_rate / 1000;
    ts.tv_nsec = (poll_rate % 1000) * NSEC_CONVERTION_FACTOR;
    
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

        nanosleep(&ts, NULL);
    }
}
