#include "radun.h"
#include <string.h>
#include <stdlib.h>

#define CLI_BUFFER_SIZE 128
#define CLI_MAX_ARGS 8

static char cli_buffer[CLI_BUFFER_SIZE];
static uint16_t cli_idx = 0;
static bool cli_echo = true;

/* CLI command structure */
typedef struct {
    const char *name;
    void (*handler)(int argc, char *argv[]);
    const char *help;
} cli_cmd_t;

/* CLI commands */
static void cli_help(int argc, char *argv[]);
static void cli_status(int argc, char *argv[]);
static void cli_arm(int argc, char *argv[]);
static void cli_disarm(int argc, char *argv[]);
static void cli_pid(int argc, char *argv[]);
static void cli_dump(int argc, char *argv[]);
static void cli_version(int argc, char *argv[]);
static void cli_reset(int argc, char *argv[]);

static const cli_cmd_t cli_commands[] = {
    {"help", cli_help, "Show available commands"},
    {"status", cli_status, "Show system status"},
    {"arm", cli_arm, "Arm motors (DANGEROUS!)"},
    {"disarm", cli_disarm, "Disarm motors"},
    {"pid", cli_pid, "Show/set PID gains: pid [roll|pitch|yaw] [p|i|d] [value]"},
    {"dump", cli_dump, "Dump sensor data"},
    {"version", cli_version, "Show firmware version"},
    {"reset", cli_reset, "Reset to bootloader"},
};

static const int cli_num_commands = sizeof(cli_commands) / sizeof(cli_cmd_t);

/* External variables */
extern bool armed;
extern bool fs_active;
extern imu_t imu;
extern bat_t bat;
extern pid_t pid;
extern crsf_t rx;

void cli_init(void) {
    cli_idx = 0;
    memset(cli_buffer, 0, sizeof(cli_buffer));
    cli_print("\r\nRADUN CLI v1.1\r\nType 'help' for commands\r\n");
    cli_print("\r\n> ");
}

void cli_print(const char *str) {
    if (usb_cdc_is_connected()) {
        usb_cdc_send((const uint8_t*)str, strlen(str));
    } else {
        for (const char *p = str; *p; p++) {
            uart_putc(USART6, *p);
        }
    }
}

static void cli_println(const char *str) {
    cli_print(str);
    cli_print("\r\n");
}

static void cli_process_line(char *line) {
    char *argv[CLI_MAX_ARGS];
    int argc = 0;

    /* Tokenize */
    char *token = strtok(line, " \t\r\n");
    while (token != NULL && argc < CLI_MAX_ARGS) {
        argv[argc++] = token;
        token = strtok(NULL, " \t\r\n");
    }

    if (argc == 0) {
        return;
    }

    /* Find command */
    for (int i = 0; i < cli_num_commands; i++) {
        if (strcmp(argv[0], cli_commands[i].name) == 0) {
            cli_commands[i].handler(argc, argv);
            return;
        }
    }

    cli_println("Unknown command. Type 'help' for list.");
}

void cli_feed(uint8_t c) {
    if (c == '\r' || c == '\n') {
        cli_buffer[cli_idx] = '\0';
        if (cli_idx > 0) {
            if (cli_echo) cli_print("\r\n");
            cli_process_line(cli_buffer);
            cli_idx = 0;
            cli_print("> ");
        }
    } else if (c == 8 || c == 127) {  /* Backspace */
        if (cli_idx > 0) {
            cli_idx--;
            if (cli_echo) {
                cli_print("\b \b");
            }
        }
    } else if (cli_idx < CLI_BUFFER_SIZE - 1) {
        if (c >= 32 && c < 127) {  /* Printable */
            cli_buffer[cli_idx++] = c;
            if (cli_echo) {
                char buf[2] = {c, 0};
                cli_print(buf);
            }
        }
    }
}

/* Command implementations */
static void cli_help(int argc, char *argv[]) {
    (void)argc; (void)argv;
    cli_println("Available commands:");
    for (int i = 0; i < cli_num_commands; i++) {
        char buf[64];
        snprintf(buf, sizeof(buf), "  %-12s - %s", cli_commands[i].name, cli_commands[i].help);
        cli_println(buf);
    }
}

static void cli_status(int argc, char *argv[]) {
    (void)argc; (void)argv;
    char buf[128];

    cli_println("--- System Status ---");

    snprintf(buf, sizeof(buf), "Armed: %s", armed ? "YES" : "NO");
    cli_println(buf);

    snprintf(buf, sizeof(buf), "Failsafe: %s", fs_active ? "YES" : "NO");
    cli_println(buf);

    snprintf(buf, sizeof(buf), "IMU Healthy: %s", imu.healthy ? "YES" : "NO");
    cli_println(buf);

    snprintf(buf, sizeof(buf), "Battery: %.2fV, %.1fA", bat.v, bat.i);
    cli_println(buf);

    snprintf(buf, sizeof(buf), "Cell Count: %dS", battery_get_cell_count());
    cli_println(buf);

    snprintf(buf, sizeof(buf), "RX Valid: %s, LQ: %d", rx.valid ? "YES" : "NO", rx.lq);
    cli_println(buf);

    snprintf(buf, sizeof(buf), "Throttle: %d", rx.thr);
    cli_println(buf);

    snprintf(buf, sizeof(buf), "Attitude: Roll=%.1f, Pitch=%.1f", imu.att[0], imu.att[1]);
    cli_println(buf);
}

static void cli_arm(int argc, char *argv[]) {
    (void)argc; (void)argv;
    if (armed) {
        cli_println("Already armed!");
    } else if (fs_active) {
        cli_println("Arming blocked: failsafe active");
    } else if (!rx.valid) {
        cli_println("Arming blocked: receiver not valid");
    } else if (battery_low_voltage()) {
        cli_println("Arming blocked: battery low");
    } else {
        cli_println("WARNING: Arming motors! Remove propellers first!");
        armed = true;
        cli_println("Armed.");
    }
}

static void cli_disarm(int argc, char *argv[]) {
    (void)argc; (void)argv;
    if (!armed) {
        cli_println("Already disarmed.");
    } else {
        armed = false;
        cli_println("Disarmed.");
    }
}

static void cli_pid(int argc, char *argv[]) {
    if (argc == 1) {
        /* Show all PID */
        char buf[64];
        cli_println("--- PID Gains ---");
        snprintf(buf, sizeof(buf), "Roll:  P=%.1f I=%.1f D=%.1f", pid.kp[0], pid.ki[0], pid.kd[0]);
        cli_println(buf);
        snprintf(buf, sizeof(buf), "Pitch: P=%.1f I=%.1f D=%.1f", pid.kp[1], pid.ki[1], pid.kd[1]);
        cli_println(buf);
        snprintf(buf, sizeof(buf), "Yaw:   P=%.1f I=%.1f D=%.1f", pid.kp[2], pid.ki[2], pid.kd[2]);
        cli_println(buf);
    } else if (argc == 4) {
        /* Set PID */
        int axis = -1;
        if (strcmp(argv[1], "roll") == 0) axis = 0;
        else if (strcmp(argv[1], "pitch") == 0) axis = 1;
        else if (strcmp(argv[1], "yaw") == 0) axis = 2;
        
        if (axis < 0) {
            cli_println("Invalid axis. Use: roll, pitch, yaw");
            return;
        }
        
        int term = -1;
        if (strcmp(argv[2], "p") == 0) term = 0;
        else if (strcmp(argv[2], "i") == 0) term = 1;
        else if (strcmp(argv[2], "d") == 0) term = 2;
        
        if (term < 0) {
            cli_println("Invalid term. Use: p, i, d");
            return;
        }
        
        float value = atof(argv[3]);
        
        if (term == 0) pid.kp[axis] = value;
        else if (term == 1) pid.ki[axis] = value;
        else pid.kd[axis] = value;
        
        char buf[64];
        snprintf(buf, sizeof(buf), "Set %s %s to %.1f", argv[1], argv[2], value);
        cli_println(buf);
    } else {
        cli_println("Usage: pid [roll|pitch|yaw] [p|i|d] [value]");
    }
}

static void cli_dump(int argc, char *argv[]) {
    (void)argc; (void)argv;
    char buf[160];

    cli_println("--- Sensor Data ---");
    snprintf(buf, sizeof(buf), "Gyro: %.1f, %.1f, %.1f dps", imu.dps[0], imu.dps[1], imu.dps[2]);
    cli_println(buf);
    snprintf(buf, sizeof(buf), "Acc:  %.2f, %.2f, %.2f g", imu.acc[0], imu.acc[1], imu.acc[2]);
    cli_println(buf);
    snprintf(buf, sizeof(buf), "Batt: %.2fV, %.1fA, cells=%d", bat.v, bat.i, battery_get_cell_count());
    cli_println(buf);
    snprintf(buf, sizeof(buf), "RX: valid=%s, lq=%d, thr=%d, aux=0x%04X", rx.valid ? "YES" : "NO", rx.lq, rx.thr, rx.aux);
    cli_println(buf);
}

static void cli_version(int argc, char *argv[]) {
    (void)argc; (void)argv;
    cli_println("RADUN Firmware v3.0");
    cli_println("Target: ACCIF435 (AT32F435G)");
    cli_println("Build: " __DATE__ " " __TIME__);
}

static void cli_reset(int argc, char *argv[]) {
    (void)argc; (void)argv;
    cli_println("Resetting to bootloader...");
    delay_ms(100);
    NVIC_SystemReset();
    cli_println("Reset command returned unexpectedly.");
}
