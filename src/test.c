/**
 * @test.c
 *
 * @brief Test
 * @details Test
 *
 * @author Emiliano Gonzalez (egonzalez . hiperion @ gmail . com))
 * @version 0.1
 * @date 2023
 * @copyright MIT License
 * @see https://github.com/hiperiondev/libGS232
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <pty.h>
#include <termios.h>

#include "libGS232.h"

#define BUF_SIZE (32768)

int main(int, char const*[]) {
    gs232_t *context = NULL;
    uint8_t command;
    char *ret_str;
    int master, slave, r;
    char buf[BUF_SIZE];
    struct termios tty;

    gs232_init(&context);

    tty.c_iflag = (tcflag_t) 0;
    tty.c_lflag = (tcflag_t) 0;
    tty.c_cflag = CS8;
    tty.c_oflag = (tcflag_t) 0;

    int e = openpty(&master, &slave, buf, &tty, NULL);
    if (0 > e) {
        printf("Error: %s\n", strerror(errno));
        return -1;
    }

    printf("Slave PTY: %s\n", buf);

    memset(buf, 0, sizeof(buf));
    while ((r = read(master, buf, BUF_SIZE)) > 0) {
        command = gs232_parse_command(&context, buf, r);
        gs232_return_string(context, command, &ret_str);
        write(master, ret_str, strlen(ret_str));

        free(ret_str);
        memset(buf, 0, sizeof(buf));

        printf("CONTEXT:\n");
        printf("  azimuth: %d\n", context->azimuth);
        printf("  elevation: %d\n", context->elevation);
        printf("  b_protocol: %d\n", context->b_protocol);
        printf("  azimuth_nord_south: %d\n", context->azimuth_nord_south);
        printf("  is_450_degrees: %d\n", context->is_450_degrees);
        printf("  rotation_speed: %d\n", context->rotation_speed);
        printf("  memory_qty: %d\n", context->memory_qty);
        printf("  current_point: %d\n", context->current_point);

        for (uint16_t n = 0; n < context->memory_qty; n++)
            printf("  memory[%d]: %d\n", n, context->memory[n]);

        printf("\n");
    }

    close(slave);
    close(master);

    gs232_deinit(&context);

    return 0;
}
