/**
 * @libGS232.c
 *
 * @brief Library for Yaesu Antenna Rotator GS-232 A and B protocol server
 * @details Library for Yaesu Antenna Rotator GS-232 A and B protocol server
 *
 * @author Emiliano Gonzalez (egonzalez . hiperion @ gmail . com))
 * @version 0.1
 * @date 2023
 * @copyright MIT License
 * @see https://github.com/hiperiondev/libGS232
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <limits.h>
#include <ctype.h>
#include <string.h>

#include "libGS232.h"

#ifdef DEBUG
#define EP(x) [x] = #x
const char *GS232_COMMAND_STR[] = {
        EP(GS232_CLOCKWISE_ROTATION),
        EP(GS232_UP_DIRECTION_ROTATION),
        EP(GS232_COUNTER_CLOCKWISE_ROTATION),
        EP(GS232_DOWN_DIRECTION_ROTATION),
        EP(GS232_CW_CCW_ROTATION_STOP),
        EP(GS232_UP_DOWN_DIRECTION_ROTATION_STOP),
        EP(GS232_RETURN_CURRENT_AZIMUTH),
        EP(GS232_RETURN_AZIMUTH_AND_ELEVATION),
        EP(GS232_TURN_DEGREES_AZIMUTH),
        EP(GS232_AUTOMATIC_TIMED_TRACKING_AZIMUTH),
        EP(GS232_TURN_DEGREES_AZIMUTH_AND_ELEVATION),
        EP(GS232_AUTOMATIC_TIMED_TRACKING_AZIMUTH_AND_ELEVATION),
        EP(GS232_TOTAL_NUMBER_OF_SETTING_ANGLES),
        EP(GS232_START_COMMAND_IN_TIME_INTERVAL),
        EP(GS232_ROTATION_SPEED_LOW),
        EP(GS232_ROTATION_SPEED_MIDDLE1),
        EP(GS232_ROTATION_SPEED_MIDDLE2),
        EP(GS232_ROTATION_SPEED_HIGH),
        EP(GS232_OFFSET_CALIBRATION_AZIMUTH),
        EP(GS232_OFFSET_CALIBRATION_ELEVATION),
        EP(GS232_FULL_SCALE_CALIBRATION_AZIMUTH),
        EP(GS232_FULL_SCALE_CALIBRATION_ELEVATION),
        EP(GS232_RETURN_CURRENT_ELEVATION),
        EP(GS232_ALL_STOP),
        EP(GS232_LIST_OF_COMMANDS1),
        EP(GS232_LIST_OF_COMMANDS2),
        EP(GS232_LIST_OF_COMMANDS3),
        EP(GS232_AZIMUTH_TO_360),
        EP(GS232_AZIMUTH_TO_450),
        EP(GS232_TOGGLE_AZIMUTH_NORD_SOUTH),
        EP(GS232_UNKNOWN_COMMAND)
};
#define DBG_HEX(str, len)  \
       fprintf(stderr, "DEBUG (fn %s): HEX =", __FUNCTION__); \
       for(uint32_t ___pos___dbg___hex___ = 0; ___pos___dbg___hex___< len;___pos___dbg___hex___++) \
           fprintf(stderr, " %02x", str[___pos___dbg___hex___]); \
       fprintf(stderr, "\n");
#define DBG_PRINT(fmt, args...)  \
       fprintf(stderr, "DEBUG (fn %s): ", __FUNCTION__); \
       fprintf(stderr, "" fmt, ##args)
#else
#define DBG_PRINT(fmt, args...)
#define DBG_HEX(str, len)
#endif

enum GS232_VALUE_TYPE {
    GS232_AZIMUTH,                // aaa
    GS232_TIME_AZIMUTH,           // ttt aaa aaa aaa ...
    GS232_AZIMUTH_ELEVATION,      // aaa eee
    GS232_TIME_AZIMUTH_ELEVATION, // ttt aaa eee aaa eee ...
};

#ifdef DEBUG
const char *GS232_VALUE_TYPE_STR[] = {
        EP(GS232_AZIMUTH),
        EP(GS232_TIME_AZIMUTH),
        EP(GS232_AZIMUTH_ELEVATION),
        EP(GS232_TIME_AZIMUTH_ELEVATION)
};
#endif

static uint32_t gs232_values(gs232_t **ctx, char *buffer, uint32_t buffer_len) {
    if ((buffer_len - 1) % 4 != 0)
        return GS232_FAIL;

    char *buffer_value = buffer + 1;
    char val[4];
    uint16_t cnt = 0;
    char *end = NULL;
    long i = 0;

    DBG_HEX(buffer_value, buffer_len - 1);

    (*ctx)->memory_qty = 0;
    val[3] = '\0';

    DBG_PRINT("values:\n");
    do {
        if((*ctx)->memory_qty > MEMORY_POINTS) {
            DBG_PRINT("GS232_TOOMANYVALUES\n");
            return GS232_TOOMANYVALUES;
        }

        if (!isdigit(buffer_value[cnt]) || !isdigit(buffer_value[cnt + 1]) || !isdigit(buffer_value[cnt + 2])) {
            DBG_PRINT("GS232_FAIL (%c %c %c)\n", buffer_value[cnt], buffer_value[cnt + 1], buffer_value[cnt + 2]);
            return GS232_FAIL;
        }

        val[0] = buffer_value[cnt++];
        val[1] = buffer_value[cnt++];
        val[2] = buffer_value[cnt++];
        ++cnt;

        errno = 0;
        i = strtol(val, &end, 10);
        if (errno == ERANGE) {
            DBG_PRINT("GS232_OUTOFRANGE\n");
            return GS232_OUTOFRANGE;
        }

        (*ctx)->memory[(*ctx)->memory_qty++] = (uint16_t) i;
        DBG_PRINT("       -> %d\n", (uint16_t) i);
    } while (buffer_value[cnt - 1] == ' ' || buffer_value[cnt - 1] != '\r' || cnt < buffer_len - 1);

    DBG_PRINT("GS232_OK\n");
    return GS232_OK;
}

static uint32_t gs232_check_values(gs232_t **ctx, uint8_t value_type) {
    DBG_PRINT("VALUE TYPE: %s\n", GS232_VALUE_TYPE_STR[value_type]);
    switch (value_type) {
        case GS232_AZIMUTH:
            DBG_PRINT("-- azimuth: %d\n", (*ctx)->memory[0]);
            if (((*ctx)->is_450_degrees && (*ctx)->memory[0] > 450) || (!(*ctx)->is_450_degrees && (*ctx)->memory[0] > 360)) {
                DBG_PRINT("GS232_OUTOFRANGE\n");
                return GS232_OUTOFRANGE;
            }
            break;

        case GS232_TIME_AZIMUTH:
            DBG_PRINT("-- time: %d\n", (*ctx)->memory[0]);
            if ((*ctx)->memory[0] > 999) {
                DBG_PRINT("GS232_OUTOFRANGE\n");
                return GS232_OUTOFRANGE;
            }

            for (uint16_t val = 1; val < (*ctx)->memory_qty; val++) {
                DBG_PRINT("-- azimuth: %d\n", (*ctx)->memory[val]);
                if (((*ctx)->is_450_degrees && (*ctx)->memory[val] > 450) || (!(*ctx)->is_450_degrees && (*ctx)->memory[val] > 360)) {
                    DBG_PRINT("GS232_OUTOFRANGE\n");
                    return GS232_OUTOFRANGE;
                }
            }
            break;

        case GS232_AZIMUTH_ELEVATION:
            for (uint16_t val = 0; val < (*ctx)->memory_qty; val += 2) {
                DBG_PRINT("-- azimuth: %d, elevation: %d\n", (*ctx)->memory[0], (*ctx)->memory[1]);

                if (((*ctx)->is_450_degrees && (*ctx)->memory[val] > 450) || (!(*ctx)->is_450_degrees && (*ctx)->memory[val] > 360)
                        || (*ctx)->memory[val + 1] > 180) {
                    DBG_PRINT("GS232_OUTOFRANGE\n");
                    return GS232_OUTOFRANGE;
                }
            }
            break;

        case GS232_TIME_AZIMUTH_ELEVATION:
            DBG_PRINT("-- time: %d\n", (*ctx)->memory[0]);
            if ((*ctx)->memory[0] > 999) {
                DBG_PRINT("GS232_OUTOFRANGE\n");
                return GS232_OUTOFRANGE;
            }

            for (uint16_t val = 1; val < (*ctx)->memory_qty; val += 2) {
                DBG_PRINT("-- azimuth: %d, elevation: %d\n", (*ctx)->memory[val], (*ctx)->memory[val + 1]);
                if (((*ctx)->is_450_degrees && (*ctx)->memory[val] > 450) || (!(*ctx)->is_450_degrees && (*ctx)->memory[val] > 360)
                        || (*ctx)->memory[val + 1] > 180) {
                    DBG_PRINT("GS232_OUTOFRANGE\n");
                    return GS232_OUTOFRANGE;
                }
            }
            break;

        default:
            DBG_PRINT("GS232_FAIL\n");
            return GS232_FAIL;
    }

    DBG_PRINT("GS232_OK\n");
    return GS232_OK;
}

uint8_t gs232_parse_command(gs232_t **ctx, char *buffer, uint32_t buffer_len) {
    DBG_PRINT("buffer[%d]: %s\n", buffer_len, buffer);
    DBG_HEX(buffer, buffer_len);

    if (buffer[buffer_len - 1] == '\n') // some software (not standard!)
        --buffer_len;

    if (buffer == NULL || buffer_len < 2 || buffer[buffer_len - 1] != '\r') {
        DBG_PRINT("FAIL AT START! (NULL= %s, LEN: %d, END: %02x)\n", (buffer == NULL) ? "true" : "false", buffer_len, buffer[buffer_len]);
        return GS232_FAIL;
    }

    uint8_t command = GS232_FAIL;

    DBG_PRINT("PARSE COMMAND: %c\n", toupper(buffer[0]));
    switch (toupper(buffer[0])) {
        case 'R':
            command =  GS232_CLOCKWISE_ROTATION;
            break;

        case 'U':
            command =  GS232_UP_DIRECTION_ROTATION;
            break;

        case 'L':
            command =  GS232_COUNTER_CLOCKWISE_ROTATION;
            break;

        case 'D':
            command =  GS232_DOWN_DIRECTION_ROTATION;
            break;

        case 'A':
            command =  GS232_CW_CCW_ROTATION_STOP;
            break;

        case 'E':
            command =  GS232_UP_DOWN_DIRECTION_ROTATION_STOP;
            break;

        case 'C':
            if (buffer[1] == '2') {
                command =  GS232_RETURN_AZIMUTH_AND_ELEVATION;
            } else {
                command =  GS232_RETURN_CURRENT_AZIMUTH;
            }
            break;

        case 'M': {
            if (buffer_len < 5 || gs232_values(ctx, buffer, buffer_len) != GS232_OK) {
                command = GS232_UNKNOWN_COMMAND;
                break;
            }

            if (buffer[4] == '\r') {
                if (gs232_check_values(ctx, GS232_AZIMUTH) != GS232_OK) {
                    command = GS232_UNKNOWN_COMMAND;
                    break;
                }

                command = GS232_TURN_DEGREES_AZIMUTH;
            } else {
                if (gs232_check_values(ctx, GS232_TIME_AZIMUTH) != GS232_OK) {
                    command = GS232_UNKNOWN_COMMAND;
                    break;
                }

                command = GS232_AUTOMATIC_TIMED_TRACKING_AZIMUTH;
            }
        }
            break;

        case 'W':
            if (buffer_len < 5 || gs232_values(ctx, buffer, buffer_len) != GS232_OK) {
                command =  GS232_UNKNOWN_COMMAND;
                break;
            }

            if (buffer[4] == '\r') {
                if (gs232_check_values(ctx, GS232_AZIMUTH_ELEVATION) != GS232_OK) {
                    command =  GS232_UNKNOWN_COMMAND;
                    break;
                }

                command =  GS232_TURN_DEGREES_AZIMUTH_AND_ELEVATION;
            } else {
                if (gs232_check_values(ctx, GS232_TIME_AZIMUTH_ELEVATION) != GS232_OK) {
                    command =  GS232_UNKNOWN_COMMAND;
                    break;
                }

                command =  GS232_AUTOMATIC_TIMED_TRACKING_AZIMUTH_AND_ELEVATION;
            }
            break;

        case 'N':
            command =  GS232_TOTAL_NUMBER_OF_SETTING_ANGLES;
            break;

        case 'T':
            command =  GS232_START_COMMAND_IN_TIME_INTERVAL;
            break;

        case 'X':
            switch (buffer[1]) {
                case '1':
                    (*ctx)->rotation_speed = 1;
                    command =  GS232_ROTATION_SPEED_LOW;
                    break;
                case '2':
                    (*ctx)->rotation_speed = 2;
                    command =  GS232_ROTATION_SPEED_MIDDLE1;
                    break;
                case '3':
                    (*ctx)->rotation_speed = 3;
                    command =  GS232_ROTATION_SPEED_MIDDLE2;
                    break;
                case '4':
                    (*ctx)->rotation_speed = 4;
                    command =  GS232_ROTATION_SPEED_HIGH;
                    break;
            }
            break;

        case 'O':
            if (buffer[1] == '2') {
                command =  GS232_OFFSET_CALIBRATION_AZIMUTH;
            } else {
                command =  GS232_OFFSET_CALIBRATION_ELEVATION;
            }
            break;

        case 'F':
            if (buffer[1] == '2') {
                command =  GS232_FULL_SCALE_CALIBRATION_ELEVATION;
            } else {
                command =  GS232_FULL_SCALE_CALIBRATION_AZIMUTH;
            }
            break;

        case 'B':
            command =  GS232_RETURN_CURRENT_ELEVATION;
            break;

        case 'S':
            command =  GS232_ALL_STOP;
            break;

        case 'H':
            switch (buffer[1]) {
                case '\r':
                    command = GS232_LIST_OF_COMMANDS1;
                    break;

                case '2':
                    command = GS232_LIST_OF_COMMANDS2;
                    break;

                ///////////////// GS-232B /////////////////
                case '3':
                    if (!(*ctx)->b_protocol)
                        command = GS232_UNKNOWN_COMMAND;
                    else
                        command = GS232_LIST_OF_COMMANDS3;
                    break;
            }
            break;

        //////////////////// GS-232B ////////////////////
        case 'P':
            if (!(*ctx)->b_protocol)
                command = GS232_UNKNOWN_COMMAND;
            else if (buffer[1] == '3' && buffer[2] == '6') {
                (*ctx)->is_450_degrees = false;
                command = GS232_AZIMUTH_TO_360;
            } else if (buffer[1] == '4' && buffer[2] == '5') {
                (*ctx)->is_450_degrees = true;
                command = GS232_AZIMUTH_TO_450;
            }
            break;

        case 'Z':
            if (!(*ctx)->b_protocol)
                command = GS232_UNKNOWN_COMMAND;
            else {
                (*ctx)->azimuth_nord_south = !(*ctx)->azimuth_nord_south;
                command = GS232_TOGGLE_AZIMUTH_NORD_SOUTH;
            }
            break;
    }

    DBG_PRINT("command: %s\n", GS232_COMMAND_STR[command]);
    return command;
}

uint8_t gs232_return_string(gs232_t *ctx, uint8_t command, char **ret_str) {
    DBG_PRINT("command: %s\n", GS232_COMMAND_STR[command]);
    switch (command) {
        case GS232_CLOCKWISE_ROTATION:
        case GS232_UP_DIRECTION_ROTATION:
        case GS232_COUNTER_CLOCKWISE_ROTATION:
        case GS232_DOWN_DIRECTION_ROTATION:
        case GS232_CW_CCW_ROTATION_STOP:
        case GS232_UP_DOWN_DIRECTION_ROTATION_STOP:
        case GS232_TURN_DEGREES_AZIMUTH:
        case GS232_AUTOMATIC_TIMED_TRACKING_AZIMUTH:
        case GS232_TURN_DEGREES_AZIMUTH_AND_ELEVATION:
        case GS232_AUTOMATIC_TIMED_TRACKING_AZIMUTH_AND_ELEVATION:
        case GS232_START_COMMAND_IN_TIME_INTERVAL:
        case GS232_ROTATION_SPEED_LOW:
        case GS232_ROTATION_SPEED_MIDDLE1:
        case GS232_ROTATION_SPEED_MIDDLE2:
        case GS232_ROTATION_SPEED_HIGH:
        case GS232_ALL_STOP:
        case GS232_AZIMUTH_TO_360:
        case GS232_AZIMUTH_TO_450:
        case GS232_TOGGLE_AZIMUTH_NORD_SOUTH:
        {
            const char tmp[] = "\r";
            (*ret_str) = strdup(tmp);
        }
            break;

        case GS232_LIST_OF_COMMANDS1: // H
        {
            const char tmp[] = ""
                    "---------- COMMAND LIST 1 ----------\n"
                    "R  Clockwise Rotation\n"
                    "L  Counter Clockwise Rotation\n"
                    "A  CW/CCW Rotation Stop\n"
                    "C  Antenna Direction Value\n"
                    "M  Antenna Direction Setting. MXXX\n"
                    "M  Time Interval Direction Setting.\n"
                    "   MTTT XXX XXX XXX ---\n"
                    "   (TTT = Step value)\n"
                    "   (XXX = Horizontal Angle)\n"
                    "T  Start Command in the time interval direction setting\n"
                    "   mode.\n"
                    "N  Total number of setting angles in “M” mode and traced\n"
                    "   number of all datas (setting angles)\n"
                    "X1 Rotation Speed 1 (Horizontal) Low\n"
                    "X2 Rotation Speed 2 (Horizontal) Middle 1\n"
                    "X3 Rotation Speed 3 (Horizontal) Middle 2\n"
                    "X4 Rotation Speed 4 (Horizontal) High\n"
                    "S  All Stop\n"
                    "O  Offset Calibration\n"
                    "F  Full Scale Calibration\r\0";

            (*ret_str) = strdup(tmp);
        }

            break;

        case GS232_LIST_OF_COMMANDS2: // H2
        {
            const char tmp[] = ""
                    "---------- HELP COMMAND 2 ----------\n"
                    "U  UP Direction Rotation\n"
                    "D  DOWN Direction Rotation\n"
                    "E  UP/DOWN Direction Rotation Stop\n"
                    "C2 Antenna Direction Value\n"
                    "W  Antenna Direction Setting.\n"
                    "   WXXX YYY\n"
                    "W  Time Interval Direction Setting.\n"
                    "   WTTT XXX YYY XXX YYY ---\n"
                    "   (TTT = Step value)\n"
                    "   (XXX = Horizontal Angle)\n"
                    "   (YYY = Elevation Angle)\n"
                    "T  Start Command in the time interval direction setting\n"
                    "   mode.\n"
                    "N  Total number of setting angle in “W” mode and traced\n"
                    "   number of all datas (setting angles)\n"
                    "S  All Stop\n"
                    "02 Offset Calibration\n"
                    "F2 Full Scale Calibration\n"
                    "B  Elevation Antenna Direction Value\r\0";

            (*ret_str) = strdup(tmp);
        }

            break;

        case GS232_LIST_OF_COMMANDS3: // H3
        {
            char *ptr;
            char tmp[] = ""
                    "---------- HELP COMMAND 3 ----------\n"
                    "P45 Set_mode 450 Degree\n"
                    "P36 Set_mode 360 Degree\n"
                    "Z   Switch N Center/S Center\n\n"
                    "--------------- MODE ---------------\n"
                    "mode ##0 Degree\n"
                    "@ Center\r\0";

            ptr = strchr(tmp, '@');
            *ptr = ctx->azimuth_nord_south ? 'S' : 'N';
            ptr = strchr(tmp, '#');
            if (ctx->is_450_degrees) {
                *ptr = '4';
                *(ptr + 1) = '5';
            } else {
                *ptr = '3';
                *(ptr + 1) = '6';
            }

            (*ret_str) = strdup(tmp);
        }

            break;

        case GS232_RETURN_CURRENT_AZIMUTH: // C
        {
            char tmp[10];
            sprintf(tmp, "%s%03d\r", ctx->b_protocol ? "AZ=" : "+0", ctx->azimuth);
            (*ret_str) = strdup(tmp);
        }

            break;

        case GS232_RETURN_AZIMUTH_AND_ELEVATION: // C2
        {
            char tmp[19];
            sprintf(tmp, "%s%03d%s%03d\r\n", ctx->b_protocol ? "AZ=" : "+0", ctx->azimuth, ctx->b_protocol ? "EL=" : "+0", ctx->elevation);
            (*ret_str) = strdup(tmp);
        }

            break;

        case GS232_RETURN_CURRENT_ELEVATION: // B
        {
            char tmp[10];
            sprintf(tmp, "%s%03d\r", ctx->b_protocol ? "EL=" : "+0", ctx->elevation);
            (*ret_str) = strdup(tmp);
        }
            break;

        case GS232_OFFSET_CALIBRATION_AZIMUTH: // O
        {
            // TODO: complete command
            const char tmp[] = "\r\0";
            (*ret_str) = strdup(tmp);
        }

            break;

        case GS232_OFFSET_CALIBRATION_ELEVATION: // O2
        {
            // TODO: complete command
            const char tmp[] = "\r\0";
            (*ret_str) = strdup(tmp);
        }

            break;

        case GS232_FULL_SCALE_CALIBRATION_AZIMUTH: // F
        {
            // TODO: complete command
            const char tmp[] = "\r\0";
            (*ret_str) = strdup(tmp);
        }

            break;

        case GS232_FULL_SCALE_CALIBRATION_ELEVATION: // F2
        {
            // TODO: complete command
            const char tmp[] = "\r\0";
            (*ret_str) = strdup(tmp);
        }

            break;

        case GS232_TOTAL_NUMBER_OF_SETTING_ANGLES: // N
        {
            char tmp[16];
            // TODO: current used point start on 0 or 1 ??
            sprintf(tmp, "%s%04d%s%04d\r\n", ctx->b_protocol ? "=" : "+", ctx->current_point + 1, ctx->b_protocol ? "=" : "+", ctx->memory_qty);
            (*ret_str) = strdup(tmp);
        }

            break;

        default: {
            const char tmp[] = "?>\r\0";
            (*ret_str) = strdup(tmp);
        }
            break;
    }

    DBG_PRINT("return string: %s\n", (*ret_str));
    DBG_HEX((*ret_str), strlen((*ret_str)));
    return GS232_OK;
}

uint8_t gs232_init(gs232_t **ctx) {
    *ctx = malloc(sizeof(gs232_t));
    if (*ctx == NULL)
        return GS232_FAIL;

    (*ctx)->azimuth = 0;
    (*ctx)->elevation = 0;
    (*ctx)->b_protocol = false;
    (*ctx)->azimuth_nord_south = false;
    (*ctx)->is_450_degrees = false;
    (*ctx)->rotation_speed = 1;
    (*ctx)->memory_qty = 0;
    (*ctx)->current_point = 0;

    for (uint16_t n = 0; n < MEMORY_POINTS; n++)
        (*ctx)->memory[n] = 0;

    return GS232_OK;
}

uint8_t gs232_deinit(gs232_t **ctx) {
    if (*ctx != NULL)
        free(*ctx);

    return GS232_OK;
}
