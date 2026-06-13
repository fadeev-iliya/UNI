#ifndef UNIPROTOCOL_H
#define UNIPROTOCOL_H

#include <Arduino.h>

#define UBC_SYNC_BYTE 0x00

// Command Codes
enum UniCommand : uint8_t {
    CMD_MOTORS        = 0x01,
    CMD_STOP          = 0x02,
    CMD_MOVE_DIST     = 0x03,
    CMD_ROTATE        = 0x04,
    CMD_GET_DISTANCE  = 0x05,
    CMD_GET_ANGLE     = 0x06,
    CMD_GET_ODOMETRY  = 0x07,
    CMD_MOTORS_ARC    = 0x08,
    CMD_MOVE_TIME     = 0x09,
    CMD_MOVE_ARC_TIME = 0x0A,
    CMD_MOTOR_LEFT    = 0x0B,
    CMD_MOTOR_RIGHT   = 0x0C,
    CMD_STOP_LEFT     = 0x0D,
    CMD_STOP_RIGHT    = 0x0E,
    CMD_RESET_DIST    = 0x0F,
    CMD_RESET_ANGLE   = 0x10,
    CMD_GET_ABS_X     = 0x11,
    CMD_GET_ABS_Y     = 0x12,
    CMD_GET_ABS_ANGLE = 0x13,
    CMD_GET_L_TICKS   = 0x14,
    CMD_GET_R_TICKS   = 0x15,
    CMD_DISPLAY_PRINT = 0x16,
    CMD_DISPLAY_CLEAR = 0x17,
    CMD_PRINT_ODOM    = 0x18,
    CMD_BLINK_LED     = 0x19,
    CMD_GET_BATTERY   = 0x1A,
    CMD_MOVE_ARC_DIST = 0x1B,
    CMD_DISPLAY_PRINT_NAMED = 0x1C,
    CMD_IS_MOVING     = 0x1D,
    CMD_MOTORS_SYNC   = 0x1E,
    CMD_ROTATE_TO     = 0x1F,
    CMD_MOVE_TO       = 0x20,
    CMD_MOVE_ARC_RADIUS = 0x21,
    CMD_SET_POSITION  = 0x22,
    CMD_HOLD_POSITION = 0x23
};

class UniProtocol {
public:
    // Подсчет CRC-8 (полином 0x07)
    static uint8_t crc8(const uint8_t *data, uint8_t len) {
        uint8_t crc = 0x00;
        for (uint8_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x80)
                    crc = (crc << 1) ^ 0x07;
                else
                    crc <<= 1;
            }
        }
        return crc;
    }

    // Кодирование COBS
    // output должен иметь размер как минимум (length + 2) для оверхеда COBS (обычно +1, но оставляем запас)
    // Возвращает количество байт в output
    static size_t cobsEncode(const uint8_t *input, size_t length, uint8_t *output) {
        size_t read_index = 0;
        size_t write_index = 1;
        size_t code_index = 0;
        uint8_t code = 1;

        while (read_index < length) {
            if (input[read_index] == 0) {
                output[code_index] = code;
                code = 1;
                code_index = write_index++;
                read_index++;
            } else {
                output[write_index++] = input[read_index++];
                code++;
                if (code == 0xFF) {
                    output[code_index] = code;
                    code = 1;
                    code_index = write_index++;
                }
            }
        }
        output[code_index] = code;
        return write_index;
    }

    // Декодирование COBS
    // Возвращает количество раскодированных байт (0 = ошибка)
    static size_t cobsDecode(const uint8_t *input, size_t length, uint8_t *output) {
        size_t read_index = 0;
        size_t write_index = 0;
        uint8_t code;
        uint8_t i;

        if (length == 0) return 0;

        while (read_index < length) {
            code = input[read_index];
            if (read_index + code > length && code != 1) return 0; // Ошибка данных

            read_index++;

            for (i = 1; i < code; i++) {
                output[write_index++] = input[read_index++];
            }
            if (code != 0xFF && read_index != length) {
                output[write_index++] = 0;
            }
        }
        return write_index;
    }
};

#endif
