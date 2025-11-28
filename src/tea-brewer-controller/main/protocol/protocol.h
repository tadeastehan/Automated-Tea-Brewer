/**
 * @file protocol.h
 * @brief Binary protocol for ESP-to-ESP communication
 *
 * Frame format:
 *   [START][LENGTH][CMD][DATA...][CRC]
 *   START:  0xAA (1 byte)
 *   LENGTH: Data length + CMD (1 byte, max 255)
 *   CMD:    Command ID (1 byte)
 *   DATA:   Command data (0-252 bytes)
 *   CRC:    XOR checksum of LENGTH, CMD, and DATA (1 byte)
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "motor/motor_control.h"

/* ============================================
   PROTOCOL CONSTANTS
   ============================================ */
#define PROTO_START_BYTE 0xAA
#define PROTO_MAX_DATA_SIZE 252
#define PROTO_MIN_FRAME_SIZE 4 // START + LENGTH + CMD + CRC

/* ============================================
   COMMAND IDs
   ============================================ */
typedef enum
{
    /* Commands (Display -> Motor Controller) */
    CMD_PING = 0x01,
    CMD_GET_STATUS = 0x02,
    CMD_CALIBRATE = 0x10,
    CMD_HOME = 0x11,
    CMD_MOVE_PERCENT = 0x20,
    CMD_MOVE_POSITION = 0x21,
    CMD_STOP = 0x22,
    CMD_ENABLE = 0x23,
    CMD_DISABLE = 0x24,
    CMD_SET_SGT = 0x30,
    CMD_GET_SGT = 0x31,
    CMD_SAVE_CALIBRATION = 0x40,
    CMD_CLEAR_CALIBRATION = 0x41,
    CMD_GET_TEMPERATURE = 0x50,
    CMD_GET_POT_PRESENCE = 0x51,

    /* Responses (Motor Controller -> Display) */
    RSP_ACK = 0x80,
    RSP_NAK = 0x81,
    RSP_STATUS = 0x82,
    RSP_POSITION = 0x83,
    RSP_SGT = 0x84,
    RSP_PONG = 0x85,
    RSP_TEMPERATURE = 0x86,
    RSP_POT_PRESENCE = 0x87,

    /* Async notifications (Motor Controller -> Display) */
    NOTIFY_MOVE_COMPLETE = 0xA0,
    NOTIFY_HOME_COMPLETE = 0xA1,
    NOTIFY_CALIBRATE_DONE = 0xA2,
    NOTIFY_ERROR = 0xAF,
} protocol_cmd_t;

/* ============================================
   ERROR CODES
   ============================================ */
typedef enum
{
    PROTO_ERR_NONE = 0x00,
    PROTO_ERR_INVALID_CMD = 0x01,
    PROTO_ERR_INVALID_PARAM = 0x02,
    PROTO_ERR_NOT_CALIBRATED = 0x03,
    PROTO_ERR_NOT_HOMED = 0x04,
    PROTO_ERR_BUSY = 0x05,
    PROTO_ERR_MOTOR_FAULT = 0x06,
} protocol_error_t;

/* ============================================
   DATA STRUCTURES
   ============================================ */

/* Status response data */
typedef struct __attribute__((packed))
{
    uint8_t state; // motor_state_t
    uint8_t error; // motor_error_t
    uint8_t flags; // Bit 0: calibrated, Bit 1: homed
    int32_t position_steps;
    int16_t position_percent; // Fixed point: value / 10 = percent
    int32_t total_steps;
    int8_t sgt_threshold;
} proto_status_t;

/* Move command data */
typedef struct __attribute__((packed))
{
    int16_t percent; // Fixed point: value / 10 = percent
} proto_move_percent_t;

typedef struct __attribute__((packed))
{
    int32_t position;
} proto_move_position_t;

/* Parsed frame */
typedef struct
{
    uint8_t cmd;
    uint8_t length;
    uint8_t data[PROTO_MAX_DATA_SIZE];
    bool valid;
} proto_frame_t;

/* ============================================
   FUNCTIONS
   ============================================ */

/**
 * @brief Calculate CRC for frame data
 * @param data Pointer to data (LENGTH, CMD, DATA...)
 * @param length Number of bytes
 * @return CRC byte
 */
uint8_t proto_calculate_crc(const uint8_t *data, uint8_t length);

/**
 * @brief Build a protocol frame
 * @param buffer Output buffer (must be at least length + 4 bytes)
 * @param cmd Command ID
 * @param data Data payload (can be NULL if no data)
 * @param data_len Length of data payload
 * @return Total frame length
 */
uint8_t proto_build_frame(uint8_t *buffer, uint8_t cmd, const uint8_t *data, uint8_t data_len);

/**
 * @brief Parse received bytes into a frame
 * @param parser Parser state (call proto_parser_init first)
 * @param byte Received byte
 * @param frame Output frame (filled when complete frame received)
 * @return true if complete frame received
 */
bool proto_parse_byte(uint8_t byte, proto_frame_t *frame);

/**
 * @brief Reset parser state
 */
void proto_parser_reset(void);

/**
 * @brief Build ACK response
 */
uint8_t proto_build_ack(uint8_t *buffer);

/**
 * @brief Build NAK response with error code
 */
uint8_t proto_build_nak(uint8_t *buffer, protocol_error_t error);

/**
 * @brief Build status response
 */
uint8_t proto_build_status(uint8_t *buffer, const motor_status_t *status);

/**
 * @brief Build position response
 */
uint8_t proto_build_position(uint8_t *buffer, int32_t steps, float percent);

/**
 * @brief Build SGT response
 */
uint8_t proto_build_sgt(uint8_t *buffer, int8_t sgt);

/**
 * @brief Build PONG response
 */
uint8_t proto_build_pong(uint8_t *buffer);

/**
 * @brief Build notification
 */
uint8_t proto_build_notify(uint8_t *buffer, uint8_t notify_type, uint8_t data);

/**
 * @brief Build temperature response
 * @param buffer Output buffer
 * @param object_temp Object temperature in °C
 * @param ambient_temp Ambient temperature in °C
 * @return Frame length
 */
uint8_t proto_build_temperature(uint8_t *buffer, float object_temp, float ambient_temp);

/**
 * @brief Build pot presence response
 * @param buffer Output buffer
 * @param is_present true if pot is present, false otherwise
 * @param distance_mm Current distance reading in mm
 * @return Frame length
 */
uint8_t proto_build_pot_presence(uint8_t *buffer, bool is_present, uint16_t distance_mm);

#endif // PROTOCOL_H