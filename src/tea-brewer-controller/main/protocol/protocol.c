/**
 * @file protocol.c
 * @brief Binary protocol implementation
 */

#include "protocol.h"
#include <string.h>

/* Parser state */
typedef enum {
    PARSE_WAIT_START,
    PARSE_LENGTH,
    PARSE_CMD,
    PARSE_DATA,
    PARSE_CRC
} parse_state_t;

static parse_state_t parse_state = PARSE_WAIT_START;
static uint8_t parse_length = 0;
static uint8_t parse_index = 0;
static uint8_t parse_buffer[PROTO_MAX_DATA_SIZE + 3];

/* ============================================
   CRC CALCULATION
   ============================================ */
uint8_t proto_calculate_crc(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
    }
    return crc;
}

/* ============================================
   FRAME BUILDING
   ============================================ */
uint8_t proto_build_frame(uint8_t *buffer, uint8_t cmd, const uint8_t *data, uint8_t data_len)
{
    uint8_t idx = 0;
    
    buffer[idx++] = PROTO_START_BYTE;
    buffer[idx++] = data_len + 1;  // Length = data + cmd
    buffer[idx++] = cmd;
    
    if (data && data_len > 0) {
        memcpy(&buffer[idx], data, data_len);
        idx += data_len;
    }
    
    /* Calculate CRC over length, cmd, and data */
    buffer[idx] = proto_calculate_crc(&buffer[1], data_len + 2);
    idx++;
    
    return idx;
}

uint8_t proto_build_ack(uint8_t *buffer)
{
    return proto_build_frame(buffer, RSP_ACK, NULL, 0);
}

uint8_t proto_build_nak(uint8_t *buffer, protocol_error_t error)
{
    uint8_t err = (uint8_t)error;
    return proto_build_frame(buffer, RSP_NAK, &err, 1);
}

uint8_t proto_build_status(uint8_t *buffer, const motor_status_t *status)
{
    proto_status_t data;
    
    data.state = (uint8_t)status->state;
    data.error = (uint8_t)status->error;
    data.flags = 0;
    if (status->is_calibrated) data.flags |= 0x01;
    if (status->is_homed) data.flags |= 0x02;
    data.position_steps = status->position_steps;
    data.position_percent = (int16_t)(status->position_percent * 10.0f);
    data.total_steps = status->total_steps;
    data.sgt_threshold = status->sgt_threshold;
    
    return proto_build_frame(buffer, RSP_STATUS, (uint8_t*)&data, sizeof(data));
}

uint8_t proto_build_position(uint8_t *buffer, int32_t steps, float percent)
{
    uint8_t data[6];
    memcpy(data, &steps, 4);
    int16_t pct = (int16_t)(percent * 10.0f);
    memcpy(&data[4], &pct, 2);
    
    return proto_build_frame(buffer, RSP_POSITION, data, 6);
}

uint8_t proto_build_sgt(uint8_t *buffer, int8_t sgt)
{
    return proto_build_frame(buffer, RSP_SGT, (uint8_t*)&sgt, 1);
}

uint8_t proto_build_pong(uint8_t *buffer)
{
    return proto_build_frame(buffer, RSP_PONG, NULL, 0);
}

uint8_t proto_build_notify(uint8_t *buffer, uint8_t notify_type, uint8_t data)
{
    return proto_build_frame(buffer, notify_type, &data, 1);
}

uint8_t proto_build_temperature(uint8_t *buffer, float object_temp, float ambient_temp)
{
    uint8_t data[8];
    /* Pack temperatures as 32-bit floats */
    memcpy(&data[0], &object_temp, 4);
    memcpy(&data[4], &ambient_temp, 4);
    
    return proto_build_frame(buffer, RSP_TEMPERATURE, data, 8);
}

/* ============================================
   FRAME PARSING
   ============================================ */
void proto_parser_reset(void)
{
    parse_state = PARSE_WAIT_START;
    parse_length = 0;
    parse_index = 0;
}

bool proto_parse_byte(uint8_t byte, proto_frame_t *frame)
{
    frame->valid = false;
    
    switch (parse_state) {
        case PARSE_WAIT_START:
            if (byte == PROTO_START_BYTE) {
                parse_state = PARSE_LENGTH;
                parse_index = 0;
            }
            break;
            
        case PARSE_LENGTH:
            parse_length = byte;
            parse_buffer[0] = byte;
            parse_index = 1;
            if (parse_length == 0 || parse_length > PROTO_MAX_DATA_SIZE + 1) {
                proto_parser_reset();
            } else {
                parse_state = PARSE_CMD;
            }
            break;
            
        case PARSE_CMD:
            parse_buffer[parse_index++] = byte;
            if (parse_length == 1) {
                parse_state = PARSE_CRC;
            } else {
                parse_state = PARSE_DATA;
            }
            break;
            
        case PARSE_DATA:
            parse_buffer[parse_index++] = byte;
            if (parse_index >= parse_length + 1) {  // +1 for length byte
                parse_state = PARSE_CRC;
            }
            break;
            
        case PARSE_CRC:
            {
                uint8_t calculated_crc = proto_calculate_crc(parse_buffer, parse_index);
                if (calculated_crc == byte) {
                    frame->cmd = parse_buffer[1];
                    frame->length = parse_length - 1;  // Data length (without cmd)
                    if (frame->length > 0) {
                        memcpy(frame->data, &parse_buffer[2], frame->length);
                    }
                    frame->valid = true;
                }
                proto_parser_reset();
                return frame->valid;
            }
            break;
    }
    
    return false;
}