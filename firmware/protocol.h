#ifndef ADELINO_PROTOCOL_H
#define ADELINO_PROTOCOL_H

#include <stdint.h>

// ============================================================
// Binary Serial Protocol
// ============================================================

// Sync and type bytes
#define SYNC_CMD          0xAA
#define SYNC_STATE        0xBB
#define TYPE_CMD          0x01
#define TYPE_STATE_NO_IMU 0x02
#define TYPE_STATE_IMU    0x03
#define TYPE_CMD_LED      0x04

// Packet sizes
#define CMD_PACKET_SIZE         14
#define STATE_NO_IMU_PACKET_SIZE 14
#define STATE_IMU_PACKET_SIZE    30

// LED command (variable length): header(6) + count*3 + checksum(1) = 7 + count*3
// Max: 7 + 32*3 = 103 bytes
#define LED_CMD_HEADER_SIZE     6
#define LED_CMD_MAX_PACKET_SIZE 103

// Largest inbound packet (used for rx buffer sizing)
#define RX_BUF_SIZE             LED_CMD_MAX_PACKET_SIZE

// -- Command Packet (PC -> Arduino, 14 bytes) --
// [0]    sync    = 0xAA
// [1]    type    = 0x01
// [2-3]  J1 PWM  uint16 LE
// [4-5]  J2 PWM  uint16 LE
// [6-7]  J3 PWM  uint16 LE
// [8-9]  J4 PWM  uint16 LE
// [10-11] J5 PWM uint16 LE
// [12]   flags   (bit 0: LED, bit 1: disable watchdog)
// [13]   checksum (XOR of bytes 0..12)
//
// -- LED Command Packet (PC -> Arduino, 7 + count*3 bytes, variable) --
// [0]         sync       = 0xAA
// [1]         type       = 0x04
// [2]         flags      (bit 0: call show() after updating pixel buffer)
// [3]         brightness (0-255 global brightness)
// [4]         offset     (start pixel index, 0-31)
// [5]         count      (number of pixels in this packet, 0-32)
// [6..6+N*3)  count pixels × 3 bytes (R, G, B), row-major logical order
// [6+N*3]     checksum   (XOR of all preceding bytes)

struct CommandPacket {
    uint16_t pwm[5];
    uint8_t flags;
};

struct LedCommandPacket {
    uint8_t flags;
    uint8_t brightness;
    uint8_t offset;
    uint8_t count;
    const uint8_t* rgb;  // points into rx buffer, count*3 bytes
};

// -- State Packet without IMU (Arduino -> PC, 14 bytes) --
// [0]    sync    = 0xBB
// [1]    type    = 0x02
// [2-11] J1-J5 PWM uint16 LE (5 joints)
// [12]   status  (bit 0: IMU valid, bit 1: watchdog active)
// [13]   checksum (XOR of bytes 0..12)

// -- State Packet with IMU (Arduino -> PC, 30 bytes) --
// [0]    sync    = 0xBB
// [1]    type    = 0x03
// [2-5]  quat_w  float32 LE
// [6-9]  quat_x  float32 LE
// [10-13] quat_y float32 LE
// [14-17] quat_z float32 LE
// [18-27] J1-J5 PWM uint16 LE
// [28]   status
// [29]   checksum (XOR of bytes 0..28)

struct StatePacket {
    float quat[4];      // w, x, y, z (only used if IMU_ENABLED)
    uint16_t pwm[5];
    uint8_t status;
};

inline uint8_t compute_checksum(const uint8_t* buf, uint8_t len) {
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len; i++) {
        cs ^= buf[i];
    }
    return cs;
}

// Parse a command packet from the serial buffer.
// Returns true if the packet is valid.
inline bool parse_command(const uint8_t* buf, CommandPacket* cmd) {
    if (buf[0] != SYNC_CMD || buf[1] != TYPE_CMD) return false;
    if (compute_checksum(buf, 13) != buf[13]) return false;

    for (uint8_t i = 0; i < 5; i++) {
        cmd->pwm[i] = (uint16_t)buf[2 + i*2] | ((uint16_t)buf[3 + i*2] << 8);
    }
    cmd->flags = buf[12];
    return true;
}

// Parse an LED command packet from the serial buffer (variable length).
// Caller must have already verified the buffer has enough bytes.
// Returns true if the packet is valid.  rgb pointer borrows into buf.
inline bool parse_led_command(const uint8_t* buf, uint8_t total_len, LedCommandPacket* cmd) {
    if (buf[0] != SYNC_CMD || buf[1] != TYPE_CMD_LED) return false;
    if (compute_checksum(buf, total_len - 1) != buf[total_len - 1]) return false;

    cmd->flags      = buf[2];
    cmd->brightness = buf[3];
    cmd->offset     = buf[4];
    cmd->count      = buf[5];
    cmd->rgb        = &buf[6];  // count*3 bytes of RGB data
    return true;
}

// Given a type byte, return the expected full packet size.
// Returns 0 for unknown types, or LED_CMD_NEEDS_HEADER for variable-length LED.
#define LED_CMD_NEEDS_HEADER 0xFF
inline uint8_t expected_cmd_size(uint8_t type_byte) {
    switch (type_byte) {
        case TYPE_CMD:     return CMD_PACKET_SIZE;
        case TYPE_CMD_LED: return LED_CMD_NEEDS_HEADER;  // variable, need count byte first
        default:           return 0;
    }
}

// Compute LED packet size from the count byte (buf[5]).
inline uint8_t led_cmd_total_size(uint8_t count) {
    return 7 + count * 3;  // header(6) + pixels + checksum(1)
}

// Build a state packet without IMU into the output buffer.
inline void build_state_no_imu(uint8_t* buf, const uint16_t* pwm, uint8_t status) {
    buf[0] = SYNC_STATE;
    buf[1] = TYPE_STATE_NO_IMU;
    for (uint8_t i = 0; i < 5; i++) {
        buf[2 + i*2] = pwm[i] & 0xFF;
        buf[3 + i*2] = (pwm[i] >> 8) & 0xFF;
    }
    buf[12] = status;
    buf[13] = compute_checksum(buf, 13);
}

// Build a state packet with IMU into the output buffer.
inline void build_state_imu(uint8_t* buf, const float* quat, const uint16_t* pwm, uint8_t status) {
    buf[0] = SYNC_STATE;
    buf[1] = TYPE_STATE_IMU;
    // Pack quaternion as float32 LE
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t* fp = (uint8_t*)&quat[i];
        buf[2 + i*4 + 0] = fp[0];
        buf[2 + i*4 + 1] = fp[1];
        buf[2 + i*4 + 2] = fp[2];
        buf[2 + i*4 + 3] = fp[3];
    }
    for (uint8_t i = 0; i < 5; i++) {
        buf[18 + i*2] = pwm[i] & 0xFF;
        buf[19 + i*2] = (pwm[i] >> 8) & 0xFF;
    }
    buf[28] = status;
    buf[29] = compute_checksum(buf, 29);
}

#endif // ADELINO_PROTOCOL_H
