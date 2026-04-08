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

// Packet sizes
#define CMD_PACKET_SIZE         14
#define STATE_NO_IMU_PACKET_SIZE 14
#define STATE_IMU_PACKET_SIZE    30

// -- Command Packet (PC -> Arduino, 14 bytes) --
// [0]    sync    = 0xAA
// [1]    type    = 0x01
// [2-3]  J1 PWM  uint16 LE
// [4-5]  J2 PWM  uint16 LE
// [6-7]  J3 PWM  uint16 LE
// [8-9]  J4 PWM  uint16 LE
// [10-11] J5 PWM uint16 LE
// [12]   flags   (bit 0: LED)
// [13]   checksum (XOR of bytes 0..12)

struct CommandPacket {
    uint16_t pwm[5];
    uint8_t flags;
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
