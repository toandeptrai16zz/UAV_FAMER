#ifndef DSHOT_PROTOCOL_H
#define DSHOT_PROTOCOL_H

#include <stdint.h>

/* Standard unidirectional DShot packet: 11-bit value, telemetry=0, CRC4. */
static inline uint16_t dshot_make_packet(uint16_t value)
{
    uint16_t payload;
    uint16_t checksum;

    if (value == 0u) {
        /* DShot command 0 is the motor-stop command. */
        payload = 0u;
    } else {
        if (value < 48u) value = 48u;
        if (value > 2047u) value = 2047u;
        payload = (uint16_t)((value & 0x07FFu) << 1);
    }

    checksum = (uint16_t)((payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0Fu);
    return (uint16_t)((payload << 4) | checksum);
}

#endif /* DSHOT_PROTOCOL_H */
