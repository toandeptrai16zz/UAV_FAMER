#ifndef DSHOT_PROTOCOL_H
#define DSHOT_PROTOCOL_H

#include <stdint.h>

/*
 * Build a standard unidirectional DShot packet.
 * value must be 0 (stop) or a throttle value in the range 48..2047.
 * Bit 0 of the 12-bit payload is the telemetry request; this implementation
 * sends no telemetry request. The 4-bit CRC is not complemented.
 */
static inline uint16_t dshot_make_packet(uint16_t value)
{
    uint16_t payload = (uint16_t)((value & 0x07FFu) << 1);
    uint16_t checksum = (uint16_t)((payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0Fu);
    return (uint16_t)((payload << 4) | checksum);
}

#endif /* DSHOT_PROTOCOL_H */
