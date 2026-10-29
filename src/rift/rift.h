/*
 * rift: the Rift CV1 camera beacon on the receiver's radio, beside the band
 * link, and the beacon-clock timestamps in the band acknowledgements.
 */
#ifndef RIFT_H
#define RIFT_H

#include <stdbool.h>
#include <stdint.h>

/* Acknowledgement payload (receiver to band), 10 bytes:
 *   [0]     RIFT_ACK_TIME
 *   [1..4]  frame number on the beacon clock, uint32 LE (beacon clock / period)
 *   [5..7]  offset into that frame in 1/16 us, uint24 LE
 *   [8..9]  frame period in us, uint16 LE
 * The time is the packet's RADIO ADDRESS event at the receiver. */
#define RIFT_ACK_TIME 0xe0
#define RIFT_ACK_LEN  10

/* The band's frame packet (riftcv1-band src/connection/rift_imu.h has the layout),
 * 36 bytes: RIFT_IMU_PACKET, tracker id, then two IMU slices and the orientation */
#define RIFT_IMU_PACKET 0xe1
#define RIFT_IMU_LEN    36

/* From the ESB handler: a frame packet, queued to be printed as IMU <hex>. */
void rift_imu_rx(const uint8_t *data);
void rift_imu_print_stats(void);

/* Starts the beacon; call before ESB starts. */
void rift_init(void);

/* Runs one console line as a beacon command (beacon_app.h lists them);
 * console lines that aren't the receiver's own commands go here. */
void rift_command(const char *line);

/* The beacon line and the acknowledgement timestamp counts, for stats. */
void rift_print_stats(void);

/* Every ESB call that touches the radio goes between these, so it never runs
 * while the beacon has the radio. */
unsigned int rift_radio_lock(void);
void rift_radio_unlock(unsigned int key);

#endif
