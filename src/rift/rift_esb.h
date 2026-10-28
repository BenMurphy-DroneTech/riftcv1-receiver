/*
 * The two links between the patched ESB (esb/esb.c) and rift.c.
 */
#ifndef RIFT_ESB_H
#define RIFT_ESB_H

#include <stdbool.h>
#include <stdint.h>

/* rift.c: called from ESB's radio interrupt as each acknowledgement is built,
 * when no payload is queued for its pipe. Writes the acknowledgement payload
 * into data and returns its length (0 for none). */
uint8_t rift_ack_payload(uint32_t pipe, uint8_t *data);

/* esb.c: true while ESB's PRX listens or sends an acknowledgement. */
bool esb_rift_rx_active(void);

#endif
