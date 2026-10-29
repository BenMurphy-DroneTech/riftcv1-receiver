/*
 * rift_imu_rx: the band's frame packets (riftcv1-band src/connection/rift_imu.h:
 * two half-frame IMU slices and the orientation per camera frame), printed on
 * the console as "IMU <hex>" for host/rift_imu.py in riftcv1-mocap to decode.
 * The ESB handler queues them; a thread of their own prints them, so a burst
 * never holds up the radio.
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "rift.h"

#define QUEUE 32

K_MSGQ_DEFINE(rift_imu_q, RIFT_IMU_LEN, QUEUE, 4);
static uint32_t n_received, n_no_room;

void rift_imu_rx(const uint8_t *data)
{
	n_received++;
	if (k_msgq_put(&rift_imu_q, data, K_NO_WAIT)) {
		n_no_room++;
	}
}

void rift_imu_print_stats(void)
{
	printk("# imu: %u frame packets, %u dropped for want of room\n", n_received, n_no_room);
}

static void rift_imu_thread(void)
{
	static const char hex[] = "0123456789abcdef";
	uint8_t p[RIFT_IMU_LEN];
	char line[2 * RIFT_IMU_LEN + 1];

	while (1) {
		k_msgq_get(&rift_imu_q, p, K_FOREVER);
		for (int i = 0; i < RIFT_IMU_LEN; i++) {
			line[2 * i] = hex[p[i] >> 4];
			line[2 * i + 1] = hex[p[i] & 15];
		}
		line[2 * RIFT_IMU_LEN] = 0;
		printk("IMU %s\n", line);
	}
}

K_THREAD_DEFINE(rift_imu_thread_id, 1024, rift_imu_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(10), 0, 0);
