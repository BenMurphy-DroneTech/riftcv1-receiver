/*
 * rift: the Rift CV1 camera beacon on the receiver's radio, beside the band
 * link, and the beacon-clock timestamps in the band acknowledgements.
 *
 * The beacon is riftcv1-mocap's firmware/rift_common (a west project, built
 * with RIFT_SHARED): beacon_tx sends a CV1 sync beacon every 2000 us, started
 * by TIMER2 (1 MHz, the beacon's clock) CC2 through PPI, and beacon_app keeps
 * its settings and commands. Between beacons the radio is ESB's.
 *
 * TIMER3 counts at 16 MHz from the same crystal, so it stays tied to TIMER2;
 * the tie (anchor_t3 at TIMER2 = anchor_local) is refreshed from every beacon,
 * whose PPI channel forks to a TIMER3 capture. TIMER3:
 *   CC0  capture of RADIO ADDRESS, only the first after each RXREADY (a PPI
 *        group), so it holds the arrival of the packet being received
 *   CC1  compare: prepare, PREP_US before the beacon's TXEN: stop ESB, save the
 *        radio registers, set up and arm the beacon
 *   CC2  capture of the beacon's TXEN (the tie)
 *   CC3  capture of now
 *   CC4  compare: restore, after the beacon: radio registers back, ESB
 *        restarted, the slot's work (LED flash) done, the next prepare set
 * The patched ESB (esb/esb.c) calls rift_ack_payload while each
 * acknowledgement ramps up; it turns CC0 into the beacon clock.
 */
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <esb.h>
#include <helpers/nrfx_gppi.h>

#include "beacon_tx.h"
#include "led_pulse.h"
#include "beacon_app.h"
#include "rift.h"
#include "rift_esb.h"

#define PREP_US      100   /* prepare this long before TXEN               */
#define MIN_LEAD_US  30    /* less than this to TXEN: skip the slot        */
#define RESTORE_US   130   /* restore this long after the slot time; the
				beacon is on air from 0 to 120 us           */
#define DONE_US      250   /* wait at most until then for it to finish     */

enum { IDLE, ON_AIR };
static volatile int state = IDLE;
static volatile bool due_prepare, due_restore;
static bool esb_was_rx;
static int slot_arm;
static uint32_t slot_ts, slot_local;

static volatile uint32_t anchor_t3, anchor_local;
static int addr_ch, rxready_ch, addr_group;

static uint32_t acks_stamped, acks_late, acks_no_capture;
static uint32_t prep_max_ticks, tie_err_max, ties;

static K_MUTEX_DEFINE(rift_mutex);
static K_SEM_DEFINE(rift_started, 0, 1);

static struct {
	uint32_t mode, pcnf0, pcnf1, base0, base1, prefix0, prefix1, txaddress,
		 rxaddresses, crccnf, crcpoly, crcinit, frequency, shorts, inten,
		 txpower, modecnf0, packetptr, tifs, datawhiteiv;
} saved;

static void radio_save(void)
{
	saved.mode = NRF_RADIO->MODE;
	saved.pcnf0 = NRF_RADIO->PCNF0;
	saved.pcnf1 = NRF_RADIO->PCNF1;
	saved.base0 = NRF_RADIO->BASE0;
	saved.base1 = NRF_RADIO->BASE1;
	saved.prefix0 = NRF_RADIO->PREFIX0;
	saved.prefix1 = NRF_RADIO->PREFIX1;
	saved.txaddress = NRF_RADIO->TXADDRESS;
	saved.rxaddresses = NRF_RADIO->RXADDRESSES;
	saved.crccnf = NRF_RADIO->CRCCNF;
	saved.crcpoly = NRF_RADIO->CRCPOLY;
	saved.crcinit = NRF_RADIO->CRCINIT;
	saved.frequency = NRF_RADIO->FREQUENCY;
	saved.shorts = NRF_RADIO->SHORTS;
	saved.inten = NRF_RADIO->INTENSET;
	saved.txpower = NRF_RADIO->TXPOWER;
	saved.modecnf0 = NRF_RADIO->MODECNF0;
	saved.packetptr = NRF_RADIO->PACKETPTR;
	saved.tifs = NRF_RADIO->TIFS;
	saved.datawhiteiv = NRF_RADIO->DATAWHITEIV;
}

static void radio_restore(void)
{
	NRF_RADIO->MODE = saved.mode;
	NRF_RADIO->PCNF0 = saved.pcnf0;
	NRF_RADIO->PCNF1 = saved.pcnf1;
	NRF_RADIO->BASE0 = saved.base0;
	NRF_RADIO->BASE1 = saved.base1;
	NRF_RADIO->PREFIX0 = saved.prefix0;
	NRF_RADIO->PREFIX1 = saved.prefix1;
	NRF_RADIO->TXADDRESS = saved.txaddress;
	NRF_RADIO->RXADDRESSES = saved.rxaddresses;
	NRF_RADIO->CRCCNF = saved.crccnf;
	NRF_RADIO->CRCPOLY = saved.crcpoly;
	NRF_RADIO->CRCINIT = saved.crcinit;
	NRF_RADIO->FREQUENCY = saved.frequency;
	NRF_RADIO->SHORTS = saved.shorts;
	NRF_RADIO->INTENCLR = 0xffffffff;
	NRF_RADIO->INTENSET = saved.inten;
	NRF_RADIO->TXPOWER = saved.txpower;
	NRF_RADIO->MODECNF0 = saved.modecnf0;
	NRF_RADIO->PACKETPTR = saved.packetptr;
	NRF_RADIO->TIFS = saved.tifs;
	NRF_RADIO->DATAWHITEIV = saved.datawhiteiv;
}

static void radio_disable(void)
{
	NRF_RADIO->SHORTS = 0;
	if (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled) {
		NRF_RADIO->EVENTS_DISABLED = 0;
		NRF_RADIO->TASKS_DISABLE = 1;
		while (NRF_RADIO->EVENTS_DISABLED == 0) {
		}
	}
}

static inline uint32_t t3_now(void)
{
	NRF_TIMER3->TASKS_CAPTURE[3] = 1;
	return NRF_TIMER3->CC[3];
}

/* TIMER3 when TIMER2 reaches local */
static inline uint32_t t3_at(uint32_t local)
{
	return anchor_t3 + 16u * (local - anchor_local);
}

/* Sets compare cc for TIMER3 time t; if t is (nearly) here, runs it now */
static void t3_compare(int cc, uint32_t t, volatile bool *due)
{
	NRF_TIMER3->EVENTS_COMPARE[cc] = 0;
	NRF_TIMER3->CC[cc] = t;
	if ((int32_t)(t - t3_now()) < 16) {
		*due = true;
		NVIC_SetPendingIRQ(TIMER3_IRQn);
	}
}

static void schedule_prepare(void)
{
	uint32_t local = beacon_tx_next() - beacon_clock_offset();

	t3_compare(1, t3_at(local - TX_LEAD_US - PREP_US), &due_prepare);
}

static void prepare(void)
{
	uint32_t t0 = t3_now();
	uint32_t local = beacon_tx_next() - beacon_clock_offset();

	if ((int32_t)(local - TX_LEAD_US - beacon_now()) < MIN_LEAD_US) {
		beacon_app_slot(beacon_tx_skip());
		schedule_prepare();
		return;
	}
	esb_was_rx = esb_rift_rx_active();
	if (esb_was_rx) {
		esb_stop_rx();
	} else {
		radio_disable();
	}
	NRF_PPI->CHENCLR = BIT(addr_ch);   /* no capture of the beacon */
	radio_save();
	NRF_RADIO->INTENCLR = 0xffffffff;
	beacon_tx_radio_setup();
	NRF_RADIO->EVENTS_END = 0;
	slot_arm = beacon_tx_arm(&slot_ts);
	slot_local = slot_ts - beacon_clock_offset();
	state = ON_AIR;

	uint32_t ticks = t3_now() - t0;

	if (ticks > prep_max_ticks) {
		prep_max_ticks = ticks;
	}
	t3_compare(4, t3_at(slot_local + RESTORE_US), &due_restore);
}

static void restore(void)
{
	if (slot_arm == BEACON_TX_ARMED) {
		while (NRF_RADIO->EVENTS_DISABLED == 0 &&
		       (int32_t)(beacon_now() - slot_local) < DONE_US) {
		}
	}
	bool sent = slot_arm == BEACON_TX_ARMED && NRF_RADIO->EVENTS_END;

	radio_disable();
	if (sent) {
		/* The beacon's TXEN, at TIMER2 = slot_local - TX_LEAD_US, was
		 * captured in CC2: the exact tie between the timers */
		uint32_t local = slot_local - TX_LEAD_US;
		uint32_t t3 = NRF_TIMER3->CC[2];
		int32_t err = (int32_t)(t3 - t3_at(local));
		uint32_t abs_err = err < 0 ? -err : err;

		if (ties > 0 && abs_err > tie_err_max) {
			tie_err_max = abs_err;
		}
		ties++;
		anchor_t3 = t3;
		anchor_local = local;
		beacon_tx_sent++;
	}
	radio_restore();
	state = IDLE;
	if (esb_was_rx) {
		esb_start_rx();
	}
	beacon_app_slot(slot_ts);
	due_prepare = false;
	schedule_prepare();
}

static void timer3_isr(const void *arg)
{
	ARG_UNUSED(arg);
	if (NRF_TIMER3->EVENTS_COMPARE[4]) {
		NRF_TIMER3->EVENTS_COMPARE[4] = 0;
		due_restore = true;
	}
	if (NRF_TIMER3->EVENTS_COMPARE[1]) {
		NRF_TIMER3->EVENTS_COMPARE[1] = 0;
		due_prepare = true;
	}
	(void)NRF_TIMER3->EVENTS_COMPARE[1];
	if (due_restore) {
		due_restore = false;
		if (state == ON_AIR) {
			restore();
		}
	}
	if (due_prepare) {
		due_prepare = false;
		if (state == IDLE) {
			prepare();
		}
	}
}

uint8_t rift_ack_payload(uint32_t pipe, uint8_t *data)
{
	if (pipe == 0) {
		return 0;   /* the pairing pipe */
	}
	if (NRF_PPI->CHEN & BIT(addr_ch)) {
		acks_no_capture++;   /* no ADDRESS since the last RXREADY */
		return 0;
	}
	if (NRF_RADIO->STATE != RADIO_STATE_STATE_TxRu) {
		acks_late++;   /* the acknowledgement is already going out */
		return 0;
	}
	uint32_t c = NRF_TIMER3->CC[0];
	unsigned int key = irq_lock();
	uint32_t a_t3 = anchor_t3, a_local = anchor_local, ofs = beacon_clock_offset();

	irq_unlock(key);

	int32_t d = (int32_t)(c - a_t3);                 /* 1/16 us after the tie */
	uint32_t us = a_local + ofs + (uint32_t)(d >> 4); /* beacon clock, floored */
	uint32_t period = beacon_app_period();
	uint32_t frame = us / period;
	uint32_t offset = (us % period) * 16 + (d & 15);

	data[0] = RIFT_ACK_TIME;
	sys_put_le32(frame, &data[1]);
	sys_put_le24(offset, &data[5]);
	sys_put_le16(period, &data[8]);
	acks_stamped++;
	return RIFT_ACK_LEN;
}

unsigned int rift_radio_lock(void)
{
	while (1) {
		unsigned int key = irq_lock();

		if (state == IDLE) {
			return key;
		}
		irq_unlock(key);
		k_busy_wait(20);
	}
}

void rift_radio_unlock(unsigned int key)
{
	irq_unlock(key);
}

unsigned int beacon_app_lock(void)
{
	return rift_radio_lock();
}

void beacon_app_unlock(unsigned int key)
{
	rift_radio_unlock(key);
}

void rift_command(const char *line)
{
	k_mutex_lock(&rift_mutex, K_FOREVER);
	beacon_app_command(line);
	k_mutex_unlock(&rift_mutex);
}

void rift_print_stats(void)
{
	k_mutex_lock(&rift_mutex, K_FOREVER);
	beacon_app_stats();
	printk("# acks: %u stamped, %u late, %u without a capture; prepare up to %u.%02u us; "
	       "timer tie within %u/16 us over %u beacons\n",
	       acks_stamped, acks_late, acks_no_capture, prep_max_ticks / 16,
	       (prep_max_ticks % 16) * 100 / 16, tie_err_max, ties);
	acks_stamped = acks_late = acks_no_capture = 0;
	k_mutex_unlock(&rift_mutex);
}

static void rift_thread(void)
{
	k_sem_take(&rift_started, K_FOREVER);
	while (1) {
		k_msleep(5);
		k_mutex_lock(&rift_mutex, K_FOREVER);
		beacon_app_poll();
		k_mutex_unlock(&rift_mutex);
	}
}
K_THREAD_DEFINE(rift_thread_id, 1024, rift_thread, NULL, NULL, NULL, 6, 0, 0);

void rift_init(void)
{
	beacon_tx_init(FRAME_PERIOD_US);
	led_pulse_init();

	NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
	NRF_TIMER3->PRESCALER = 0; /* 16 MHz */
	NRF_TIMER3->TASKS_CLEAR = 1;
	NRF_TIMER3->TASKS_START = 1;

	/* A first tie, good to a microsecond; the first beacon makes it exact */
	unsigned int key = irq_lock();

	anchor_t3 = t3_now();
	anchor_local = beacon_now();
	irq_unlock(key);

	/* The beacon's TXEN also captures TIMER3 */
	NRF_PPI->FORK[beacon_tx_ppi()].TEP = (uint32_t)&NRF_TIMER3->TASKS_CAPTURE[2];

	/* RADIO ADDRESS -> TIMER3 capture 0, then that channel switches itself
	 * off (its group's DIS task) until the next RXREADY switches it on */
	addr_ch = nrfx_gppi_channel_alloc(0);
	rxready_ch = nrfx_gppi_channel_alloc(0);
	addr_group = nrfx_gppi_group_channel_alloc(0);
	NRF_PPI->CH[addr_ch].EEP = (uint32_t)&NRF_RADIO->EVENTS_ADDRESS;
	NRF_PPI->CH[addr_ch].TEP = (uint32_t)&NRF_TIMER3->TASKS_CAPTURE[0];
	NRF_PPI->FORK[addr_ch].TEP = (uint32_t)&NRF_PPI->TASKS_CHG[addr_group].DIS;
	NRF_PPI->CHG[addr_group] = BIT(addr_ch);
	NRF_PPI->CH[rxready_ch].EEP = (uint32_t)&NRF_RADIO->EVENTS_RXREADY;
	NRF_PPI->CH[rxready_ch].TEP = (uint32_t)&NRF_PPI->TASKS_CHG[addr_group].EN;
	NRF_PPI->CHENSET = BIT(rxready_ch);

	beacon_app_init();

	IRQ_CONNECT(TIMER3_IRQn, 0, timer3_isr, NULL, 0);
	NRF_TIMER3->INTENSET = TIMER_INTENSET_COMPARE1_Msk | TIMER_INTENSET_COMPARE4_Msk;
	irq_enable(TIMER3_IRQn);
	schedule_prepare();
	k_sem_give(&rift_started);
}
