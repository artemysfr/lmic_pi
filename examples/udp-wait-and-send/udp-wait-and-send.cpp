/*******************************************************************************
 * Based on code which is:
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example waits for data on UDP port 1700 and send them through LoRa to
 * be processed by a "The Things Network" compatible server.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1, 
*  0.1% in g2). 
 *
 * Change DEVADDR to a unique address! 
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h, default is:
 *   #define CFG_sx1272_radio 1
 * for SX1272 and RFM92, but change to:
 *   #define CFG_sx1276_radio 1
 * for SX1276 and RFM95.
 *
 *******************************************************************************/

#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>		/* exit */
#include <unistd.h>
#include <pthread.h>

#include <wiringPi.h>
#include <lmic.h>
#include <hal.h>
#include <local_hal.h>

// LoRaWAN Application identifier (AppEUI)
// Not used in this example
static const u1_t APPEUI[8] =
    { 0x02, 0x00, 0x00, 0x00, 0x00, 0xEE, 0xFF, 0xC0 };

// LoRaWAN DevEUI, unique device ID (LSBF)
// Not used in this example
static const u1_t DEVEUI[8] =
    { 0x42, 0x42, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

// LoRaWAN NwkSKey, network session key 
// Use this key for The Things Network
static const u1_t DEVKEY[16] =
    { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88,
	0x09, 0xCF, 0x4F, 0x3C
};

// LoRaWAN AppSKey, application session key
// Use this key to get your data decrypted by The Things Network
static const u1_t ARTKEY[16] =
    { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88,
	0x09, 0xCF, 0x4F, 0x3C
};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
static const u4_t DEVADDR = 0xffffffff;	// <-- Change this address for every node!

enum my_sf_t { SF_7 = 7, SF_8, SF_9, SF_10, SF_11, SF_12 };

void die(const char *s)
{
	perror(s);
	exit(1);
}

void warn(const char *s)
{
	printf(s);
}

//////////////////////////////////////////////////
// APPLICATION CALLBACKS
//////////////////////////////////////////////////

// provide application router ID (8 bytes, LSBF)
void os_getArtEui(u1_t * buf)
{
	memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui(u1_t * buf)
{
	memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey(u1_t * buf)
{
	memcpy(buf, DEVKEY, 16);
}

u4_t cntr = 0;
u1_t mydata[256] = { " " };

static osjob_t sendjob;
char buffer[256] = { "NO Data !!! " };

// Pin mapping
lmic_pinmap pins = {
	.nss = 6,
	.rxtx = UNUSED_PIN,	// Not connected on RFM92/RFM95
	.rst = 0,		// Needed on RFM92/RFM95
	.dio = {7, 4, 5}
};

void onEvent(ev_t ev)
{
	//debug_event(ev);

	switch (ev) {
		// scheduled data sent (optionally data received)
		// note: this includes the receive window!
	case EV_TXCOMPLETE:
		// use this event to keep track of actual transmissions
		fprintf(stdout, "Event EV_TXCOMPLETE, time: %d\n",
			millis() / 1000);
		if (LMIC.dataLen) {	// data received in rx slot after tx
			//debug_buf(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
			fprintf(stdout, "Data Received!\n");
		}
		break;
	default:
		break;
	}
}

struct sockaddr_in si_me, si_other;
int s;
unsigned int slen = sizeof(si_other);

static void *wait_for_datagram(void *arg)
{
	while (1) {
		ssize_t count =
		    recvfrom(s, buffer, sizeof(buffer), 0,
			     (struct sockaddr *)&si_other,
			     &slen);
		if (count == -1) {
			die("recvfrom()");
		} else if (count == sizeof(buffer)) {
			warn("datagram too large for buffer: truncated");
		} else {
			printf("Received packet from %s:%d\nData: %s\n\n",
			       inet_ntoa(si_other.sin_addr),
			       ntohs(si_other.sin_port), buffer);
		}
	}

	printf("Thread ended !!!!\n");

	return NULL;
}

static void do_send(osjob_t * j)
{
	char *buf = buffer;
	time_t t = time(NULL);

	fprintf(stdout, "[%x] (%ld) %s\n", hal_ticks(), t, ctime(&t));
	// Show TX channel (channel numbers are local to LMIC)
	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & (1 << 7)) {
		fprintf(stdout, "OP_TXRXPEND, not sending");
	} else {
		// Prepare upstream data transmission at the next possible time.
		int i = 0;
		while (buf[i]) {
			mydata[i] = buf[i];
			i++;
		}
		fprintf(stdout, "buf size = %d\n", i);
		mydata[i] = '\0';
		LMIC_setTxData2(1, mydata, strlen(buf), 0);
	}
	// Schedule a timed job to run at the given timestamp (absolute system time)
	os_setTimedCallback(j, os_getTime() + sec2osticks(20), do_send);
}

void setup(_dr_eu868_t sf)
{
	// LMIC init
	wiringPiSetup();

	os_init();
	// Reset the MAC state. Session and pending data transfers will be discarded.
	LMIC_reset();
	// Set static session parameters. Instead of dynamically establishing a session 
	// by joining the network, precomputed session parameters are be provided.
	LMIC_setSession(0x1, DEVADDR, (u1_t *) DEVKEY, (u1_t *) ARTKEY);
	// Disable data rate adaptation
	LMIC_setAdrMode(0);
	// Disable link check validation
	LMIC_setLinkCheckMode(0);
	// Disable beacon tracking
	LMIC_disableTracking();
	// Stop listening for downstream data (periodical reception)
	LMIC_stopPingable();
	// Set data rate and transmit power (note: txpow seems to be ignored by the library)
	LMIC_setDrTxpow(sf, 14);
	//
}

static _dr_eu868_t check_spreading_factor(int factor)
{
	_dr_eu868_t ret = DR_SF7;

	switch (factor) {
	case SF_7:
		ret = DR_SF7;
		break;
	case SF_8:
		ret = DR_SF8;
		break;
	case SF_9:
		ret = DR_SF9;
		break;
	case SF_10:
		ret = DR_SF10;
		break;
	case SF_11:
		ret = DR_SF11;
		break;
	case SF_12:
		ret = DR_SF12;
		break;
	default:
		break;
	}

	return ret;
}

void loop()
{
	do_send(&sendjob);

	while (1) {
		os_runloop();	/* will call periodically do_send() */
	}
}

// Set default spreading factor (SF7 - SF12)
_dr_eu868_t sf = DR_SF7;

pthread_t thread_id;

int main(int argc, char *argv[])
{
	int option = -1;
	int err;

	while ((option = getopt(argc, argv, "s:")) != -1) {
		switch (option) {
		case 's':
			sf = check_spreading_factor(atoi(strdup(optarg)));
			break;
		default:
			fprintf(stderr, "Usage: %s [-s spreading factor]\n",
				argv[0]);
			exit(EXIT_FAILURE);
			break;
		}
	}
	//fprintf(stdout, "using spreading factor SF%d\n", sf);
	setup(sf);

	if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		die("socket");
	}
	memset((char *)&si_me, 0, sizeof(si_me));
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(1700);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(s, (struct sockaddr *)&si_me, sizeof(si_me)) == -1) {
		die("bind");
	}

	err = pthread_create(&thread_id, NULL, &wait_for_datagram, NULL);
	if (err != 0)
		printf("\ncan't create thread :[%s]", strerror(err));
	else
		printf("\n Thread created successfully\n");

	while (1) {
		loop();
	}

	return 0;
}
