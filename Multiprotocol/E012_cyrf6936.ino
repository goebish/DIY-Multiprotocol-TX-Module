/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
 // compatible with E012 and E015

#if defined(E012_CYRF6936_INO)

#include "iface_cyrf6936.h"

//Protocols constants
#define E012_BIND_COUNT			1000
#define E012_INITIAL_WAIT		500
#define E012_ADDRESS_LENGTH		5

#define E012_PACKET_PERIOD		4525
#define E012_RF_BIND_CHANNEL	0x3c
#define E012_NUM_RF_CHANNELS	4
#define E012_PACKET_SIZE		15

#define E015_PACKET_PERIOD		4500	// stock Tx=9000, but let's send more packets ...
#define E015_RF_CHANNEL			0x2d	// 2445 MHz
#define E015_PACKET_SIZE		10
#define E015_BIND_PACKET_SIZE	9

//Channels
#define E012_ARM_SW			CH5_SW
#define E012_FLIP_SW		CH6_SW
#define E012_LED_SW			CH7_SW
#define E012_HEADLESS_SW	CH8_SW
#define E012_RTH_SW			CH9_SW

// E012 flags packet[1]
#define E012_FLAG_FLIP		0x40
#define E012_FLAG_HEADLESS	0x10
#define E012_FLAG_RTH		0x04
// E012 flags packet[7]
#define E012_FLAG_EXPERT	0x02

// E015 flags packet[6]
#define E015_FLAG_DISARM	0x80
#define E015_FLAG_ARM		0x40
// E015 flags packet[7]
#define E015_FLAG_FLIP		0x80
#define E015_FLAG_HEADLESS	0x10
#define E015_FLAG_RTH		0x08
#define E015_FLAG_LED		0x04
#define E015_FLAG_EXPERT	0x02
#define E015_FLAG_INTERMEDIATE 0x01

const uint8_t PROGMEM E012_init_vals[][2] = {
	{CYRF_02_TX_CTRL, 0x00},		// transmit err & complete interrupts disabled
	{CYRF_05_RX_CTRL, 0x00},		// receive err & complete interrupts disabled
	{CYRF_28_CLK_EN, 0x02},			// Force Receive Clock Enable, MUST be set
	{CYRF_32_AUTO_CAL_TIME, 0x3c},	// must be set to 3C
	{CYRF_35_AUTOCAL_OFFSET, 0x14},	// must be  set to 14
	{CYRF_06_RX_CFG, 0x48},			// LNA manual control, Rx Fast Turn Mode Enable
	{CYRF_1B_TX_OFFSET_LSB, 0x00},	// Tx frequency offset LSB
	{CYRF_1C_TX_OFFSET_MSB, 0x00},	// Tx frequency offset MSB
	{CYRF_0F_XACT_CFG, 0x24},		// Force End State, transaction end state = idle
	{CYRF_03_TX_CFG, 0x00 | 7},		// GFSK mode, PA = +4 dBm
	{CYRF_12_DATA64_THOLD, 0x0a},	// 64 Chip Data PN Code Correlator Threshold = 10
	{CYRF_0F_XACT_CFG, 0x04},		// Transaction End State = idle
	{CYRF_39_ANALOG_CTRL, 0x01},	// synth setting time for all channels is the same as for slow channels
	{CYRF_0F_XACT_CFG, 0x24},		//Force IDLE
	{CYRF_29_RX_ABORT, 0x00},		//Clear RX abort
	{CYRF_12_DATA64_THOLD, 0x0a},	//set pn correlation threshold
	{CYRF_10_FRAMING_CFG, 0x4a},	//set sop len and threshold
	{CYRF_29_RX_ABORT, 0x0f},		//Clear RX abort?
	{CYRF_03_TX_CFG, 0x00 | 4},		// GFSK mode, set power (0-7)
	{CYRF_10_FRAMING_CFG, 0x4a},	// 0b11000000 //set sop len and threshold
	{CYRF_1F_TX_OVERRIDE, 0x04},	//disable tx CRC
	{CYRF_1E_RX_OVERRIDE, 0x14},	//disable rx crc
	{CYRF_14_EOP_CTRL, 0x00},		//set EOP sync == 0
	{CYRF_01_TX_LENGTH, 70 },		// payload length // TODO: different for E012 and E015 bind/data, in emulation layer ?
};


static void __attribute__((unused)) E015_check_arming()
{
	uint8_t arm_channel = E012_ARM_SW;

	if (arm_channel != arm_channel_previous)
	{
		arm_channel_previous = arm_channel;
		if (arm_channel)
		{
			armed = 1;
			arm_flags ^= E015_FLAG_ARM;
		}
		else
		{
			armed = 0;
			arm_flags ^= E015_FLAG_DISARM;
		}
	}
}

static void __attribute__((unused)) E012_send_packet(uint8_t bind)
{
	uint8_t can_flip = 0, calibrate = 1;
	if (sub_protocol == E012)
	{
		packet_length = E012_PACKET_SIZE;
		packet[0] = rx_tx_addr[1];
		if (bind)
		{
			packet[1] = 0xaa;
			memcpy(&packet[2], hopping_frequency, E012_NUM_RF_CHANNELS);
			memcpy(&packet[6], rx_tx_addr, E012_ADDRESS_LENGTH);
			rf_ch_num = E012_RF_BIND_CHANNEL;
		}
		else
		{
			packet[1] = 0x01
				| GET_FLAG(E012_RTH_SW, E012_FLAG_RTH)
				| GET_FLAG(E012_HEADLESS_SW, E012_FLAG_HEADLESS)
				| GET_FLAG(E012_FLIP_SW, E012_FLAG_FLIP);
			packet[2] = convert_channel_16b_limit(AILERON, 0xc8, 0x00); // aileron
			packet[3] = convert_channel_16b_limit(ELEVATOR, 0x00, 0xc8); // elevator
			packet[4] = convert_channel_16b_limit(RUDDER, 0xc8, 0x00); // rudder
			packet[5] = convert_channel_16b_limit(THROTTLE, 0x00, 0xc8); // throttle
			packet[6] = 0xaa;
			packet[7] = E012_FLAG_EXPERT;	// rate (0-2)
			packet[8] = 0x00;
			packet[9] = 0x00;
			packet[10] = 0x00;
			rf_ch_num = hopping_frequency[hopping_frequency_no++];
			hopping_frequency_no %= E012_NUM_RF_CHANNELS;
		}
		packet[11] = 0x00;
		packet[12] = 0x00;
		packet[13] = 0x56;
		packet[14] = rx_tx_addr[2];
	}
	else if (sub_protocol == E015)
	{ // E015
		if (bind)
		{
			packet[0] = 0x18;
			packet[1] = 0x04;
			packet[2] = 0x06;
			// data phase address
			memcpy(&packet[3], rx_tx_addr, E012_ADDRESS_LENGTH);
			// checksum
			packet[8] = packet[3];
			for (uint8_t i = 4; i < 8; i++)
				packet[8] += packet[i];
			packet_length = E015_BIND_PACKET_SIZE;
		}
		else
		{
			E015_check_arming();
			packet[0] = convert_channel_16b_limit(THROTTLE, 0, 225); // throttle
			packet[1] = convert_channel_16b_limit(RUDDER, 225, 0); // rudder
			packet[2] = convert_channel_16b_limit(AILERON, 0, 225); // aileron
			packet[3] = convert_channel_16b_limit(ELEVATOR, 225, 0); // elevator
			packet[4] = 0x20; // elevator trim
			packet[5] = 0x20; // aileron trim
			packet[6] = arm_flags;
			packet[7] = E015_FLAG_EXPERT
				| GET_FLAG(E012_FLIP_SW, E015_FLAG_FLIP)
				| GET_FLAG(E012_LED_SW, E015_FLAG_LED)
				| GET_FLAG(E012_HEADLESS_SW, E015_FLAG_HEADLESS)
				| GET_FLAG(E012_RTH_SW, E015_FLAG_RTH);
			packet[8] = 0;
			// checksum
			packet[9] = packet[0];
			for (uint8_t i = 1; i < 9; i++)
				packet[9] += packet[i];
			packet_length = E015_PACKET_SIZE;
		}
	}
	
	HS6200_Configure(1);
	CYRF_ConfigRFChannel(rf_ch_num);
	HS6200_WritePayload(packet, packet_length);
}

static void __attribute__((unused)) E012_init()
{
	if (sub_protocol == E012)
		HS6200_SetTXAddr((uint8_t*)"\x55\x42\x9C\x8F\xC9", E012_ADDRESS_LENGTH);
	else if (sub_protocol == E015)
		HS6200_SetTXAddr((uint8_t*)"\x62\x54\x79\x38\x53", E012_ADDRESS_LENGTH);
	
	for (uint8_t i = 0; i < sizeof(E012_init_vals) / 2; i++)
		CYRF_WriteRegister(pgm_read_byte_near(&E012_init_vals[i][0]), pgm_read_byte_near(&E012_init_vals[i][1]));

	CYRF_SetTxRxMode(TX_EN);
}

uint16_t E012_callback()
{
	if (IS_BIND_IN_PROGRESS)
	{
		if (bind_counter == 0)
		{
			HS6200_SetTXAddr(rx_tx_addr, E012_ADDRESS_LENGTH);
			BIND_DONE;
		}
		else
		{
			E012_send_packet(1);
			bind_counter--;
		}
	}
	else
	{
#ifdef MULTI_SYNC
		telemetry_set_input_sync(packet_period);
#endif
		E012_send_packet(0);
	}
	return packet_period;
}

static void __attribute__((unused)) E012_initialize_txid()
{
	// rf channels
	uint32_t lfsr = random(0xfefefefe);
	for (uint8_t i = 0; i < E012_NUM_RF_CHANNELS; i++)
	{
		hopping_frequency[i] = 0x10 + ((lfsr & 0xff) % 0x32);
		lfsr >>= 8;
	}

	// test, use same channel for bind and data, known working (with SDR TX)
	hopping_frequency[0] = 0x3c;
	hopping_frequency[1] = 0x3c;
	hopping_frequency[2] = 0x3c;
	hopping_frequency[3] = 0x3c;
}

uint16_t initE012()
{
	BIND_IN_PROGRESS;
	if (sub_protocol == E012)
	{
		E012_initialize_txid();
		packet_period = E012_PACKET_PERIOD;
	}
	else if (sub_protocol == E015)
	{
		packet_period = E015_PACKET_PERIOD;
		rf_ch_num = E015_RF_CHANNEL;
		armed = 0;
		arm_flags = 0;
		arm_channel_previous = E012_ARM_SW;
	}
	E012_init();
	bind_counter = E012_BIND_COUNT;
	hopping_frequency_no = 0;
	return E012_INITIAL_WAIT;
}

// CYRF6936 HS6200 emulation layer
static uint8_t hs6200_crc;
static uint16_t hs6200_crc_init;
static uint8_t hs6200_tx_addr[5];
static uint8_t hs6200_address_length;

static const uint8_t hs6200_scramble[] = {
	0x80,0xf5,0x3b,0x0d,0x6d,0x2a,0xf9,0xbc,
	0x51,0x8e,0x4c,0xfd,0xc1,0x65,0xd0 }; // todo: find all 32 bytes ...

void HS6200_SetTXAddr(const uint8_t* addr, uint8_t len)
{
	if (len < 4)
		len = 4;
	else if (len > 5)
		len = 5;

	// preamble
	if (hs6200_tx_addr[hs6200_address_length - 1] & 0x80)
		CYRF_WritePreamble(0x555502);
	else
		CYRF_WritePreamble(0xAAAA02);

	// precompute address crc
	hs6200_crc_init = 0xffff;
	for (int i = 0; i < len; i++)
		hs6200_crc_init = crc16_update(hs6200_crc_init, addr[len - 1 - i], 8);
	memcpy(hs6200_tx_addr, addr, len);
	hs6200_address_length = len;
}

static uint16_t hs6200_calc_crc(uint8_t* msg, uint8_t len)
{
	uint8_t pos;
	uint16_t crc = hs6200_crc_init;

	// pcf + payload
	for (pos = 0; pos < len - 1; pos++)
		crc = crc16_update(crc, msg[pos], 8);
	// last byte (1 bit only)
	if (len > 0)
		crc = crc16_update(crc, msg[pos + 1], 1);
	return crc;
}

void HS6200_Configure(uint8_t enable_crc)
{
	hs6200_crc = enable_crc;
}

void HS6200_WritePayload(uint8_t* msg, uint8_t len)
{
	uint8_t payload[71];
	const uint8_t no_ack = 1; // never ask for an ack
	static uint8_t pid;
	uint8_t pos = 0;

	if (len > sizeof(hs6200_scramble))
		len = sizeof(hs6200_scramble);

	// preamble (already tried with/without/aa/55 ...)
	uint8_t preamble = 0x55;
	for(uint8_t pc=0; pc<5; pc++)
		payload[pos++] = preamble;

	// address
	for (int i = hs6200_address_length - 1; i >= 0; i--)
		payload[pos++] = hs6200_tx_addr[i];

	// guard bytes
	payload[pos++] = hs6200_tx_addr[0];
	payload[pos++] = hs6200_tx_addr[0];

	// packet control field (9 bit)
	payload[pos++] = ((len & 0x3f) << 2) | (pid & 0x03);
	payload[pos] = (no_ack & 0x01) << 7;
	pid++;

	// scrambled payload
	if (len > 0)
	{
		payload[pos++] |= (msg[0] ^ hs6200_scramble[0]) >> 1;
		for (uint8_t i = 1; i < len; i++)
			payload[pos++] = ((msg[i - 1] ^ hs6200_scramble[i - 1]) << 7) | ((msg[i] ^ hs6200_scramble[i]) >> 1);
		payload[pos] = (msg[len - 1] ^ hs6200_scramble[len - 1]) << 7;
	}

	// crc
	if (hs6200_crc)
	{
		uint16_t crc = hs6200_calc_crc(&payload[hs6200_address_length + 2], len + 2);
		uint8_t hcrc = crc >> 8;
		uint8_t lcrc = crc & 0xff;
		payload[pos++] |= (hcrc >> 1);
		payload[pos++] = (hcrc << 7) | (lcrc >> 1);
		payload[pos++] = lcrc << 7;
	}

	//CYRF wants LSB first
	for (uint8_t i = 0; i < pos; i++)
		payload[i] = E010R5_BR(payload[i]);
	
	// transmit packet (just transmit 71 bytes for now ...)
	uint8_t* buffer = payload;
	CYRF_WriteRegister(CYRF_02_TX_CTRL, 0x40);
	CYRF_WriteRegisterMulti(CYRF_20_TX_BUFFER, buffer, 16);
	CYRF_WriteRegister(CYRF_02_TX_CTRL, 0x80);
	buffer += 16;
	for (uint8_t i = 0; i < 6; i++)
	{
		while ((CYRF_ReadRegister(CYRF_04_TX_IRQ_STATUS) & 0x10) == 0);
		CYRF_WriteRegisterMulti(CYRF_20_TX_BUFFER, buffer, 8);
		buffer += 8;
	}
	while ((CYRF_ReadRegister(CYRF_04_TX_IRQ_STATUS) & 0x10) == 0);
	CYRF_WriteRegisterMulti(CYRF_20_TX_BUFFER, buffer, 6);
}
//
// End of HS6200 emulation
////////////////////////////

#endif
