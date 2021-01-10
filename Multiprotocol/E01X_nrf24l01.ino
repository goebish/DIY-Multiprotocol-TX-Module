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

#if defined(E01X_NRF24L01_INO)

#include "iface_nrf24l01.h"

//Protocols constants
#define E01X_BIND_COUNT			500
#define E01X_INITIAL_WAIT		500
#define E01X_ADDRESS_LENGTH		5

#define E016H_PACKET_PERIOD 4080
#define E016H_PACKET_SIZE     10
#define E016H_BIND_CHANNEL    80
#define E016H_NUM_CHANNELS    4

//Channels
#define E01X_ARM_SW			CH5_SW
#define E016H_STOP_SW		CH5_SW
#define E01X_FLIP_SW		CH6_SW
#define E01X_LED_SW			CH7_SW
#define E01X_HEADLESS_SW	CH8_SW
#define E01X_RTH_SW			CH9_SW

// E016H flags packet[1]
#define E016H_FLAG_CALIBRATE	0x80
#define E016H_FLAG_STOP		0x20
#define E016H_FLAG_FLIP		0x04
// E016H flags packet[3]
#define E016H_FLAG_HEADLESS	0x10
#define E016H_FLAG_RTH		0x04
// E016H flags packet[7]
#define E016H_FLAG_TAKEOFF	0x80
#define E016H_FLAG_HIGHRATE	0x08

static void __attribute__((unused)) E01X_send_packet(uint8_t bind)
{
    uint8_t can_flip = 0, calibrate = 1;
	
	packet_length=E016H_PACKET_SIZE;
	if(bind)
	{
		rf_ch_num=E016H_BIND_CHANNEL;
		memcpy(packet, &rx_tx_addr[1], 4);
		memcpy(&packet[4], hopping_frequency, 4);
		packet[8] = 0x23;
	}
	else
	{
		// trim commands
		packet[0] = 0;
		// aileron
		uint16_t val = convert_channel_16b_limit(AILERON, 0, 0x3ff);
		can_flip |= (val < 0x100) || (val > 0x300);
		packet[1] = val >> 8;
		packet[2] = val & 0xff;
		if(val < 0x300) calibrate = 0;
		// elevator
		val = convert_channel_16b_limit(ELEVATOR, 0x3ff, 0);
		can_flip |= (val < 0x100) || (val > 0x300);
		packet[3] = val >> 8;
		packet[4] = val & 0xff;
		if(val < 0x300) calibrate = 0;
		// throttle
		val = convert_channel_16b_limit(THROTTLE, 0, 0x3ff);
		packet[5] = val >> 8;
		packet[6] = val & 0xff;
		if(val > 0x100) calibrate = 0;
		// rudder
		val = convert_channel_16b_limit(RUDDER, 0, 0x3ff);
		packet[7] = val >> 8;
		packet[8] = val & 0xff;
		if(val > 0x100) calibrate = 0;
		// flags
		packet[1] |= GET_FLAG(E016H_STOP_SW, E016H_FLAG_STOP)
				  |  (can_flip ? GET_FLAG(E01X_FLIP_SW, E016H_FLAG_FLIP) : 0)
				  |  (calibrate ? E016H_FLAG_CALIBRATE : 0);
		packet[3] |= GET_FLAG(E01X_HEADLESS_SW, E016H_FLAG_HEADLESS)
				  |  GET_FLAG(E01X_RTH_SW, E016H_FLAG_RTH);
		packet[7] |= E016H_FLAG_HIGHRATE;
		// frequency hopping
		rf_ch_num=hopping_frequency[hopping_frequency_no++ & 0x03];
	
		// checksum
		packet[9] = packet[0];
		for (uint8_t i=1; i < E016H_PACKET_SIZE-1; i++)
			packet[9] += packet[i];
	}

	// Power on, TX mode, CRC enabled
	XN297_Configure( _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
	
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_ch_num);

	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();

	XN297_WritePayload(packet, packet_length);
	
	// Check and adjust transmission power. We do this after
	// transmission to not bother with timeout after power
	// settings change -  we have plenty of time until next
	// packet.
	NRF24L01_SetPower();
}

static void __attribute__((unused)) E01X_init()
{
	NRF24L01_Initialize();
	NRF24L01_SetTxRxMode(TX_EN);
	XN297_SetTXAddr((uint8_t *)"\x5a\x53\x46\x30\x31", 5);  // bind address
	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);
	NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
	NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1 Mbps
	NRF24L01_SetPower();
	NRF24L01_Activate(0x73);                          // Activate feature register
	NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);       // Disable dynamic payload length on all pipes
	NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x01);     // Set feature bits on
	NRF24L01_Activate(0x73);
}

uint16_t E01X_callback()
{
	if(IS_BIND_IN_PROGRESS)
	{
		if (bind_counter == 0)
		{
			XN297_SetTXAddr(rx_tx_addr, E01X_ADDRESS_LENGTH);
			BIND_DONE;
		}
		else
		{
			E01X_send_packet(1);
			bind_counter--;
		}
	}
	else
	{
		#ifdef MULTI_SYNC
			telemetry_set_input_sync(packet_period);
		#endif
		E01X_send_packet(0);
	}
	return packet_period;
}

static void __attribute__((unused)) E016H_initialize_txid()
{
	// tx id
    rx_tx_addr[0] = 0xa5;
    rx_tx_addr[1] = 0x00;

    // rf channels
    uint32_t lfsr=random(0xfefefefe);
    for(uint8_t i=0; i<E016H_NUM_CHANNELS; i++)
    {
		hopping_frequency[i] = (lfsr & 0xFF) % 80; 
		lfsr>>=8;
	}
}

uint16_t initE01X()
{
	BIND_IN_PROGRESS;
	E016H_initialize_txid();
	packet_period=E016H_PACKET_PERIOD;
	E01X_init();
	bind_counter = E01X_BIND_COUNT;
	hopping_frequency_no = 0;
	return E01X_INITIAL_WAIT;
}

#endif
