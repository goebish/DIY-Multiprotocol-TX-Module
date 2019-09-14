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

#if defined(SCANNER_CC2500_INO)

#include "iface_cc2500.h"

#define SCAN_MAX_RADIOCHANNEL    249 // 2483 MHz
#define SCAN_CHANNEL_LOCK_TIME   300 // with precalibration, channel requires only 90 usec for synthesizer to settle
#define SCAN_CHANS_PER_PACKET	 5
#define AVERAGE_INTVL       30

static uint8_t scan_tlm_index;

enum ScanStates {
	SCAN_CHANNEL_CHANGE = 0,
	SCAN_GET_RSSI = 1,
};

static void __attribute__((unused)) Scanner_cc2500_init()
{
	/* Initialize CC2500 chip */
	CC2500_WriteReg(CC2500_08_PKTCTRL0, 0x12);   // Packet Automation Control
    CC2500_WriteReg(CC2500_0B_FSCTRL1,  0x0A);   // Frequency Synthesizer Control
    CC2500_WriteReg(CC2500_0C_FSCTRL0,  0x00);   // Frequency Synthesizer Control
    CC2500_WriteReg(CC2500_0D_FREQ2,    0x5C);   // Frequency Control Word, High Byte
    CC2500_WriteReg(CC2500_0E_FREQ1,    0x4E);   // Frequency Control Word, Middle Byte
    CC2500_WriteReg(CC2500_0F_FREQ0,    0xC3);   // Frequency Control Word, Low Byte
    CC2500_WriteReg(CC2500_10_MDMCFG4,  0x8D);   // Modem Configuration
    CC2500_WriteReg(CC2500_11_MDMCFG3,  0x3B);   // Modem Configuration
    CC2500_WriteReg(CC2500_12_MDMCFG2,  0x10);   // Modem Configuration
    CC2500_WriteReg(CC2500_13_MDMCFG1,  0x23);   // Modem Configuration
    CC2500_WriteReg(CC2500_14_MDMCFG0,  0xA4);   // Modem Configuration
    CC2500_WriteReg(CC2500_15_DEVIATN,  0x62);   // Modem Deviation Setting
    CC2500_WriteReg(CC2500_18_MCSM0,    0x08);   // Main Radio Control State Machine Configuration
    CC2500_WriteReg(CC2500_19_FOCCFG,   0x1D);   // Frequency Offset Compensation Configuration
    CC2500_WriteReg(CC2500_1A_BSCFG,    0x1C);   // Bit Synchronization Configuration
    CC2500_WriteReg(CC2500_1B_AGCCTRL2, 0xC7);   // AGC Control
    CC2500_WriteReg(CC2500_1C_AGCCTRL1, 0x00);   // AGC Control
    CC2500_WriteReg(CC2500_1D_AGCCTRL0, 0xB0);   // AGC Control
    CC2500_WriteReg(CC2500_21_FREND1,   0xB6);   // Front End RX Configuration

	CC2500_SetTxRxMode(RX_EN);  // Receive mode
	CC2500_Strobe(CC2500_SIDLE);
	CC2500_Strobe(CC2500_SRX);

	delayMicroseconds(1000);  // wait for RX to activate
}

static void __attribute__((unused)) Scanner_calibrate()
{
	for (int c = 0; c < SCAN_MAX_RADIOCHANNEL; c++) {
		CC2500_Strobe(CC2500_SIDLE);
		CC2500_WriteReg(CC2500_0A_CHANNR, c);
		CC2500_Strobe(CC2500_SCAL);
		delayMicroseconds(900);
		calData[c] = CC2500_ReadReg(CC2500_25_FSCAL1);
	}
	CC2500_Strobe(CC2500_SIDLE);
}

static void __attribute__((unused)) Scanner_scan_next()
{
	CC2500_WriteReg(CC2500_0A_CHANNR, rf_ch_num);
	CC2500_WriteReg(CC2500_25_FSCAL1, calData[rf_ch_num]);
	CC2500_Strobe(CC2500_SFRX);
	CC2500_Strobe(CC2500_SRX);
}

static int __attribute__((unused)) Scanner_scan_rssi()
{
	uint8_t rssi;
	rssi = CC2500_ReadReg(0x40 | CC2500_34_RSSI);  // 0.5 db/count, RSSI value read from the RSSI status register is a 2�s complement number
	uint8_t rssi_rel;
	if (rssi >= 128) {
		rssi_rel = rssi - 128;  // relative power levels 0-127 (equals -137 to -72 dBm)
	}
	else {
		rssi_rel = rssi + 128;  // relativ power levels 128-255 (equals -73 to -10 dBm)
	}
	return rssi_rel;
}

uint16 Scanner_callback()
{
	int rssi_value;
	if (telemetry_link == 1) // telemetry packet not sent yet
		return SCAN_CHANNEL_LOCK_TIME;
	switch (phase) {
	case SCAN_CHANNEL_CHANGE:
		rf_ch_num++;
		if (rf_ch_num >= (SCAN_MAX_RADIOCHANNEL + 1))
			rf_ch_num = 0;
		if (scan_tlm_index++ == 0)
			pkt[0] = rf_ch_num;  // start channel for telemetry packet
		Scanner_scan_next();
		phase = SCAN_GET_RSSI;
		return SCAN_CHANNEL_LOCK_TIME;
	case SCAN_GET_RSSI:
		phase = SCAN_CHANNEL_CHANGE;
		pkt[scan_tlm_index] = Scanner_scan_rssi();
		if (scan_tlm_index == SCAN_CHANS_PER_PACKET) {
			// send data to TX
			telemetry_link = 1;
			scan_tlm_index = 0;
		}
	}
	return AVERAGE_INTVL;
}

uint16_t initScanner(void)
{
	rf_ch_num = SCAN_MAX_RADIOCHANNEL;
	scan_tlm_index = 0;
	telemetry_link = 0;
	phase = SCAN_CHANNEL_CHANGE;
	CC2500_Reset();
	Scanner_cc2500_init();
	CC2500_Strobe(CC2500_SRX);
	Scanner_calibrate();
	CC2500_Strobe(CC2500_SIDLE);
	CC2500_SetTxRxMode(RX_EN);
	CC2500_Strobe(CC2500_SRX);  // Receive mode
	BIND_DONE;
	return 1250;
}

#endif
