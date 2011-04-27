/*
 * Copyright (C) 2007-2010 Bjoern Biesenbach <bjoern@bjoern-b.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *
 * 14. Oct 2008
 * Based on lib of Benedikt benedikt83 at gmx.net
 * http://www.mikrocontroller.net/topic/71682
 */

#if defined(__AVR_ATmega8__)

#ifdef _RFM02
#define _RFM12_TXONLY
#endif

#define SDI		3
#define SCK		5
#define CS		2
#define SDO		4

#ifdef _RFM02
	#define IRQ 1
#endif

#define FFIT_PIN PD2
#define FFIT_DDR DDRD
#define FFIT_PORT PORTD

#define RF_PORT	PORTB
#define RF_DDR	DDRB
#define RF_PIN	PINB

#elif defined(__AVR_ATmega16__)

#define SDI		5
#define SCK		7
#define CS		4
#define SDO		6


#define FFIT_PIN PD2
#define FFIT_DDR DDRD
#define FFIT_PORT PORTD

#define RF_PORT	PORTB
#define RF_DDR	DDRB
#define RF_PIN	PINB

#elif defined(__AVR_ATmega32__)

#define SDI		5
#define SCK		7
#define CS		4
#define SDO		6

#define FFIT_PIN PD2
#define FFIT_DDR DDRD
#define FFIT_PORT PORTD

#define RF_PORT	PORTB
#define RF_DDR	DDRB
#define RF_PIN	PINB

#else
#error Not supported MCU! Use mega8, 16 or 32
#endif

// Do this in your makefile!
//#define _RFM12_TXONLY

/** initialize module
 */
extern void rf12_init(uint8_t firstinit);

/** configure module
 */
extern void rf12_config(unsigned short baudrate, 
		unsigned char channel,
		unsigned char power,
		unsigned char environment);

/** transmit a packet
 */
extern void rf12_txpacket(uint8_t *data, uint8_t count, uint8_t address, uint8_t ack_required);

#ifndef _RFM12_TXONLY
/** data in receive buffer?
 */
extern unsigned char rf12_data(void);

/** get one byte from receive buffer
 */
extern unsigned char rf12_getchar(void);
#endif

/** transmit buffer full?
 */
extern unsigned char rf12_busy(void);

/** out rfm12 into deep sleep
 */
void rf12_sleep(void);

// call this function every 1ms from timer interrupt
void rf12_every_1_ms(void);

/** macro for calculating frequency value out of frequency in MHz
 */
#ifdef _868MHZ
#warning Building 868MHz version
#define RF12FREQ(freq)	((unsigned short)((freq-860.0)/0.005))
#else
#define RF12FREQ(freq)	((unsigned short)((freq-430.0)/0.0025))
#endif

#define QUIET		1
#define NORMAL		2
#define NOISY		3

#define RECEIVED_OK		1		// Daten erfolgreich empfangen
#define RECEIVED_FAIL		2		// Daten fehlerhaft empfangen -> bitte nochmal senden
#define DATAINBUFFER		4		// Empfänger möchte Daten senden

