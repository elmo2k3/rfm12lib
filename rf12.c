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
 *
 * Based on lib of Benedikt benedikt83 at gmx.net
 * http://www.mikrocontroller.net/topic/71682
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>
#include <util/delay.h>

#include "rf12.h"
#include "main.h"

/* rx/tx buffer size */
#define MAX_BUF	140

static volatile uint8_t timeout_counter;
static unsigned char rxbuf[MAX_BUF];
static volatile unsigned char rf12_RxHead;
static volatile unsigned char rf12_RxTail;

static unsigned char tx_id __attribute__((section(".noinit"))); 

void tx_packet(unsigned char retrans);

static void rf12_rxmode(void);
static void rf12_stoprx(void);
static void rf12_setbandwidth(unsigned char bandwidth, unsigned char gain, unsigned char drssi);
static void rf12_setfreq(unsigned short freq);
static void rf12_setbaud(unsigned short baud);
static void rf12_setpower(unsigned char power, unsigned char mod);
static inline void rf12_ready(void);

typedef union conver2_ {
unsigned int w;
unsigned char b[2];
} CONVERTW;

#if !defined _RFM01 && !defined _RFM02 && !defined _RFM12 && !defined _TXONLY
#error "define _RFM01 or _RFM02 or _RFM12 or _TXONLY!"
#endif

#define LED_RX_ON() (PORTC &= ~(1<<PC0));
#define LED_RX_OFF() (PORTC |= (1<<PC0));
#define LED_TX_ON() (PORTC &= ~(1<<PC1));
#define LED_TX_OFF() (PORTC |= (1<<PC1));

#ifndef cbi
#define cbi(sfr, bit)     (_SFR_BYTE(sfr) &= ~_BV(bit)) 
#endif
#ifndef sbi
#define sbi(sfr, bit)     (_SFR_BYTE(sfr) |= _BV(bit))  
#endif

#ifdef _RFM02
#warning Building for RFM02 TX Module
#define _TXONLY
#endif

#ifdef _RFM01
#warning Building for RFM01 RX Module
#endif

#ifdef _TXONLY
#warning Building TXONLY
#endif

#if defined _RFM02 || defined _RFM01
void rf12_trans(unsigned short wert)
{	
	unsigned char i;

	cbi(RF_PORT, CS);
	for (i=0; i<16; i++)
	{
		if (wert&32768)
			sbi(RF_PORT, SDI);
		else
			cbi(RF_PORT, SDI);
		sbi(RF_PORT, SCK);
		wert<<=1;
		_delay_us(0.3);
		cbi(RF_PORT, SCK);
	}
	sbi(RF_PORT, CS);
}
#else
unsigned short rf12_trans(unsigned short wert)
{	
	CONVERTW val;
	val.w=wert;
	cbi(RF_PORT, CS);
	SPDR = val.b[1];
	while(!(SPSR & (1<<SPIF)));
	val.b[1]=SPDR;
	SPDR = val.b[0];
	while(!(SPSR & (1<<SPIF)));
	val.b[0]=SPDR;
	sbi(RF_PORT, CS);
	return val.w;
}
#endif

void rf12_init(uint8_t firstinit)
{
	RF_PORT|=(1<<CS);
#ifdef _RFM02
	RF_PORT |=(1<<IRQ);
#endif
#ifndef _RFM02
	RF_DDR&=~(1<<SDO);
#endif
	RF_DDR|=(1<<SDI)|(1<<SCK)|(1<<CS);

#ifndef _TXONLY
	FFIT_DDR &= ~(1<<FFIT_PIN);
	FFIT_PORT |= (1<<FFIT_PIN);
#endif

#ifdef _RFM12
#if F_CPU >= 10000000
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
	SPSR = (1<<SPI2X);
#else
	SPCR=(1<<SPE)|(1<<MSTR);
#endif
#endif
	
//#ifndef _TXONLY
//#ifdef _3WRGB
//	TCCR0 = (1<<CS00);
//	OCR0 =((F_CPU)/10000)-1;
//	TIMSK |= (1<<OCIE0);
//#else 
//	TCCR1A= 0;
//	TCCR1B=(1<<WGM12)| (1<<CS10);
//	OCR1A=((F_CPU)/10000)-1;
//	TIMSK|=(1<<OCIE1A);
//#endif
//#endif

	if(firstinit)
	{
		for (unsigned char i=0; i<20; i++)
			_delay_ms(10);					// wait until POR done
	}
#ifdef _RFM02
	rf12_trans(0xC0E1);			// power settings
   	_delay_ms(5);
#ifdef _868MHZ
	rf12_trans(0x9780);
#else
	rf12_trans(0x8F80);
#endif
	rf12_trans(0xC2A0);			// enable tx sync bit, disable low bat detector
#endif
#ifdef _RFM01
	rf12_trans(0xC0E1);
	rf12_trans(0x8979);
	rf12_trans(0xC4AB);
	rf12_trans(0xE000);
	rf12_trans(0xCC00);
	rf12_trans(0xC6F7);			// AFC settings: autotuning: -10kHz...+7,5kHz
#endif
#ifdef _RFM12
#ifdef _868MHZ
	rf12_trans(0x80E7);
#else
	rf12_trans(0x80D7);					// Enable FIFO
#endif
	rf12_trans(0xC2AB);					// Data Filter: internal
	rf12_trans(0xCA81);					// Set FIFO mode
	rf12_trans(0xE000);					// disable wakeuptimer
	rf12_trans(0xC800);					// disable low duty cycle
	rf12_trans(0xC4F7);					// AFC settings: autotuning: -10kHz...+7,5kHz
#endif
	rf12_RxHead=0;
	rf12_RxTail=0;

#ifndef _TXONLY
	rf12_rxmode();
#endif
}

void rf12_config(unsigned short baudrate, unsigned char channel, unsigned char power, unsigned char environment)
{
#ifdef _868MHZ
	rf12_setfreq(RF12FREQ(868.92));
#else
	rf12_setfreq(RF12FREQ(433.4)+13*channel); // Sende/Empfangsfrequenz auf 433,4MHz + channel * 325kHz einstellen
#endif
#ifndef _RFM01
   	rf12_setpower(0, 5);					// 6mW Ausgangangsleistung, 90kHz Frequenzshift
   	rf12_setbandwidth(4, environment, 0);	// 200kHz Bandbreite, Verst�rkung je nach Umgebungsbedingungen, DRSSI threshold: -103dBm (-environment*6dB)
#endif
	rf12_setbaud(baudrate);					// Baudrate
}

#ifndef _TXONLY

static void rf12_rxmode(void)
{
#ifndef _RFM01
	rf12_trans(0x82C8);					// RX on
	rf12_trans(0xCA81);					// set FIFO mode
	_delay_ms(.8);
	rf12_trans(0xCA83);					// enable FIFO: sync word search
	rf12_trans(0);
#else
	rf12_trans(0xCE84);
	_delay_us(100);
	rf12_trans(0xCE87);
#endif
	GICR|=(1<<INT0);					// Low Level Interrupt f�r FFIT (�ber Inverter angeschlossen !)
}

static void rf12_stoprx(void)
{
	GICR&=~(1<<INT0);					// Interrupt aus
#ifndef _RFM01
	rf12_trans(0x8208);					// RX off
#endif
	_delay_ms(1);
}

#endif

static void rf12_setbandwidth(unsigned char bandwidth, unsigned char gain, unsigned char drssi)
{
#ifdef _RFM02
#ifdef _868MHZ
	rf12_trans(0x9780|(bandwidth&7));
#else
	rf12_trans(0x8F80|(bandwidth&7));
#endif
#else
	rf12_trans(0x9500|((bandwidth&7)<<5)|((gain&3)<<3)|(drssi&7));
#endif
}

static void rf12_setfreq(unsigned short freq)
{	if (freq<96)						// 430,2400MHz
		freq=96;
	else if (freq>3903)					// 439,7575MHz
		freq=3903;
	rf12_trans(0xA000|freq);
}

static void rf12_setbaud(unsigned short baud)
{
#ifdef _RFM02
	if (baud<1345)
		baud=1345;
	if (baud<19000)
		rf12_trans(0xD240);		// 25% PLL current
	else if (baud<37000)
		rf12_trans(0xD2C0);		// 33% PLL current
	else
		rf12_trans(0xD200);		// 50% PLL current
	rf12_trans(0xC800|((344828UL/baud)-1));	// Baudrate= 344827,59/(R+1)
#endif
#ifdef _RFM01
	if (baud<664)
		baud=664;
	if (baud<5400)						// Baudrate= 344827,58621/(R+1)/(1+CS*7)
		rf12_trans(0xC880|((43104/baud)-1));	// R=(344828/8)/Baud-1
	else
		rf12_trans(0xC800|((344828UL/baud)-1));	// R=344828/Baud-1
#endif
#ifdef _RFM12
	if (baud<664)
		baud=664;
	if (baud<5400)						// Baudrate= 344827,58621/(R+1)/(1+CS*7)
		rf12_trans(0xC680|((43104/baud)-1));	// R=(344828/8)/Baud-1
	else
		rf12_trans(0xC600|((344828UL/baud)-1));	// R=344828/Baud-1
#endif
}

static void rf12_setpower(unsigned char power, unsigned char mod)
{	
#ifdef _RFM02
	rf12_trans(0xB000|((power&7)<<8));
#else
	rf12_trans(0x9800|(power&7)|((mod&15)<<4));
#endif
}

static inline void rf12_ready(void)
{
	cbi(RF_PORT, CS);
	asm("nop");
	asm("nop");
#ifndef _RFM02
	while (!(RF_PIN&(1<<SDO)));			// wait until FIFO ready
#endif
}

void rf12_txbyte(unsigned char val)
{
#ifdef _RFM02
	unsigned char val_temp = val;
	unsigned char j;
	for (j=0; j<8; j++)
	{
		while(RF_PIN&(1<<IRQ));
		while(!(RF_PIN&(1<<IRQ)));
		if (val&128)
			sbi(RF_PORT, SDI);
		else
			cbi(RF_PORT, SDI);
		val<<=1;
	}
	if(val_temp == 0x00 || val_temp == 0xFF)
	{
		val = 0xAA;
		for (j=0; j<8; j++)
		{
			while(RF_PIN&(1<<IRQ));
			while(!(RF_PIN&(1<<IRQ)));
			if (val&128)
				sbi(RF_PORT, SDI);
			else
				cbi(RF_PORT, SDI);
			val<<=1;
		}
	}
#endif
#ifdef _RFM12
	rf12_ready();
	rf12_trans(0xB800|val);
	if ((val==0x00)||(val==0xFF))		// Stuffbyte einf�gen um ausreichend Pegelwechsel zu haben
	{	rf12_ready();
		rf12_trans(0xB8AA);
	}
#endif
}

#ifndef _TXONLY

unsigned char rf12_rxbyte(void)
{	
	unsigned char val;
	unsigned char i;
#ifdef _RFM01
	cbi(RF_PORT,SDI);
	rf12_ready();
	for(i=0;i<16;i++)
	{
		sbi(RF_PORT, SCK);
		_delay_us(0.1);
		cbi(RF_PORT, SCK);
	}
	val = 0;
	for(i=0;i<8;i++)
	{
		val <<=1;
		if (RF_PIN&(1<<SDO))
			val |= 1;
		sbi(RF_PORT, SCK);
		_delay_us(0.2);
		cbi(RF_PORT, SCK);
	}
	sbi(RF_PORT, CS);
	sbi(RF_PORT,SDI);
	_delay_us(1);
	cbi(RF_PORT,SDI);

	if((val == 0x00) || (val == 0xFF)) // weg mit stuffbyte
	{
		_delay_ms(0.5);
		cbi(RF_PORT, CS);
		//rf12_ready();
		for(i=0;i<16;i++)
		{
			sbi(RF_PORT, SCK);
			_delay_us(0.1);
			cbi(RF_PORT, SCK);
		}
		for(i=0;i<8;i++)
		{
			sbi(RF_PORT, SCK);
			_delay_us(0.2);
			cbi(RF_PORT, SCK);
		}
	}
	sbi(RF_PORT, CS);
	sbi(RF_PORT,SDI);

#else
	val =rf12_trans(0xB000);
	if ((val==0x00)||(val==0xFF))		// Stuffbyte wieder entfernen
	{	rf12_ready();
		rf12_trans(0xB000);
	}
#endif
	return val;
}

#endif

//void rf12_txdata(unsigned char *data, unsigned char number, unsigned char status, unsigned char id, unsigned char toAddress)
static void rf12_txdata(uint8_t type, uint8_t destination, uint8_t *data, uint8_t length, uint8_t id)
{	
	unsigned char i, crc;
#ifdef _BASE_USB
	LED_TX_ON();
#endif
	//LED_TX=1;
	
	
#ifdef _RFM02
	unsigned char wert;
	wert=0xC6;
	cbi(RF_PORT, CS);
	for (i=0; i<8; i++)
	{
		if (wert&128)
		sbi(RF_PORT, SDI);
		else
			cbi(RF_PORT, SDI);
		sbi(RF_PORT, SCK);
		wert<<=1;
		_delay_us(0.2);
		cbi(RF_PORT, SCK);
	}
	rf12_txbyte(0xAA);
	rf12_txbyte(0xAA);
	rf12_txbyte(0xAA);
	rf12_txbyte(0x2D);
	rf12_txbyte(0xD4);
#endif
#ifdef _RFM12
	rf12_trans(0x8238);					// TX on
	rf12_ready();
	rf12_trans(0xB8AA);					// Sync Data
	rf12_ready();
	rf12_trans(0xB8AA);
	rf12_ready();
	rf12_trans(0xB8AA);
	rf12_ready();
	rf12_trans(0xB82D);
	rf12_ready();
	rf12_trans(0xB8D4);
#endif
	/* New protocol starts here 
	 *
	 * | Type | Length | Source | Destination | Id | Data | CRC |
	 */

	switch(type)
	{
		/* Only data
		 * | Type | Length |
		 */
		case 0x00:
			break;

		/* Ping
		 */
		case 0x10:
			break;

		/* Pong
		 */
		case 0x11:
			break;

		/* Ack
		 */
		case 0x12:
			break;

		/* Retransmission request
		 */
		case 0x13:
			break;

		/* Data + CRC
		 */
		case 0x18:
			break;

		/* Data + CRC + Id
		 */
		case 0x19:
			crc = _crc_ibutton_update(0,type);
			rf12_txbyte(type);
			crc = _crc_ibutton_update(crc, length);
			rf12_txbyte(length);
			crc = _crc_ibutton_update(crc, destination);
			rf12_txbyte(destination);
			crc = _crc_ibutton_update(crc, MY_ADDRESS);
			rf12_txbyte(MY_ADDRESS);
			crc = _crc_ibutton_update(crc, id);
			rf12_txbyte(id);
			for(i=0;i<length;i++)
			{
				crc = _crc_ibutton_update(crc, *data);
				rf12_txbyte(*data++);
			}
			rf12_txbyte(crc); // append crc
			rf12_txbyte(0); // dummy data
#ifdef _RFM12
			rf12_trans(0x8208); // tx off
#endif
			break;
			

		/* Data + CRC + Id + Ack request
		 */
		case 0x1A:
			break;
	}
#ifdef _RMF02
	sbi(RF_PORT, CS);
	while(RF_PIN&(1<<IRQ));		// wait until transfer done
	rf12_trans(0xC464);			// TX off after 10us
#endif
#ifdef _BASE_USB
	LED_TX_OFF();
#endif

}

#ifndef _TXONLY

ISR(SIG_INTERRUPT0)
{	
	static unsigned char bytecnt=0, number, id, crc,rf_data[MAX_BUF];
	static unsigned char rx_lastid=255, toAddress, fromAddress, type;
	static uint16_t stupidcounter = 0;
	cli();

#ifdef _BASE_USB
	LED_RX_ON();
#endif

#ifdef _RFM01
	if(timeout_counter > 5)
	{
#else
	if(timeout_counter > 7)
	{
#endif
#ifdef _BASE_USB
		LED_RX_OFF();
#endif
		bytecnt = 0;
		stupidcounter = 0;
		timeout_counter = 0;
	}

	if (bytecnt==0)
	{
		type = rf12_rxbyte(); // type of packet
		bytecnt=1;
		timeout_counter = 0;
	}
	else if(type == 0x19)
	{
		if (bytecnt==1)
		{	
			crc =_crc_ibutton_update (0, type);
			number = rf12_rxbyte();	 // number of bytes of data
			crc=_crc_ibutton_update (crc, number);
			bytecnt=2;
			timeout_counter = 0;
		}
		else if (bytecnt==2)
		{	
			toAddress = rf12_rxbyte();			// destination
			crc=_crc_ibutton_update (crc, toAddress);
			bytecnt=3;
			timeout_counter = 0;
		}
		else if (bytecnt==3)
		{	
			fromAddress = rf12_rxbyte();			// source
			crc=_crc_ibutton_update (crc, fromAddress);
			bytecnt = 4;
			timeout_counter = 0;
		}
		else if (bytecnt==4)
		{	id = rf12_rxbyte();				// packet id
			crc = _crc_ibutton_update (crc, id);
			bytecnt=5;
			timeout_counter = 0;
		}
		else if (bytecnt<255)
		{	rf_data[bytecnt-5]=rf12_rxbyte();
			crc=_crc_ibutton_update (crc, rf_data[bytecnt-5]);
			bytecnt++;
			timeout_counter = 0;
			if ((bytecnt-5)>=number)		// all bytes received?
				bytecnt=255;
		}
		else
		{	
			timeout_counter = 0;
			unsigned char crcref;
			crcref = rf12_rxbyte();			// CRC received
#ifdef _RFM12
			rf12_trans(0xCA81);			// restart syncword detection:
			rf12_trans(0xCA83);			// enable FIFO
#endif
#ifdef _RFM01
			rf12_trans(0xCE84);
			_delay_us(1);
			rf12_trans(0xCE87);
#endif

			if (crcref == crc && toAddress == MY_ADDRESS) // CRC ok and packet for me
			{
				if (id!=rx_lastid) // new data?
				{	
					unsigned char i, tmphead;

					rx_lastid = id;	// save current id
					tmphead = rf12_RxHead;
					for(i=0; i<number; i++)	// Komplettes Paket in den Empfangspuffer kopieren
					{	
						tmphead++;
						if (tmphead>=MAX_BUF)
							tmphead=0;
						if (tmphead == rf12_RxTail)	// receive buffer overflow !!!
							break;				// restlichen Daten ignorieren
						else
							rxbuf[tmphead] = rf_data[i]; // Daten in Empfangspuffer kopieren
					}
					rf12_RxHead = tmphead;
				}
			}
			bytecnt=0;
			stupidcounter = 0;
#ifdef _BASE_USB
		LED_RX_OFF();
#endif
		}
	} // end type 0x19
	else
	{
		rf12_rxbyte();
#ifdef _RFM01
		rf12_trans(0xCE84);
		_delay_us(100);
		rf12_trans(0xCE87);
		_delay_us(100);
#endif
		bytecnt = 0;
	}

	if(++stupidcounter > 256)
	{
		bytecnt = 0;
		stupidcounter = 0;
#ifdef _RFM01
		rf12_trans(0xCE84);
		_delay_us(100);
		rf12_trans(0xCE87);
		_delay_us(100);
#endif
	}
	sei();
}

#endif

/** alle 100us
 */
#ifndef _TXONLY

/* hack for a special device where i need
 * timer1 for pwm */
//#ifdef _3WRGB
//ISR(SIG_OUTPUT_COMPARE0)
//#else
//ISR(SIG_OUTPUT_COMPARE1A)
//#endif
//{	
//	if(timeout_counter++ > 254)
//		timeout_counter = 255;
//}

void rf12_every_1_ms()
{
	if(timeout_counter++ > 254)
		timeout_counter = 255;
}

#endif

#ifndef _TXONLY

unsigned char rf12_data(void)
{
	if(rf12_RxHead == rf12_RxTail)
		return 0;
	else
		return 1;
}

unsigned char rf12_getchar(void)
{    
	unsigned short tmptail;
	/* block until there is data to return */
	while (rf12_RxHead == rf12_RxTail); 
	tmptail = (rf12_RxTail + 1);     // calculate buffer index
	if (tmptail>=MAX_BUF)
		tmptail=0;
	rf12_RxTail = tmptail; 
	return rxbuf[tmptail];		// get data from receive buffer
}

#endif

void rf12_txpacket(uint8_t *data, uint8_t count, uint8_t address, uint8_t ack_required)
{
	if(!ack_required)
	{
#ifndef _TXONLY
		rf12_stoprx();
#endif
		rf12_txdata(0x19, address, data, count, tx_id++);
#ifndef _TXONLY
		rf12_rxmode();
#endif
	}
	else
	{
		/* ack is required */
	}
}

void rf12_sleep(void)
{
#ifdef _RFM02
	rf12_trans(0xC400); //sleep command!	
	_delay_ms(2);
	rf12_trans(0xC001);			// power settings
#endif
	rf12_trans(0x8201);
}

