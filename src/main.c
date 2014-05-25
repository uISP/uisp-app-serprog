/*  
 *   Copyright (C) 2014  Andrew 'Necromant' Andrianov <andrew@ncrmnt.org>
 *   Copyright (C) 2012  Denis 'GNUtoo' Carikli <GNUtoo@no-log.org>
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <arch/antares.h>
#include <avr/boot.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <generated/usbconfig.h>
#include <arch/vusb/usbportability.h>
#include <arch/vusb/usbdrv.h>
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <lib/circ_buf.h>



enum {
	SEND_ENCAPSULATED_COMMAND = 0,
	GET_ENCAPSULATED_RESPONSE,
	SET_COMM_FEATURE,
	GET_COMM_FEATURE,
	CLEAR_COMM_FEATURE,
	SET_LINE_CODING = 0x20,
	GET_LINE_CODING,
	SET_CONTROL_LINE_STATE,
	SEND_BREAK
};

#define HW_CDC_BULK_OUT_SIZE 8
#define HW_CDC_BULK_IN_SIZE 8


typedef union usbDWord {
	uint32_t	dword;
    uchar bytes[4];
} usbDWord_t;

static const PROGMEM char configDescrCDC[] = { /* USB configuration descriptor */
	9, /* sizeof(usbDescrConfig): length of descriptor in bytes */
	USBDESCR_CONFIG, /* descriptor type */
	67,
	0, /* total length of data returned (including inlined descriptors) */
	2, /* number of interfaces in this configuration */
	1, /* index of this configuration */
	0, /* configuration name string index */
#if USB_CFG_IS_SELF_POWERED
	(1 << 7) | USBATTR_SELFPOWER, /* attributes */
#else
	(1 << 7), /* attributes */
#endif
	USB_CFG_MAX_BUS_POWER/2, /* max USB current in 2mA units */

/* interface descriptor follows inline: */
	9, /* sizeof(usbDescrInterface): length of descriptor in bytes */
	USBDESCR_INTERFACE, /* descriptor type */
	0, /* index of this interface */
	0, /* alternate setting for this interface */
	USB_CFG_HAVE_INTRIN_ENDPOINT, /* endpoints excl 0: number of endpoint descriptors to follow */
	USB_CFG_INTERFACE_CLASS,
	USB_CFG_INTERFACE_SUBCLASS,
	USB_CFG_INTERFACE_PROTOCOL,
	0, /* string index for interface */

/* CDC Class-Specific descriptor */
	5, /* sizeof(usbDescrCDC_HeaderFn): length of descriptor in bytes */
	0x24, /* descriptor type */
	0, /* header functional descriptor */
	0x10, 0x01,

	4, /* sizeof(usbDescrCDC_AcmFn): length of descriptor in bytes */
	0x24, /* descriptor type */
	2, /* abstract control management functional descriptor */
	0x02, /* SET_LINE_CODING, GET_LINE_CODING, SET_CONTROL_LINE_STATE */

	5, /* sizeof(usbDescrCDC_UnionFn): length of descriptor in bytes */
	0x24, /* descriptor type */
	6, /* union functional descriptor */
	0, /* CDC_COMM_INTF_ID */
	1, /* CDC_DATA_INTF_ID */

	5, /* sizeof(usbDescrCDC_CallMgtFn): length of descriptor in bytes */
	0x24, /* descriptor type */
	1, /* call management functional descriptor */
	3, /* allow management on data interface, handles call management by itself */
	1, /* CDC_DATA_INTF_ID */

/* Endpoint Descriptor */
	7, /* sizeof(usbDescrEndpoint) */
	USBDESCR_ENDPOINT, /* descriptor type = endpoint */
	0x80 | 3, /* IN endpoint number */
	0x03, /* attrib: Interrupt endpoint */
	8, 0, /* maximum packet size */
	USB_CFG_INTR_POLL_INTERVAL, /* in ms */

/* Interface Descriptor */
	9, /* sizeof(usbDescrInterface): length of descriptor in bytes */
	USBDESCR_INTERFACE, /* descriptor type */
	1, /* index of this interface */
	0, /* alternate setting for this interface */
	2, /* endpoints excl 0: number of endpoint descriptors to follow */
	0x0A, /* Data Interface Class Codes */
	0,
	0, /* Data Interface Class Protocol Codes */
	0, /* string index for interface */

/* Endpoint Descriptor */
	7, /* sizeof(usbDescrEndpoint) */
	USBDESCR_ENDPOINT, /* descriptor type = endpoint */
	0x81, /* OUT endpoint number 1 */
	0x02, /* attrib: Bulk endpoint */
	8, 0, /* maximum packet size */
	8, /* in ms */

/* Endpoint Descriptor */
	7, /* sizeof(usbDescrEndpoint) */
	USBDESCR_ENDPOINT, /* descriptor type = endpoint */
	0x01, /* IN endpoint number 1 */
	0x02, /* attrib: Bulk endpoint */
	8, 0, /* maximum packet size */
	8, /* in ms */
};


uchar usbFunctionDescriptor(usbRequest_t *rq)
{

	if(rq->wValue.bytes[1] == USBDESCR_DEVICE){
		usbMsgPtr = (uchar *)usbDescriptorDevice;
		return usbDescriptorDevice[0];
	}else{ /* must be config descriptor */
		usbMsgPtr = (uchar *)configDescrCDC;
		return sizeof(configDescrCDC);
	}
}

static uchar stopbit, parity, databit;
static usbDWord_t baud;

uchar sendEmptyFrame;
static uchar intr3Status; /* used to control interrupt endpoint transmissions */



uchar usbFunctionSetup(uchar data[8])
{
	usbRequest_t *rq = (void *)data;
	if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){ /* class request type */

		if( rq->bRequest==GET_LINE_CODING || rq->bRequest==SET_LINE_CODING ){
			return 0xff;
/* GET_LINE_CODING -> usbFunctionRead() */
/* SET_LINE_CODING -> usbFunctionWrite() */
		}
		if(rq->bRequest == SET_CONTROL_LINE_STATE){

#if USB_CFG_HAVE_INTRIN_ENDPOINT3
/* Report serial state (carrier detect). On several Unix platforms,
 * tty devices can only be opened when carrier detect is set.
 */
			if( intr3Status==0 )
				intr3Status = 2;
#endif
		}
#if 1
/* Prepare bulk-in endpoint to respond to early termination */
		if((rq->bmRequestType & USBRQ_DIR_MASK) == USBRQ_DIR_HOST_TO_DEVICE)
			sendEmptyFrame = 1;
#endif
	}

	return 0;
}


uchar usbFunctionRead( uchar *data, uchar len )
{
	data[0] = baud.bytes[0];
	data[1] = baud.bytes[1];
	data[2] = baud.bytes[2];
	data[3] = baud.bytes[3];
	data[4] = stopbit;
	data[5] = parity;
	data[6] = databit;

	return 7;
}

uchar usbFunctionWrite( uchar *data, uchar len )
{
/* SET_LINE_CODING */
	baud.bytes[0] = data[0];
	baud.bytes[1] = data[1];
	baud.bytes[2] = data[2];
	baud.bytes[3] = data[3];

	stopbit = data[4];
	parity = data[5];
	databit = data[6];

	if( parity>2 )
		parity = 0;
	if( stopbit==1 )
		stopbit = 0;

	return 1;
}


#define OB_SIZE 256

#if OB_SIZE % 2 
#error "CB must be power of two"
#endif


static char out_buffer[OB_SIZE];
static int o_head, o_tail;
static char in_buffer[OB_SIZE];
static int i_head, i_tail;

void usbDoLoop()
{
	usbPoll();

	if (usbAllRequestsAreDisabled() && 
	    (CIRC_SPACE(i_head, i_tail, OB_SIZE) > HW_CDC_BULK_OUT_SIZE))
		usbEnableAllRequests();

	if( usbInterruptIsReady() && 
	    (CIRC_CNT(o_head, o_tail, OB_SIZE) || sendEmptyFrame) ) {
		int len = min_t(int, 
				HW_CDC_BULK_OUT_SIZE, 
				CIRC_CNT_TO_END(o_head, o_tail, OB_SIZE));
		/* Actual data transfer here */ 
		usbSetInterrupt(&out_buffer[o_tail], len);
		o_tail = (o_tail + len) & (OB_SIZE - 1);
	}

#if 0
/* We need to report rx and tx carrier after open attempt */
	if(intr3Status != 0 && usbInterruptIsReady3()){
		static uchar serialStateNotification[10] = {0xa1, 0x20, 0, 0, 0, 0, 2, 0, 3, 0};
		
		if(intr3Status == 2){
			usbSetInterrupt3(serialStateNotification, 8);
		}else{
			usbSetInterrupt3(serialStateNotification+8, 2);
		}
		intr3Status--;
	}


#endif	
}


void usb_putc(unsigned char data )
{
	while (!CIRC_SPACE(o_head, o_tail, OB_SIZE) )
		usbDoLoop();
	out_buffer[o_head] = data; 
	o_head = (o_head + 1) & (OB_SIZE -1);
}

unsigned char usb_getc(void)
{
	while (!CIRC_CNT(i_head, i_tail, OB_SIZE) )
		usbDoLoop();
	unsigned char data = in_buffer[i_tail++]; 
	i_tail &= (OB_SIZE -1);
	return data;
}

void usb_puts(unsigned char *str)
{
	int i;
	for (i=0; i<strlen(str); i++){
		usb_putc(str[i]);
	}
}

void usbFunctionWriteOut( uchar *data, uchar len )
{
	uchar pos = 0;
	do { 
		int tocopy = min_t(int, len-pos, CIRC_SPACE_TO_END(i_head, i_tail, OB_SIZE));
		memcpy(&in_buffer[i_head], &data[pos], tocopy);
		pos += tocopy;
		i_head = (i_head + tocopy) & (OB_SIZE -1);
	} while (len!=pos); 
	
	/* Postpone requests till we process all data */
	if (CIRC_SPACE(i_head, i_tail, OB_SIZE) < HW_CDC_BULK_OUT_SIZE) 
		usbDisableAllRequests(); 
}

void usbReconnect()
{
	DDRD=0xff;
	_delay_ms(250);
	DDRD=0;
}

ANTARES_INIT_LOW(io_init)
{
	DDRC=1<<2;
	PORTC=0xff;
 	usbReconnect();
}

ANTARES_INIT_HIGH(uinit)
{
	intr3Status = 0;
	sendEmptyFrame = 0;
  	usbInit();
}

ANTARES_APP(pollapp) { 
	PORTC&=~1<<2;
	unsigned char byte = usb_getc(); 
	serprog_handle_command(byte); 
}




/* ////////////////////////////////////////////////////////////////////////// */ 
/* ACTUAL SERPROG STUFF                                                       */

#define SPI_PORT PORTB
#define SCK PORTB5 /* port 13 */
#define MISO PORTB4 /* port 12 */
#define MOSI PORTB3 /* port 11 */
#define SS PORTB2 /* port 10 */
#define DDR_SPI DDRB


#define LED_ON  PORTC|=1<<2
#define LED_OFF PORTC&=~(1<<2)


inline void select_chip(void)
{
	LED_ON;
	PORTB &= ~(1<<SS);
}

inline void unselect_chip(void)
{
	LED_OFF;
	PORTB |= (1<<SS);
}

ANTARES_INIT_HIGH(setup_spi)
{
	/* set SS low */
	SPI_PORT &= ~(1<<SS);
	/* Enable MOSI,SCK,SS as output like on
	http://en.wikipedia.org/wiki/File:SPI_single_slave.svg */
	DDR_SPI = (1<<MOSI)|(1<<SCK)|(1<<SS);
	/* Enable SPI Master, set the clock to F_CPU / 2 */
	/* CPOL and CPHA are 0 for SPI mode 0 (see wikipedia) */
	/* we use mode 0 like for the linux spi in flashrom*/
	SPCR = (1<<SPE)|(1<<MSTR);
	SPSR = (1<<SPI2X);
}

char readwrite_spi(char c)
{
	char r;

	/* transmit c on the SPI bus */
	SPDR = c;
	/* Wait for the transmission to be comlpeted */
	loop_until_bit_is_set(SPSR,SPIF);
	r = SPDR;
	return r;
}



#define S_ACK 0x06
#define S_NAK 0x15

#define S_CMD_NOP          0x00UL /* No operation                                 */
#define S_CMD_Q_IFACE      0x01UL /* Query interface version                      */
#define S_CMD_Q_CMDMAP     0x02UL /* Query supported commands bitmap              */
#define S_CMD_Q_PGMNAME    0x03UL /* Query programmer name                        */
#define S_CMD_Q_SERBUF     0x04UL /* Query Serial Buffer Size                     */
#define S_CMD_Q_BUSTYPE    0x05UL /* Query supported bustypes                     */
#define S_CMD_Q_CHIPSIZE   0x06UL /* Query supported chipsize (2^n format)        */
#define S_CMD_Q_OPBUF      0x07UL /* Query operation buffer size                  */

#define S_CMD_Q_WRNMAXLEN  0x08UL /* Query Write to opbuf: Write-N maximum length */
#define S_CMD_R_BYTE       0x09UL /* Read a single byte                           */
#define S_CMD_R_NBYTES     0x0AUL /* Read n bytes                                 */
#define S_CMD_O_INIT       0x0BUL /* Initialize operation buffer                  */
#define S_CMD_O_WRITEB     0x0CUL /* Write opbuf: Write byte with address         */
#define S_CMD_O_WRITEN     0x0DUL /* Write to opbuf: Write-N                      */
#define S_CMD_O_DELAY      0x0EUL /* Write opbuf: udelay                          */
#define S_CMD_O_EXEC       0x0FUL /* Execute operation buffer                     */

#define S_CMD_SYNCNOP      0x10UL /* Special no-operation that returns NAK+ACK    */
#define S_CMD_Q_RDNMAXLEN  0x11UL /* Query read-n maximum length                  */
#define S_CMD_S_BUSTYPE    0x12UL /* Set used bustype(s).                         */
#define S_CMD_O_SPIOP      0x13UL /* Perform SPI operation.                       */


#define S_IFACE_VERSION		0x01		/* Version of the protocol */
#define S_PGM_NAME		"uISP-serprog" /* The program's name */

/* 
 * we must split in 3 parts because else avr-gcc doesn't seem to
 *  be able to compute stuff like 1<<S_CMD_SYNCNOP (it returns 0)
 */
#define SUPPORTED_COMMANDS_LOW ( ( \
	(1<<S_CMD_NOP) | (1<<S_CMD_Q_IFACE) | (1<<S_CMD_Q_CMDMAP) \
	| (1<<S_CMD_Q_PGMNAME) | (1<<S_CMD_Q_SERBUF) | (1<<S_CMD_Q_BUSTYPE) \
        ) & 0xff)
#define SUPPORTED_COMMANDS_HIGH ( ( ( \
	(1<<(S_CMD_SYNCNOP - 16)) | (1<<(S_CMD_O_SPIOP - 16)) | (1<<(S_CMD_S_BUSTYPE - 16)) \
	) & 0xff ) )

#define SUPPORTED_BUS 0x08



/* get 24bit values in little endian */
uint32_t get24_le()
{
	uint32_t val = 0;

	val = (uint32_t)usb_getc() << 0;
	val |= (uint32_t)usb_getc() << 8;
	val |= (uint32_t)usb_getc() << 16;

	return val;
}



void serprog_handle_command(unsigned char command)
{
	int i;
	char c;
	uint32_t slen = 0; /* write len */
	uint32_t rlen = 0; /* read len */
	switch (command){
		case S_CMD_NOP:
			usb_putc(S_ACK);
			break;
		case S_CMD_Q_IFACE:
			usb_putc(S_ACK);
			usb_putc(S_IFACE_VERSION);
			/* little endian multibyte value to complete to 16bit */
			usb_putc(0);
			break;
		case S_CMD_Q_CMDMAP:
			usb_putc(S_ACK);
			/* little endian */
			usb_putc(SUPPORTED_COMMANDS_LOW);
			usb_putc(0x00);
			usb_putc(SUPPORTED_COMMANDS_HIGH);
			for (i=0;i<29;i++){
				usb_putc(0x0);
			}
			break;
		case S_CMD_Q_PGMNAME:
			usb_putc(S_ACK);
			usb_puts(S_PGM_NAME);
			for (i=strlen(S_PGM_NAME);i<16;i++){
				usb_putc(0);
			}
			break;
		case S_CMD_Q_SERBUF:
			usb_putc(S_ACK);
			usb_putc(0xFF);
			usb_putc(0xFF);
			break;
		case S_CMD_Q_BUSTYPE:
			usb_putc(S_ACK);
			usb_putc(0b00001000);
			break;
		case S_CMD_Q_CHIPSIZE:
			break;
		case S_CMD_Q_OPBUF:
			break;
		case S_CMD_Q_WRNMAXLEN:
			break;
		case S_CMD_R_BYTE:
			break;
		case S_CMD_R_NBYTES:
			break;
		case S_CMD_O_INIT:
			break;
		case S_CMD_O_WRITEB:
			break;
		case S_CMD_O_WRITEN:
			break;
		case S_CMD_O_DELAY:
			break;
		case S_CMD_O_EXEC:
			break;
		case S_CMD_SYNCNOP:
			usb_putc(S_NAK);
			usb_putc(S_ACK);
			break;
		case S_CMD_Q_RDNMAXLEN:
			break;
		case S_CMD_S_BUSTYPE:
			switch (usb_getc()) {
				case SUPPORTED_BUS:
					usb_putc(S_ACK);
					break;
				default:
					usb_putc(S_NAK);
					break;
			}
			break;
		case S_CMD_O_SPIOP:
			/* get slen */
			slen = get24_le();
			/* get rlen */
			rlen = get24_le();

			select_chip();
			/* SPI is configured in little endian */
			/* TODO: Optimize this place */
			while (slen){
				while (!CIRC_CNT(i_head, i_tail, OB_SIZE))
					usbDoLoop();

				uint32_t tocopy = min_t(uint32_t, 
						   slen, CIRC_CNT_TO_END(i_head, i_tail, OB_SIZE));
				slen -= tocopy;

				while(tocopy--)   
					readwrite_spi(in_buffer[i_tail++]);
				
				i_tail &= ( OB_SIZE - 1 );				
			}
			usb_putc(S_ACK);

			/* receive TODO: handle errors */
			while (rlen){
				while (!CIRC_SPACE(o_head, o_tail, OB_SIZE))
					usbDoLoop();				
				uint32_t tocopy = min_t(uint32_t, 
						   rlen, CIRC_SPACE_TO_END(o_head, o_tail, OB_SIZE));
				rlen-=tocopy;
				while(tocopy--)
					out_buffer[o_head++] = readwrite_spi(0x0);
				o_head &= ( OB_SIZE - 1 );
			}
			unselect_chip();
			break;
		default:
			break;
	}
}
