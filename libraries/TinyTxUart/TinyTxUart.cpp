// TinyTxUart.cpp
//
// General purpose interrupt driven bit transmitter for ATTiny85
// Message buffer consists of 8 bit bytes.
// if byte < 248
//	  Bit 7 is level to write to pin
//	  Bits 6 - 0 is tick count to maintain this level before next change
// else
//    bit count = byte - 247
//    bits from next byte in buffer (left justified) transmitted using 2 widths from
//    widths structure : 4 byte (level + delay) for low0,low1,high0,high1
//	Tick period is 26us
//  Minimum period should be > 2 ticks 52uS 
//  Maximum period is 127 * 26 = 3.2mSec use multiple periods for longer
// ir carrier modulation for 1 state (must use pin 2 or 14)
//
// Supports a serial RX, TX at 1200, 2400, 4800 baud
// Recommended to use 2400
// Author: Bob Tidey (robert@tideys.co.uk)

#include "TinyTxUart.h"

#define IRMask 2	//PB1 OC0B
#define DFLT_TIM0TICKS 108 // 26uS (38.4KHz) 8MHz clock

static const uint8_t lw_nibble[16] = {0x7A,0x76,0x75,0x73,0x6E,0x6D,0x6B,0x5E,0x5D,0x5B,0x57,0x3E,0x3D,0x3B,0x27,0x2F};
static uint8_t lwWidths[4] = {0x8B,0x31,0x8B,0x0B}; //low0,low1,high0,high1
static uint8_t necWidths[4] = {0x96,0x16,0x96,0x40}; //low0,low1,high0,high1
static uint8_t rc5Widths[4] = {0xA2,0x22,0x22,0xA2}; //low0,low1,high0,high1
static uint8_t rc6Widths[4] = {0x11,0x91,0x91,0x11}; //low0,low1,high0,high1
//variables
uint8_t txData[40];
uint8_t txDataLen = 0;
static uint8_t txNextPeriod = 0;
static uint8_t* txWidths;
static uint8_t txBits;
static uint8_t txBitCount;
static uint8_t txRepeats = 0;
static uint8_t rxtxBaud; // Divider 4,8,16,32 (9600,4800,2400,1200)
static uint8_t* tx_bufptr; // the message buffer pointer during transmission
static uint8_t TXMask; //mask for TX pin used for transmit
static uint8_t TXMaskI; //inverted mask for TX pin used for transmit
static uint8_t RXMask; //mask for TX pin used for transmit
static uint8_t* TXBytes; //Pointer to transmit data
static uint8_t TXByte; //byte to transmit uart
static volatile uint8_t TXByteCount; //counter for bytes to transmit
static int8_t TXState = 0; //transmit uart state
static uint8_t TXDiv; //Divider during tx
static uint8_t RXBuffer[RXBUFFER_MASK + 1]; //bytes being received
static volatile uint8_t RXBufferHead = 0;
static volatile uint8_t RXBufferTail = 0;
static uint8_t RXTicks; //Tick counter to sample bit
static uint8_t RXByte; //byte being received
static uint8_t RXState; //receive uart state

const uint8_t nibbleRev[16] PROGMEM = {
	0x0, 0x8, 0x4, 0xC,
	0x2, 0xA, 0x6, 0xE,
	0x1, 0x9, 0x5, 0xD,
	0x3, 0xB, 0x7, 0xF
};

uint8_t reverseBits(uint8_t b) {
	uint8_t h, l;
	h = pgm_read_byte(&nibbleRev[b >> 4]);
	l = pgm_read_byte(&nibbleRev[b & 0x0f]) << 4;
	return (l | h );
}

ISR(TIM0_OVF_vect) {
	uint8_t d;
	//FLAG_ON
	if(txRepeats > 0) {
		txNextPeriod--;
		if(txNextPeriod == 0) {
			if(txBitCount == 0) {
				if(*tx_bufptr > 247) {
					txBitCount = ((*tx_bufptr & 0x7) + 1) << 1;
					tx_bufptr++;
					txBits = *tx_bufptr;
					tx_bufptr++;
				}
			}
			if(txBitCount){
				if(txBits & 0x80) {
					d = txWidths[2 + (txBitCount & 0x1)];
				} else {
					d = txWidths[txBitCount & 0x1];
				}
				if(txBitCount & 0x1) {
					txBits <<= 1;
				}
				txBitCount--;
			} else {
				d = *tx_bufptr;
				tx_bufptr++;
			}
			if (d & 0x80){
				DDRB |= IRMask;
			} else {
				DDRB &= ~IRMask;
			}
			//Set next period
			txNextPeriod = d & 0x7f;
			//terminate isr if a period is 0 or short (52uSec)
			if(txNextPeriod < 2) {
				//end of 1 message
				txRepeats--;
				if(txRepeats == 0) {
					//set input no pull up
					DDRB &= ~IRMask;
					PORTB &= ~IRMask;
				} else {
					tx_bufptr = txData;
				}
			}
		}
	}
	if(TXByteCount) {
		if(TXDiv) {
			TXDiv--;
		} else {
			if(TXState == 0) {
				// Start bit
				PORTB &= TXMaskI; 
			} else if(TXState < (TXCOUNT - 1)) {
				//Data Bit
				if(TXByte & 1) {
					PORTB |= TXMask;
				} else {
					PORTB &= TXMaskI; 
				}
				TXByte >>= 1;
			} else {
				//Stop bit
				PORTB |= TXMask;
				if(TXState == TXCOUNT) {
					TXByteCount--;
					TXBytes++;
					TXByte = TXBytes[0];
					TXState = -1;
				}
			}
			TXState++;
			TXDiv = rxtxBaud - 1;
		}
	}
	if(RXState) {
		RXTicks++;
		if(RXTicks == rxtxBaud) {
			if(RXState == 10) {
				RXBuffer[RXBufferHead] = RXByte;
				RXBufferHead = (RXBufferHead + 1) & (RXBUFFER_MASK); 
				RXState = 0;
			} else {
				RXByte >>= 1;
				if((PINB & RXMask) != 0) {
					RXByte |= 0x80;
				}
				RXTicks = 0;
				RXState++;
			}
		}
	}
	//FLAG_OFF
}

ISR(PCINT0_vect) {
	if(RXState == 0) {
		//start bit, set sample at half bit intervals
		RXTicks = (rxtxBaud >> 1) - 1;
		RXState++;
	}
}

/**
  Set things up to transmit messages
**/
void tinyIU_init(uint8_t ticks, uint8_t baud, uint8_t TXpin,  uint8_t RXpin) {
	//38.4KHz, 26.0uS for a 8Mz clock
	uint8_t ocr = ticks ? ticks : DFLT_TIM0TICKS;
	TCCR0B = 0;			// Stop Counter
	txRepeats = 0;	// clear message transmit
	OCR0A = ocr;
	OCR0B = OCR0A >> 1;	//50% duty cycle
	//set ir input no pull up
	DDRB &= ~IRMask;
	PORTB &= ~IRMask;
	TCCR0A = 0x31;	//OCR0B set/clear, PWM PHASE CORRECT
	TIMSK &= ~(1<<TOIE0); //Disable Timer0 Overflow Interrupt
	rxtxBaud = baud;
	TXMask = 1 << TXpin;
	TXMaskI = ~TXMask;
	if(TXpin < 6) {
		TXMask = 1 << TXpin;
		TXMaskI = ~TXMask;
		PORTB |= TXMask;
		DDRB |= TXMask;
	}
	RXMask = 1<< RXpin;
	if(RXpin < 6) {
		GIMSK |= (1<<PCIE);
		PCMSK |= RXMask;
		RXBufferHead = 0;
		RXBufferTail = 0;
		RXState = 0;
		//input no pull up
		DDRB &= ~RXMask;
		PORTB &= ~RXMask;
	}
	TCCR0B = 9; //Start counter TOP = OCR0A, 16MHz
	TIMSK |= (1<<TOIE0); //Enable Timer0 Overflow Interrupt
}

/**
  Send a message
**/
void tinyIU_sendtx(uint8_t* widths, uint8_t repeats, uint8_t modulation) {
	tx_bufptr = txData;
	txWidths = widths;
	if(modulation) {
		PORTB &= ~IRMask;
		TCCR0A = 0x31;	//OCR0B set/clear, PWM PHASE CORRECT
	} else {
		//set to output high for non modulated
		PORTB |= IRMask;
		TCCR0A = 0x01;	//NORMAL, PWM PHASE CORRECT
	}
	TCNT0 = OCR0B + 1;
	txRepeats = repeats;
}

/**
  send a NEC coded message
    addr, ~addr, cmd, ~cmd
	txData buffer needs 18 bytes of space
**/
void tinyIU_sendNEC(uint8_t* pars, uint8_t repeats, uint8_t modulation) {
	uint8_t i;
	uint8_t k;
	uint8_t b;
	txData[0] = 0xF7; // 3.1mS high
	txData[1] = 0xF7; // 3.1mS high
	txData[2] = 0xEE; // 2.8mS high
	txData[3] = 0x7F; // 3.3mS low
	txData[4] = 0x2E; // 1.2mS low
	k = 5;
	for(i=0; i < 4; i++) {
		switch(i) {
			case 0 : b = *pars; break;
			case 1 : b = ~*pars; break;
			case 2 : b = *(pars+1); break;
			case 3 : b = ~*(pars+1); break;
		}
		b = reverseBits(b);
		txData[k++] = 0xFF; // 8 bits
		txData[k++] = b; // 8 bits
	}
	txData[k++] = 0x96; //560uS high
	txData[k++] = 0x16; //560uS low
	txData[k++] = 0x00; //end
	txDataLen = k;
	tinyIU_sendtx(necWidths, repeats, modulation);
}

/**
  make a rc5 or rc6 coded message
    rcType 11 = rc5 else rc6 bitCount (16 or 32)
	msg buffer needs 42(rc5), 78 bytes of space
**/
void tinyIU_sendRC(uint8_t rcType, uint8_t toggle, uint8_t* pars, uint8_t repeats, uint8_t modulation) {
	uint8_t i;
	uint8_t b;
	uint8_t k;
	uint8_t t = 0;
	uint8_t p;

	if(rcType > 12) {
		txWidths = rc6Widths;
		//rc6 start pulse
		txData[0] = txWidths[0] * 6 | 0x80; // 2.65mS high
		txData[1] = txWidths[0] << 1; // 900uS low
		// start(1) + 3 field(0) bits
		txData[2] = 0xFB; // 4 bits
		txData[3] = 0x80;
		p = txWidths[0] << 1;
		if(toggle & 0x40) {
			txData[4] = p | 0x80; // high
			txData[5] = p; // low
		} else {
			txData[4] = p; // low
			txData[5] = p | 0x80; // high
		}
		k = 6;
	} else {
		// rc5
		txWidths = rc5Widths;
		b = 0x80 | (*(pars+1) & 0x40) | (toggle ? 0x20 : 0);
		txData[0] = 0xFA; // 3 bits
		txData[1] = b;
		//extended command bit
		k = 2;
	}
	for(i=0; i < 4 && t < rcType; i++) {
		p = 8;
		b = pars[i];
		if(rcType < 13) {
			p = i ? 6 : 5;
			b <<= (8 - p);
		}
		txData[k++] = 0xF8 + (p-1); // p bits
		txData[k++] = b;
		t += p;
	}
	txData[k++] = 10; // set low
	txData[k++] = 0x00; //end
	txDataLen = k;
	tinyIU_sendtx(txWidths, repeats, modulation);
}

/**
  make a lightwave message
    10 nibbles
	msg buffer needs 26 bytes of space
**/
void tinyIU_sendLW(uint8_t* lwBytes, uint8_t repeats) {
	uint8_t i;
	uint8_t k;
	txData[0] = 0x7F; // 3.3mS low
	txData[1] = 0x7F; // 1.2mS low
	txData[2] = 0x7F; // 3.3mS low
	k = 3;
	txData[k++] = lwWidths[2]; // widthHigh0
	txData[k++] = lwWidths[3]; // widthHigh1
	for(i = 0; i < 5; i++) {
		txData[k++] = 0xFE; // send high nibble
		txData[k++] = lw_nibble[lwBytes[i] >> 4] << 1; // send next Nibble encoded as 7 bits msb justified
		txData[k++] = 0xFE; // send low nibble
		txData[k++] = lw_nibble[lwBytes[i] & 0xF] << 1; // send next Nibble encoded as 7 bits msb justified
	}
	txData[k++] = lwWidths[2]; // widthHigh0
	txData[k++] = lwWidths[3]; // widthHigh1
	txData[k++] = 0x00; //end
	txDataLen = k;
	tinyIU_sendtx(lwWidths, repeats, 0);
}

//Get raw txdata pointer
uint8_t* tinyIU_getBufferPtr() {
	return txData;
}

//Get current txdata length
uint8_t tinyIU_getBufferLen() {
	return txDataLen;
}



//Basic send of Tx byte
void tinyIU_sendTx(uint8_t* TxData, uint8_t TxLen) {
	TXBytes = TxData;
	TXByte = TxData[0];
	TXDiv = rxtxBaud - 1;
	TXState = 0;
	TXByteCount = TxLen;
}

//Basic receive of Rx byte
uint8_t tinyIU_getRx() {
	uint8_t ch = RXBuffer[RXBufferTail];
	RXBufferTail = (RXBufferTail + 1) & RXBUFFER_MASK;
return ch;
}

/**
  Check for tx send free
**/
uint8_t tinyIU_txFree() {
  return (txRepeats == 0);
}

/**
  get tx count
**/
uint8_t tinyIU_txByteCount() {
  return TXByteCount;
}

uint8_t tinyIU_rxByteCount() {
	return (RXBufferHead - RXBufferTail) & (RXBUFFER_MASK);
}

/**
  clear out RX
**/
void tinyIU_rxReset() {
	RXState = 0;
	RXBufferHead = 0;
	RXBufferTail = 0;
}