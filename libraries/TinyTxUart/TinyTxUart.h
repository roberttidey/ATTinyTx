// TinyTxUart.h
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
#include <Arduino.h>

//used for debugging, comment out to disable
#define FLAG_ACTIVE 1
#define FLAG_MASK 0x20
#ifdef FLAG_ACTIVE
	#define FLAG_ON PORTB |= FLAG_MASK;
	#define FLAG_OFF PORTB &= ~FLAG_MASK;
	#define FLAG_PULSE PORTB |= FLAG_MASK;PORTB &= ~FLAG_MASK;
#else
	#define FLAG_ON
	#define FLAG_OFF
	#define FLAG_PULSE
#endif

//Number of bits for UART RX to count to get ready
#define RXCOUNT 10
//Number of bits for UART TX to count to transmit
#define TXCOUNT 10

//RX buffer size. Should be 2^n - 1
#define RXBUFFER_MASK 7

void tinyIU_init(uint8_t ticks, uint8_t baud, uint8_t TXpin,  uint8_t RXpin);

//Basic send message, txData buffer needs to be constructed first
void tinyIU_sendtx(uint8_t* widths, uint8_t repeats, uint8_t modulation);

//Make and Send NEC message
void tinyIU_sendNEC(uint8_t* pars, uint8_t repeats, uint8_t modulation);

//Make and Send rc message rcType = 0 (rc5) else rc6 and rcType has length of cmd bits
void tinyIU_sendRC(uint8_t rcType, uint8_t toggle, uint8_t* pars, uint8_t repeats, uint8_t modulation);

//Make and Send lightwave message, 5 Bytes
void tinyIU_sendLW(uint8_t* lwBytes, uint8_t repeats);

//Get raw txdata pointer
uint8_t* tinyIU_getBufferPtr();

//Get current txdata length
uint8_t tinyIU_getBufferLen();

//Basic send Tx bytes
void tinyIU_sendTx(uint8_t* txD, uint8_t TxLen);

//Basic receive of Rx byte
uint8_t tinyIU_getRx();

//Checks whether tx is free to accept a new message
uint8_t tinyIU_txFree();

//returns 0 when free
uint8_t tinyIU_txByteCount();

//returns number of Bytes available
uint8_t tinyIU_rxByteCount();

// clear out RX
void tinyIU_rxReset();
