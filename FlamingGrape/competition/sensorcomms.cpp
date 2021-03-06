#include "competition/sensorcomms.h"
#include "hw/uart.h"
#include "hw/tick.h"
#include "debug/debug.h"
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

// constants
static const unsigned int timeout_ticks = TICK_HZ/2; // maximum number of ticks before an update times out

// board updating information
static volatile BoardNum updateboard; // current board we're updating
static volatile uint32_t updatestart; // the time we started attempting to update it. 0 if we're not updating a board.
static volatile uint8_t readinglen; // the length of the reading we're receiving
static volatile uint8_t readingpos; // the current position we are in receiving the reading

// board ID assignment information
static volatile int8_t nextnum; // the next number to assign
static volatile uint32_t assignstart; // the time we started attempting to assign nextnum. 0 if we're not assigning

// per board Data
struct BoardData {
	volatile BoardStatus status;
	uint8_t reading[sensorcomms_readinglen];
	bool reading_valid;
};

static BoardData data[BOARDNUM_MAX];

// receiving state machine
enum ReceiveState {
	STATE_COMMAND,
	STATE_READINGLEN,
	STATE_READINGDATA,
	STATE_READINGCHECKSUM,
};
static volatile ReceiveState recvstate;
static volatile bool debug;

// byte values
enum {
	BYTE_WANTID = 0xF0,
	BYTE_ASSIGNID = 0xF1, // F2, F3, F4
	BYTE_GOTID = 0xF5,
	BYTE_MAKEREADING = 0xF6, // F7, F8, F9
	BYTE_READINGSTART = 0xFF
};

static void out(uint8_t byte);
static bool in(uint8_t byte);

// public interface

void sensorcomms_init() {
	sensorcomms_reset();
}

void sensorcomms_reset() {
	nextnum = BOARDNUM_SIGNAL;
	assignstart = false;
	updatestart = 0;
	recvstate = STATE_COMMAND;
	
	for (int i=0; i<BOARDNUM_MAX; i++) {
		data[i].status = BOARDSTATUS_OFFLINE;
		data[i].reading_valid = false;
	}
}

uint8_t sensorcomms_getOnlineBoardCount() {
	uint8_t ctr=0;
	for (int i=0; i<BOARDNUM_MAX; i++)
		if (data[i].status != BOARDSTATUS_OFFLINE)
			ctr++;
	return ctr;
}

void sensorcomms_setOnlineBoardCount(uint8_t num) {
	nextnum = (BoardNum)(BOARDNUM_MAX-1-num);
}

BoardStatus sensorcomms_getBoardStatus(BoardNum board) {
	return data[board].status;
}

void sensorcomms_updateBoard(BoardNum board) {
	updateboard = board;
	updatestart = tick_getCount();
	data[board].status = BOARDSTATUS_UPDATING;
	out(BYTE_MAKEREADING + board);
	recvstate = STATE_COMMAND;
}

bool sensorcomms_waitBoard(BoardNum board, int msecs) {
	BoardData &bd = data[board];
	while (bd.status != BOARDSTATUS_READY && --msecs > 0) { _delay_ms(1); }
	return bd.status == BOARDSTATUS_READY;
}

void sensorcomms_cancelUpdate() {
	updatestart = 0;
}

const uint8_t *sensorcomms_getBoardReading(BoardNum board) {
	return data[board].reading;
}

bool sensorcomms_getBoardReadingValid(BoardNum board) {
	return data[board].reading_valid;
}

void sensorcomms_setDebug(bool newdebug) {
	debug = newdebug;
}

// internals

void sensorcomms_tick() {
	if (updatestart != 0) {
		if (tick_getCount() - updatestart > timeout_ticks) // timed out
			sensorcomms_updateBoard(updateboard); // restart the update
	}
	
	if (assignstart != 0) {
		if (tick_getCount() - assignstart > timeout_ticks) {
			out(BYTE_ASSIGNID + nextnum);
			assignstart = tick_getCount();
		}
	}
}

// called from uart ISR when xbee gets a byte
bool sensorcomms_gotByte(uint8_t byte) {
	if (in(byte)) {
		if (debug)
			debug_println("<%02x>", byte);
		return true;
	}
	
	return false;
}

static bool in(uint8_t byte) {
	if (recvstate == STATE_COMMAND) {
		switch (byte) {
			case BYTE_WANTID:
				if (nextnum < 0) {
					static bool once=false;
					if (!once)
						printf_P(PSTR("Got WantID, but no IDs left!\n"));
					once = true;
					debug_setLED(ERROR_LED, true); 
					return true;
				}
				
				data[nextnum].status = BOARDSTATUS_ASSIGNING;
				out(BYTE_ASSIGNID + nextnum);
				assignstart = tick_getCount();
				return true;
				
			case BYTE_GOTID:
				if (assignstart == 0)
					break;

				data[nextnum].status = BOARDSTATUS_READY;
				assignstart = 0;
				nextnum--;
				return true;
				
			case BYTE_READINGSTART:
				recvstate = STATE_READINGLEN;
				return true;
		}
		
		return false;
	} else if (recvstate == STATE_READINGLEN) {
		if (byte <= sensorcomms_readinglen) { // reading too big, probably corrupt
			readinglen = byte;
			readingpos = 0;
			recvstate = STATE_READINGDATA;
		} else {
			recvstate = STATE_COMMAND;
		}
		
		return true;
	} else if (recvstate == STATE_READINGDATA) {
		BoardData &updatedata = data[updateboard];
		
		updatedata.reading[readingpos++] = byte;
		if (readingpos == readinglen)
			recvstate = STATE_READINGCHECKSUM;
			
		return true;
	} else {
		BoardData &updatedata = data[updateboard];
		
		uint8_t checksum=0xFF;
		checksum += readinglen;
		for (int i=0; i<readinglen; i++)
			checksum += updatedata.reading[i];

		updatedata.reading_valid = checksum == byte;
		updatedata.status = BOARDSTATUS_READY;
		updatestart = 0; // signals we're done updating
		recvstate = STATE_COMMAND;
		
		return true;
	}
}

static void out(uint8_t byte) {
	uart_put(UART_XBEE, byte);
	
	if (debug)
		debug_println(">%x<", byte);
}

// printing utilities
void sensorcomms_printBoardStatus(BoardStatus status) {
	static char offline_str[] PROGMEM = "OFFLINE";
	static char assigning_str[] PROGMEM = "ASSIGNING";
	static char updating_str[] PROGMEM = "UPDATING";
	static char ready_str[] PROGMEM = "READY";
	
	static PGM_P table[] PROGMEM = {
		offline_str,
		assigning_str,
		updating_str,
		ready_str
	};
	
	printf_P((PGM_P)pgm_read_word(table + status));
}
