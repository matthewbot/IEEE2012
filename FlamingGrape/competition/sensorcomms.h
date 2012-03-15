#ifndef SENSORCOMMS_H
#define SENSORCOMMS_H

#include <stdint.h>

static const int sensorcomms_readinglen = 20; // maximum length of a reading

void sensorcomms_init();
void sensorcomms_tick();
bool sensorcomms_gotByte(uint8_t byte);
void sensorcomms_reset();
void sensorcomms_setDebug(bool debug);

enum BoardStatus {
	BOARDSTATUS_OFFLINE,
	BOARDSTATUS_ASSIGNING,
	BOARDSTATUS_UPDATING,
	BOARDSTATUS_READY
};
void sensorcomms_printBoardStatus(BoardStatus status);

enum BoardNum {
	BOARDNUM_VOLTAGE,
	BOARDNUM_CAPACITANCE,
	BOARDNUM_TEMPERATURE,
	BOARDNUM_SIGNAL,
	BOARDNUM_MAX
};

uint8_t sensorcomms_getOnlineBoardCount();
void sensorcomms_setOnlineBoardCount(uint8_t ctr);
BoardStatus sensorcomms_getBoardStatus(BoardNum board);
void sensorcomms_updateBoard(BoardNum board);
bool sensorcomms_waitBoard(BoardNum board, int msecs);
void sensorcomms_cancelUpdate();
const uint8_t *sensorcomms_getBoardReading(BoardNum board);
bool sensorcomms_getBoardReadingValid(BoardNum board);



#endif
