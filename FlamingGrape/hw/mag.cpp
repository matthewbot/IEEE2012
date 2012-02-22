#include "mag.h"
#include "debug/debug.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

enum Axis {
	AXIS_X = 1,
	AXIS_Y = 2,
	AXIS_Z = 3
};

enum Period {
	PERIOD_32,
	PERIOD_64,
	PERIOD_128,
	PERIOD_256,
	PERIOD_512,
	PERIOD_1024,
	PERIOD_2048,
	PERIOD_4096
};

static void mag_command(Axis a, Period p);

static MagReading reading;
static Axis current_axis;

static SPI_t &magspi = SPIC;

static PORT_t &spiport = PORTC;
static const int reset_mask = _BV(4);
static const int mosi_mask = _BV(5);
static const int sclk_mask = _BV(7);

static PORT_t &ctrlport = PORTJ;
#define CTRLINT0VEC PORTJ_INT0_vect
static const int ssnot_mask = _BV(3);
static const int drdy_mask	= _BV(4);

static const Period mag_period = PERIOD_1024;

void mag_init() {
	ctrlport.DIRSET = ssnot_mask;		// Sets ssnot and reset pins to output, drdy is automatically input
	spiport.DIRSET = reset_mask | mosi_mask | sclk_mask;
	ctrlport.OUTSET = ssnot_mask;
	PORTCFG.MPCMASK = drdy_mask; // Set up data ready pin for rising edge interrupt
	ctrlport.PIN0CTRL = PORT_ISC_RISING_gc;
	ctrlport.INT0MASK = drdy_mask;	// Set data ready pin to trigger on interrupt 0
	ctrlport.INTCTRL = PORT_INT0LVL_MED_gc;
	magspi.CTRL = SPI_PRESCALER_DIV64_gc | SPI_CLK2X_bm | SPI_MASTER_bm | SPI_ENABLE_bm;	// Sets board to master, enables SPI, SCLK to x/32 (1MHz)
	current_axis = AXIS_X;
	mag_command(current_axis, mag_period);
}

MagReading mag_getReading() {
	return reading;
}

static uint8_t mag_readwrite(uint8_t out) {
	magspi.DATA = out;
	while (!(magspi.STATUS & SPI_IF_bm)) { }
	return magspi.DATA;
}

static void mag_command(Axis a, Period p) {
	ctrlport.OUTCLR = ssnot_mask;
	spiport.OUTSET = reset_mask;
	_delay_us(1);
	spiport.OUTCLR = reset_mask;
	uint8_t byte = (p << 4) | a;
	mag_readwrite(byte);
	ctrlport.OUTSET = ssnot_mask;
}

static int16_t mag_read() {
	ctrlport.OUTCLR = ssnot_mask;
	uint8_t msb = mag_readwrite(0);
	uint8_t lsb = mag_readwrite(0);
	ctrlport.OUTSET = ssnot_mask;
	return (msb << 8) | (lsb);
}

ISR(CTRLINT0VEC) {
	int16_t val = mag_read();
	if (current_axis == AXIS_X) {
		reading.x = val;
		current_axis = AXIS_Y;
	} else if (current_axis == AXIS_Y) {
		reading.y = val;
		current_axis = AXIS_Z;
	} else {
		reading.z = val;
		current_axis = AXIS_X;
	}
	mag_command(current_axis, mag_period);
}
