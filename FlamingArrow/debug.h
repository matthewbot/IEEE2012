#ifndef DEBUG_H_
#define DEBUG_H_

#include <stdarg.h>
#include <stdlib.h>

void debug_init();
void debug_setLED(bool on);

void debug_putch(char ch);
void debug_puts(const char *s);
void debug_printf(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));

char debug_getch();
size_t debug_gets(char *buf, size_t amt);

#endif
