#ifndef SERIAL_H
#define SERIAL_H

#include <termios.h>

int open_serial(const char *port, int baudrate);
void close_serial(int fd);

#endif
