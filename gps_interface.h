#pragma once

#include <termios.h>

#include "gps.h"

typedef struct gps_serial_port{
    char* port;
    int open;
    int fd;
}gps_serial_port;

int gps_interface_open(gps_serial_port* new_serial_port, const char* port, speed_t speed);
void gps_interface_close(gps_serial_port* serial_port);

gps_protocol_type gps_interface_get_line(gps_serial_port* port, char start_sequence[GPS_MAX_START_SEQUENCE_SIZE], int* start_sequence_size, char line[GPS_MAX_LINE_SIZE], int* line_size);