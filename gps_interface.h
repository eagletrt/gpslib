#pragma once

#include <stdbool.h>
#include <sys/types.h>
#include <termios.h>

#include "gps.h"

enum SERIAL_MODE { USB, LOG_FILE };

typedef struct gps_serial_port {
	enum SERIAL_MODE type;
	char *port;
	int open;
	int fd;
	uint64_t last_timestamp;
	__off_t read_offset;
} gps_serial_port;

void gps_interface_initialize(gps_serial_port *);

int gps_interface_open(gps_serial_port *new_serial_port, const char *port,
											 speed_t speed);
int gps_interface_open_file(gps_serial_port *new_serial_port,
														const char *filename);
void gps_interface_close(gps_serial_port *serial_port);

gps_protocol_type gps_interface_get_line(
		gps_serial_port *port,
		unsigned char start_sequence[GPS_MAX_START_SEQUENCE_SIZE],
		int *start_sequence_size, char line[GPS_MAX_LINE_SIZE], int *line_size,
		bool sleep);
