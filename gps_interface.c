#include "gps_interface.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <sys/time.h>

uint64_t gps_get_timestamp(gps_serial_port* port) {

    if (port == NULL)
        return 0;

    char str[20];
    char c = '\0';
    int idx = 0;
    int in_timestamp = 0;
    int timestamp = 0;

    while (c != '(')
        read(port->fd, &c, 1);

    read(port->fd, &c, 1);

    while (c != ')' && idx < 16) {
        str[idx++] = c;
        read(port->fd, &c, 1);
    }

    if (idx != 16)
        return 0;

    str[idx] = '\0';

    return (uint64_t)strtol(str, NULL, 10);
}

int gps_interface_open_file(gps_serial_port* new_serial_port, const char* filename) {
    if(filename == NULL)
        return -1;

    new_serial_port->open = 0;
    new_serial_port->type = LOG_FILE;

    new_serial_port->fd = open(filename, O_RDONLY);

    if (new_serial_port->fd == -1)
        return -1;  // Error

    new_serial_port->open = 1;
    new_serial_port->last_timestamp = UINT64_MAX-1;
    return 0;
}

int gps_interface_open(gps_serial_port* new_serial_port, const char* port, speed_t speed){
    if(port == NULL)
        return -1;

    new_serial_port->open = 0;
    new_serial_port->fd = open(port, O_RDWR);
    // Handle in case of error
    if (new_serial_port->fd == -1){
        printf("GPS Interface: Error opening fd\n");
        return -1;
    }
    new_serial_port->port = (char*)malloc(strlen(port));
    strcpy(new_serial_port->port, port);


    struct termios tty;

    // Read in existing settings and handle errors
    if (tcgetattr(new_serial_port->fd, &tty) != 0)
    {
        printf("GPS Interface: Error tcgetattr\n");
        return -1;
    }

    // Setting baud rate
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag &= ~PARENB;        // disable parity bit
    tty.c_cflag &= ~CSTOPB;        // clear stop field
    tty.c_cflag |= CS8;            // 8 data bits per byte
    tty.c_cflag &= ~CRTSCTS;       // disable TRS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // turn on READ and ignore control lines, setting CLOCAL allows us to read data
    // local modes
    tty.c_lflag &= ~ICANON; // disable canonical mode, in canonical mode input data is received line by line, usually undesired when dealing with serial ports
    tty.c_lflag &= ~ECHO;   // if this bit (ECHO) is set, sent characters will be echoed back.
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG; // when the ISIG bit is set, INTR,QUIT and SUSP characters are interpreted. we don't want this with a serial port
    // the c_iflag member of the termios struct contains low-level settings for input processing. the c_iflag member is an int
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // clearing IXON,IXOFF,IXANY disable software flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // clearing all of this bits disable any special handling of received bytes, i want raw data
    // output modes (c_oflag). the c_oflag member of the termios struct contain low level settings for output processing, we want to disable any special handling of output chars/bytes
    tty.c_oflag &= ~OPOST; // prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR; // prevent conversion of newline to carriage return/line feed
    // setting VTIME VMIN
    tty.c_cc[VTIME] = 10; // read() will block until either any amount of data is received or the timeout ocurs
    tty.c_cc[VMIN] = 0;

    // After changing settings we need to save the tty termios struct, also error checking
    if (tcsetattr(new_serial_port->fd, TCSANOW, &tty) != 0)
    {
        printf("GPS Interface: Error tcsetattr\n");
        return -1;
    }

    new_serial_port->open = 1;

    return 0;
}

void gps_interface_close(gps_serial_port* serial_port){
    if(serial_port->open == 0)
        return;
    close(serial_port->fd);
    serial_port->open = 0;
}

gps_protocol_type gps_interface_get_line(gps_serial_port* port, char start_sequence[GPS_MAX_START_SEQUENCE_SIZE], int* start_sequence_size, char line[GPS_MAX_LINE_SIZE], int* line_size){
    uint8_t c;
    int size = -1;
    *line_size = size;
    int cr = 0; // flag for <CR> termination byte (NMEA protocol)
    int ubx_message_size = 0;
    gps_protocol_type type = GPS_PROTOCOL_TYPE_SIZE;

    if(port->type == LOG_FILE){
        uint64_t current = gps_get_timestamp(port);
        if(current > port->last_timestamp)
            usleep(current - port->last_timestamp);
        if (current != 0)
            port->last_timestamp = current;
    }

    while(size < GPS_MAX_LINE_SIZE){
        if(read(port->fd, &c, 1) <= 0)
            return GPS_PROTOCOL_TYPE_SIZE;

        // if no match
        if(size == -1){
            switch(c){
                // first sync byte for ubx message
                case 0xb5:
                    // read second syncronization byte
                    if(read(port->fd, &c, 1) <= 0)
                        return GPS_PROTOCOL_TYPE_SIZE;
                    // second sync byte for ubx message
                    if(c == 0x62){
                        type = GPS_PROTOCOL_TYPE_UBX;
                        size = 0;
                        start_sequence[0] = 0xb5;
                        start_sequence[1] = 0x62;
                        *start_sequence_size = 2;
                    }
                break;
                // first sync byte for nmea message
                case 0x24:
                    // read second syncronization byte
                    if(read(port->fd, &c, 1) <= 0)
                        return GPS_PROTOCOL_TYPE_SIZE;
                    // match second syncronyzation byte
                    if(c == 0x47 /*G*/ || c == 0x50 /*P*/){
                        type = GPS_PROTOCOL_TYPE_NMEA;
                        size = 0;
                        start_sequence[0] = 0x24;
                        start_sequence[1] = c;
                        // extra start sequence byte
                        if(read(port->fd, &c, 1) <= 0)
                            return GPS_PROTOCOL_TYPE_SIZE;
                        start_sequence[2] = c;
                        *start_sequence_size = 3;
                        ubx_message_size = 0;
                    }
                break;
            }
            // if no match, size is still -1.
            // if a match, size will be 0 and type will be or ubx or nmea.
            continue;
        }

        if(type == GPS_PROTOCOL_TYPE_NMEA){
            if(c == 0x0D){  // <CR> termination byte
                cr = 1;
            }
            else if(c == 0x0A && (cr == 1 || cr == 0)){ // <LF> termination byte
                size -= 2;    // because line is not updated with <CR> and <LF>
                break;
            } else {
                cr = 0;
                line[size] = c;
            }
        }else if(type == GPS_PROTOCOL_TYPE_UBX){
            // in byte 2 and 3 in ubx messages is specified the length of the payload (in lsb)
            // the lenght does not contains the first 4 bytes (class, id, lenght1, length2) and the last two bytes (checksum a, checksum b)
            if(size == 2)
                ubx_message_size += c;
            else if(size == 3)
                ubx_message_size += c << 8;

            line[size] = c;

            // check if the size matches the ubx_message_size
            if(size >= 3 && size - 6 == ubx_message_size)
                break;
        }

        size ++;
    }
    // sets last byte at 0
    if(size ++ < GPS_MAX_LINE_SIZE)
        line[size ++] = '\0';

    *line_size = size;
    return type;
}