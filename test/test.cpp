extern "C" {
    #include "gps.h"
}
#include "gps_interface.h"

#include <stdio.h>
#include <stdlib.h>

int main(void) {

    gps_files_t gps_files;
    gps_files.nmea[0] = stdout;
    gps_files.nmea[1] = stdout;
    gps_files.nmea[2] = stdout;
    gps_files.ubx[0] = stdout;
    gps_files.ubx[1] = stdout;
    gps_files.ubx[2] = stdout;
    gps_files.ubx[3] = stdout;
    gps_files.ubx[4] = stdout;

    gps_header_to_file(&gps_files);

    gps_parsed_data_t data;
    gps_protocol_and_message match;

    match.protocol = GPS_PROTOCOL_TYPE_UBX;
    match.message = GPS_UBX_TYPE_NAV_RELPOSNED;

    gps_to_file(&gps_files, &data, &match);

    return EXIT_SUCCESS;
}