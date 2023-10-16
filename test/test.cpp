extern "C" {
    #include "gps.h"
    #include "gps_interface.h"
}

#include <stdio.h>
#include <stdlib.h>

int main(void) {

    gps_files_t gps_files;
    for(int i = 0; i < GPS_NMEA_TYPE_SIZE; i++) {
        gps_files.nmea[i] = stdout;
    }
    for(int i = 0; i < GPS_UBX_TYPE_SIZE; i++) {
        gps_files.ubx[i] = stdout;
    }

    gps_header_to_file(&gps_files);

    gps_serial_port port;
    int res = gps_interface_open(&port, "/dev/ttyACM0", B230400);
    if(res == -1) {
        printf("Error opening serial port\n");
        return EXIT_FAILURE;
    }

    unsigned char start_sequence[GPS_MAX_START_SEQUENCE_SIZE];
    char line_buffer[GPS_MAX_LINE_SIZE];
    int start_sequence_size;
    int line_buffer_size;

    gps_parsed_data_t data;
    while(true) {
        gps_protocol_type protocol = gps_interface_get_line(&port, start_sequence, &start_sequence_size, line_buffer, &line_buffer_size, false);  
        if(protocol == GPS_PROTOCOL_TYPE_SIZE) {
            printf("Error reading line\n");
            continue;
        }
        gps_protocol_and_message prot_and_msg;
        if(gps_match_message(&prot_and_msg, line_buffer, protocol) == -1) {
            // printf("Error matching message\n");
            continue;
        }
        
        if(gps_parse_buffer(&data, &prot_and_msg, line_buffer, 0) == -1) {
            printf("Error parsing buffer\n");
            continue;
        }

        printf("UBX %s: ", gps_ubx_message_type_string((gps_ubx_message_type)prot_and_msg.message));
        gps_to_file(&gps_files, &data, &prot_and_msg);
    }

    return EXIT_SUCCESS;
}