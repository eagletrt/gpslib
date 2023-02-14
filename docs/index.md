# GPS lib

Library to interface with GPS devices. More specifically parses NMEA and UBX messages.

## Features

Reads from serial interface data from a GPS device. Then matches the data with the known messages (NMEA and UBX), then it parses the data and eventually saves the parsed values in corresponding CSV files.

### Parsed messages

***NMEA***

- GGA
- GSA
- VTG

***UBX***

- NAV-DOP
- NAV-PVT
- NAV-HPPOSECEF
- HPPOSLLH

## Usage

### Reading from GPS

Use **gps_interface.h** to access to utils functions to read data from the connected GPS.

~~~C
int gps_interface_open(
    gps_serial_port* new_serial_port,
    const char* port,
    speed_t speed
);

gps_protocol_type gps_interface_get_line(
    gps_serial_port* port,
    char start_sequence[GPS_MAX_START_SEQUENCE_SIZE],
    int* start_sequence_size,
    char line[GPS_MAX_LINE_SIZE],
    int* line_size
);
~~~

- gps_interface_open: is to open a serialport given the device name (ex: /dev/ttyACM0) and given speed.
- gps_interface_get_line: reads a line from the device returning the protocol of the line, the start sequence and the actual line.

~~~C
gps_protocol_type protocol_type = gps_interface_get_line(&port, start_sequence, &start_sequence_bytes, line, &line_bytes);
if(protocol_type == GPS_PROTOCOL_TYPE_SIZE){
    cout << "error reading gps" << endl;
}else{
    // All ok, do stuff
}
~~~

Setting protocol_type to max, is used to notify that an error occurred reading the port.

### Parsing GPS data

To parse GPS data use functions and structures in gps.h.

Define the data struct:

~~~C
gps_parsed_data_t data;
~~~

To parse data fist must be detected the message type, then if it is a velid type the parsing can take place:

~~~C
gps_protocol_and_message match;
if(gps_match_message(&match, line, protocol_type) == -1){
    printf("Protocol: %d, match failed", match.protocol); // it fails if the message is not supported from this lib
    continue;
}

gps_parse_result_t result = gps_parse_buffer(&parsed_gps, &match, line, timestamp);
if(result == GPS_PARSE_RESULT_OK){
    // Do stuff
}else{
    printf("Protocol %d. Message %d. Parse failed with error: %s", match.protocol, match.message, gps_parse_result_string[result]);
}
~~~

### Saving GPS data

The first thing is to open GPS files:

~~~C
const char* path = "/path/to/folder/";
gps_files_t gps_files;              // struct containing gps files
gps_open_files(&gps_files, path);   // open one csv file per message
gps_header_to_file(&gps_files);     // write csv headers
FILE* raw_log = fopen((std::string(path) + "gps.log").c_str(), "w");        // to log raw data from gps
~~~

To save raw data from the GPS:

~~~C
for(int i = 0; i < start_sequence_bytes; i++)
    fprintf(raw_log, "%c", start_sequence[i]);
for(int i = 0; i < line_bytes; i++)
    fprintf(raw_log, "%c", line[i]);
fprintf(raw_log, "\n");
~~~

To save also the parsed data in a CSV file:

~~~C
if(result == GPS_PARSE_RESULT_OK){  // only if parsing was successfull
    gps_to_file(&gps_files, &parsed_gps, &match);
}
~~~
