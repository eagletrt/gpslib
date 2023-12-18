#ifndef GPSLIB_COMMON_TYPES_H
#define GPSLIB_COMMON_TYPES_H

#ifndef CANLIB_PROTO_INTERFACE_TYPES
#define CANLIB_PROTO_INTERFACE_TYPES

#include <string>
#include <unordered_map>

#include "Utils/circular_buffer.h"
/**
 *  Use network_<> to get all the values from the protobuffer.
 *  Every network can be consensed into one network_<> as all the
 *  messages names are unique.
 **/

typedef std::string field_name;
typedef std::string messages_name;
typedef circular_buffer<double> double_buffer;
typedef circular_buffer<uint64_t> uint64_buffer;
typedef circular_buffer<std::string> string_buffer;

// structure contains all the messages with a enum value associated
// the type is unified to uint64_t
typedef std::unordered_map<field_name, uint64_buffer> message_enums;
typedef std::unordered_map<messages_name, message_enums> network_enums;

// structure contains all the messages with a signal associated
// the type is unified to double
typedef std::unordered_map<field_name, double_buffer> message_signals;
typedef std::unordered_map<messages_name, message_signals> network_signals;

// structure contains all the messages with a string associated
// the type is unified to string
typedef std::unordered_map<field_name, string_buffer> message_strings;
typedef std::unordered_map<messages_name, message_strings> network_strings;

#endif // CANLIB_PROTO_INTERFACE_TYPES

#endif // GPSLIB_COMMON_TYPES_H