#pragma once

#include <inttypes.h>
#include <stdio.h>

#define GPS_MAX_LINE_SIZE 300
#define GPS_MAX_START_SEQUENCE_SIZE 3 + 1
#define GPS_UBX_SYNC_FIRST_BYTE (0xb5)
#define GPS_UBX_SYNC_SECOND_BYTE (0x62)
#define GPS_NMEA_SYNC_FIRST_BYTE (0x24)
#define GPS_NMEA_SYNC_SECOND_BYTE1 (0x47)
#define GPS_NMEA_SYNC_SECOND_BYTE2 (0x50)

typedef enum gps_protocol_type {
  GPS_PROTOCOL_TYPE_NMEA,
  GPS_PROTOCOL_TYPE_UBX,
  GPS_PROTOCOL_TYPE_SIZE
} gps_protocol_type;

typedef enum gps_nmea_message_type {
  GPS_NMEA_TYPE_GGA,
  GPS_NMEA_TYPE_VTG,
  GPS_NMEA_TYPE_GSA,
  GPS_NMEA_TYPE_SIZE,
} gps_nmea_message_type;

const char *gps_nmea_message_type_string(gps_nmea_message_type type);

typedef enum gps_ubx_message_type {
  GPS_UBX_TYPE_NAV_DOP,
  GPS_UBX_TYPE_NAV_PVT,
  GPS_UBX_TYPE_NAV_HPPOSECEF,
  GPS_UBX_TYPE_NAV_HPPOSLLH,
  GPS_UBX_TYPE_NAV_RELPOSNED,
  GPS_UBX_TYPE_NAV_VELNED,
  GPS_UBX_TYPE_SIZE
} gps_ubx_message_type;

static const uint8_t gps_ubx_matches[GPS_UBX_TYPE_SIZE] = {
    0x04, // DOP
    0x07, // PVT
    0x13, // HPPOSECEF
    0x14, // HPPOSLLH
    0x3c, // RELPOSNED
    0x12 // VELNED
};

const char *gps_ubx_message_type_string(gps_ubx_message_type type);

typedef struct gps_protocol_and_message {
  gps_protocol_type protocol;
  int message; // based on protocol it is gps_nmea_message_type or
               // gps_ubx_message_type
} gps_protocol_and_message;

typedef struct gps_nmea_gga_t {
  uint64_t _timestamp;
  char time[9];
  double latitude;
  char north_south;
  double longitude;
  char east_ovest;
  uint8_t fix;
  uint8_t satellites;
  double horizontal_diluition_precision;
  const char *fix_state;
  double altitude;
  double age_of_correction;
} gps_nmea_gga_t;

typedef struct gps_nmea_vtg_t {
  uint64_t _timestamp;
  double course_over_ground_degrees;
  double course_over_ground_degrees_magnetic;
  double speed_kmh;
} gps_nmea_vtg_t;

typedef struct gps_nmea_gsa_t {
  uint64_t _timestamp;
  char mode;
  double position_diluition_precision;
  double horizontal_diluition_precision;
  double vertical_diluition_precision;
} gps_nmea_gsa_t;

// FIELD(byte_offset, original_type, struct_type, scale, offset, unit, name)
#define GPS_UBX_DOP_FIELDS                                                                                             \
  FIELD(0, uint32_t, uint32_t, "%" PRIu32, 1, 0, "ms", iTOW)                                                           \
  FIELD(4, uint16_t, double, "%f", 0.01, 0, "", gDOP)                                                                  \
  FIELD(6, uint16_t, double, "%f", 0.01, 0, "", pDOP)                                                                  \
  FIELD(8, uint16_t, double, "%f", 0.01, 0, "", tDOP)                                                                  \
  FIELD(10, uint16_t, double, "%f", 0.01, 0, "", vDOP)                                                                 \
  FIELD(12, uint16_t, double, "%f", 0.01, 0, "", hDOP)                                                                 \
  FIELD(14, uint16_t, double, "%f", 0.01, 0, "", nDOP)                                                                 \
  FIELD(16, uint16_t, double, "%f", 0.01, 0, "", eDOP)

// FIELD(byte_offset, original_type, struct_type, scale, offset, unit, name)
#define GPS_UBX_PVT_FIELDS                                                                                             \
  FIELD(0, uint32_t, uint32_t, "%" PRIu32, 1, 0, "ms", iTOW)                                                           \
  FIELD(4, uint16_t, uint16_t, "%" PRIu16, 1, 0, "y", year)                                                            \
  FIELD(6, uint8_t, uint8_t, "%" PRIu8, 1, 0, "month", month)                                                          \
  FIELD(7, uint8_t, uint8_t, "%" PRIu8, 1, 0, "day", day)                                                              \
  FIELD(8, uint8_t, uint8_t, "%" PRIu8, 1, 0, "hour", hour)                                                            \
  FIELD(9, uint8_t, uint8_t, "%" PRIu8, 1, 0, "min", min)                                                              \
  FIELD(10, uint8_t, uint8_t, "%" PRIu8, 1, 0, "sec", sec)                                                             \
  FIELD(11, uint8_t, uint8_t, "%" PRIu8, 1, 0, "", valid)                                                              \
  FIELD(12, uint32_t, uint32_t, "%" PRIu32, 1, 0, "ns", tAcc)                                                          \
  FIELD(16, int32_t, int32_t, "%" PRIi32, 1, 0, "ns", nano)                                                            \
  FIELD(20, uint8_t, uint8_t, "%" PRIu8, 1, 0, "", fixType)                                                            \
  FIELD(21, uint8_t, uint8_t, "%" PRIu8, 1, 0, "", flags)                                                              \
  FIELD(22, uint8_t, uint8_t, "%" PRIu8, 1, 0, "", flags2)                                                             \
  FIELD(23, uint8_t, uint8_t, "%" PRIu8, 1, 0, "", numSV)                                                              \
  FIELD(24, int32_t, double, "%f", 1e-7, 0, "deg", lon)                                                                \
  FIELD(28, int32_t, double, "%f", 1e-7, 0, "deg", lat)                                                                \
  FIELD(32, int32_t, double, "%f", 1e-3, 0, "m", height)                                                               \
  FIELD(36, int32_t, double, "%f", 1e-3, 0, "m", hMSL)                                                                 \
  FIELD(40, uint32_t, double, "%f", 1e-3, 0, "m", hAcc)                                                                \
  FIELD(44, uint32_t, double, "%f", 1e-3, 0, "m", vAcc)                                                                \
  FIELD(48, int32_t, double, "%f", 1e-3, 0, "m/s", velN)                                                               \
  FIELD(52, int32_t, double, "%f", 1e-3, 0, "m/s", velE)                                                               \
  FIELD(56, int32_t, double, "%f", 1e-3, 0, "m/s", velD)                                                               \
  FIELD(60, int32_t, double, "%f", 1e-3, 0, "m/s", gSpeed)                                                             \
  FIELD(64, int32_t, double, "%f", 1e-5, 0, "deg", headMot)                                                            \
  FIELD(68, uint32_t, double, "%f", 1e-3, 0, "m/s", sAcc)                                                              \
  FIELD(72, uint32_t, double, "%f", 1e-5, 0, "deg", headAcc)                                                           \
  FIELD(76, uint16_t, double, "%f", 0.01, 0, "", pDOP)                                                                 \
  FIELD(84, int32_t, double, "%f", 1e-5, 0, "deg", headVeh)                                                            \
  FIELD(88, int32_t, double, "%f", 1e-2, 0, "deg", magDec)                                                             \
  FIELD(90, uint32_t, double, "%f", 1e-2, 0, "deg", magAcc)

// FIELD(byte_offset, original_type, struct_type, scale, offset, unit, name)
#define GPS_UBX_HPPOSECEF_FIELDS                                                                                       \
  FIELD(0, uint8_t, uint8_t, "%" PRIu8, 1, 0, "", version)                                                             \
  FIELD(4, uint32_t, uint32_t, "%" PRIu32, 1, 0, "ms", iTOW)                                                           \
  FIELD(8, int32_t, double, "%f", 1e-2, 0, "m", ecefX)                                                                 \
  FIELD(12, int32_t, double, "%f", 1e-2, 0, "m", ecefY)                                                                \
  FIELD(16, int32_t, double, "%f", 1e-2, 0, "m", ecefZ)                                                                \
  FIELD(20, int8_t, double, "%f", 1e-4, 0, "m", ecefXHp)                                                               \
  FIELD(21, int8_t, double, "%f", 1e-4, 0, "m", ecefYHp)                                                               \
  FIELD(22, int8_t, double, "%f", 1e-4, 0, "m", ecefZHp)                                                               \
  FIELD(24, uint32_t, uint32_t, "%" PRIu32, 1e-4, 0, "m", pAcc)

// FIELD(byte_offset, original_type, struct_type, scale, offset, unit, name)
#define GPS_UBX_HPPOSLLH_FIELDS                                                                                        \
  FIELD(0, uint8_t, uint8_t, "%" PRIu8, 1, 0, "", version)                                                             \
  FIELD(4, uint32_t, uint32_t, "%" PRIu32, 1, 0, "ms", iTOW)                                                           \
  FIELD(8, int32_t, double, "%f", 1e-7, 0, "deg", lon)                                                                 \
  FIELD(12, int32_t, double, "%f", 1e-7, 0, "deg", lat)                                                                \
  FIELD(16, int32_t, double, "%f", 1e-3, 0, "m", height)                                                               \
  FIELD(20, int32_t, double, "%f", 1e-3, 0, "m", hMSL)                                                                 \
  FIELD(24, int8_t, double, "%f", 1e-9, 0, "deg", lonHp)                                                               \
  FIELD(25, int8_t, double, "%f", 1e-9, 0, "deg", latHp)                                                               \
  FIELD(26, int8_t, double, "%f", 1e-4, 0, "m", heightHp)                                                              \
  FIELD(27, int8_t, double, "%f", 1e-4, 0, "m", hMSLHp)                                                                \
  FIELD(28, uint32_t, double, "%f", 1e-4, 0, "m", hAcc)                                                                \
  FIELD(32, uint32_t, double, "%f", 1e-4, 0, "m", vAcc)

// FIELD(byte_offset, original_type, struct_type, scale, offset, unit, name)
#define GPS_UBX_RELPOSNED_FIELDS                                                                                       \
  FIELD(0, uint8_t,   uint8_t, "%" PRIu8, 1, 0, "", version)                                                             \
  FIELD(2, uint16_t,  uint16_t, "%" PRIu16, 1, 0, "", refStationId)                                                     \
  FIELD(4, uint32_t,  uint32_t, "%" PRIu32, 1, 0, "ms", iTOW)                                                           \
  FIELD(8, int32_t,   double, "%f", 0.01, 0, "m", relPosN)                                                               \
  FIELD(12, int32_t,  double, "%f", 0.01, 0, "m", relPosE)                                                              \
  FIELD(16, int32_t,  double, "%f", 0.01, 0, "m", relPosD)                                                              \
  FIELD(20, int32_t,  double, "%f", 0.01, 0, "m", relPosLength)                                                         \
  FIELD(24, int32_t,  double, "%f", 1e-5, 0, "deg", relPosHeading)                                                      \
  FIELD(32, int8_t,   double, "%f", 1e-4, 0, "m", relPosHPN)                                                             \
  FIELD(33, int8_t,   double, "%f", 1e-4, 0, "m", relPosHPE)                                                             \
  FIELD(34, int8_t,   double, "%f", 1e-4, 0, "m", relPosHPD)                                                             \
  FIELD(35, int8_t,   double, "%f", 1e-4, 0, "m", relPosHPLength)                                                        \
  FIELD(36, uint32_t, double, "%f", 1e-4, 0, "m", accN)                                                                \
  FIELD(40, uint32_t, double, "%f", 1e-4, 0, "m", accE)                                                                \
  FIELD(44, uint32_t, double, "%f", 1e-4, 0, "m", accD)                                                                \
  FIELD(48, uint32_t, double, "%f", 1e-4, 0, "m", accLength)                                                           \
  FIELD(52, uint32_t, double, "%f", 1e-5, 0, "deg", accHeading)                                                        \
  FIELD(60, uint32_t, uint32_t, "%" PRIu32, 1, 0, "flags", flags)

#define GPS_UBX_VELNED_FIELDS\
  FIELD(0, uint32_t, uint32_t, "%" PRIu32, 1, 0, "ms", iTOW)\
  FIELD(4, int32_t, double, "%f", 1e-2, 0, "m/s", velN)\
  FIELD(8, int32_t, double, "%f", 1e-2, 0, "m/s", velE)\
  FIELD(12, int32_t, double, "%f", 1e-2, 0, "m/s", velD)\
  FIELD(16, uint32_t, double, "%f", 1e-2, 0, "m/s", speed)\
  FIELD(20, uint32_t, double, "%f", 1e-5, 0, "deg", gSpeed)\
  FIELD(24, int32_t, double, "%f", 1e-5, 0, "deg", heading)\
  FIELD(28, uint32_t, double, "%f", 1e-2, 0, "m/s", sAcc)\
  FIELD(32, uint32_t, double, "%f", 1e-5, 0, "deg", cAcc)

// Dilution of precision
typedef struct gps_ubx_dop_t {
  uint64_t _timestamp;
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) struct_type name;
  GPS_UBX_DOP_FIELDS
#undef FIELD
} gps_ubx_dop_t;
// Navigation Position Velocity Time Solution
typedef struct gps_ubx_pvt_t {
  uint64_t _timestamp;
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) struct_type name;
  GPS_UBX_PVT_FIELDS
#undef FIELD
} gps_ubx_pvt_t;

// High Precision Position Solution in ECEF
typedef struct gps_ubx_hpposecef_t {
  uint64_t _timestamp;
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) struct_type name;
  GPS_UBX_HPPOSECEF_FIELDS
#undef FIELD
} gps_ubx_hpposecef_t;

// High Precision Geodetic Position Solution
typedef struct gps_ubx_hpposllh_t {
  uint64_t _timestamp;
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) struct_type name;
  GPS_UBX_HPPOSLLH_FIELDS
#undef FIELD
} gps_ubx_hpposllh_t;

typedef struct gps_ubx_relposned_t {
  uint64_t _timestamp;
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) struct_type name;
  GPS_UBX_RELPOSNED_FIELDS
#undef FIELD
} gps_ubx_relposned_t;

typedef struct gps_ubx_velned_t {
  uint64_t _timestamp;
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) struct_type name;
  GPS_UBX_VELNED_FIELDS
#undef FIELD
} gps_ubx_velned_t;

typedef struct gps_parsed_data_t {
  // NMEA
  gps_nmea_gga_t gga;
  gps_nmea_vtg_t vtg;
  gps_nmea_gsa_t gsa;
  // UBX
  gps_ubx_dop_t dop;
  gps_ubx_pvt_t pvt;
  gps_ubx_hpposecef_t hpposecef;
  gps_ubx_hpposllh_t hpposllh;
  gps_ubx_relposned_t relposned;
  gps_ubx_velned_t velned;
} gps_parsed_data_t;

typedef enum gps_parse_result_t {
  GPS_PARSE_RESULT_OK,
  GPS_PARSE_RESULT_NO_MATCH,
  GPS_PARSE_RESULT_MESSAGE_EMPTY,
  GPS_PARSE_RESULT_MESAGE_LENGTH,
  GPS_PARSE_RESULT_MESSAGE_UNDEFINED,
  GPS_PARSE_RESULT_FIELD_ERROR,
  GPS_PARSE_RESULT_CHECKSUM,
  GPS_PARSE_RESULT_SIZE
} gps_parse_result_t;

const char *gps_parse_result_string(gps_parse_result_t result);
const char *gps_fix_state_string(uint8_t fix_state);
const char *gps_fix_mode_string(uint8_t fix_mode);

// given buffer and protocol, match the type of message
// 0 if ok, -1 if error
int gps_match_message(gps_protocol_and_message *match, const char *buffer, gps_protocol_type protocol);
gps_parse_result_t gps_parse_buffer(gps_parsed_data_t *data, gps_protocol_and_message *match, const char *buffer,
                                    uint64_t timestamp);

typedef struct gps_files_t {
  FILE *nmea[GPS_NMEA_TYPE_SIZE];
  FILE *ubx[GPS_UBX_TYPE_SIZE];
} gps_files_t;

void gps_get_message_name(gps_protocol_and_message *match, char *buff);
int gps_open_files(gps_files_t *files, const char *path);
void gps_close_files(gps_files_t *files);

void gps_to_file(gps_files_t *files, gps_parsed_data_t *data, gps_protocol_and_message *match);
void gps_header_to_file(gps_files_t *files);