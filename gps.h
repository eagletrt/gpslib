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
  // NAV-VELNED
  GPS_UBX_TYPE_SIZE
} gps_ubx_message_type;

static const uint8_t gps_ubx_matches[GPS_UBX_TYPE_SIZE] = {
    0x04, // DOP
    0x07, // PVT
    0x13, // HPPOSECEF
    0x14, // HPPOSLLH
    0x3c  // RELPOSNED
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

// Dilution of precision
typedef struct gps_ubx_dop_t {
  uint64_t _timestamp;
  uint32_t iTOW; // [ms]
  float gDOP;    // [*0.01] uint16_t
  float pDOP;    // [*0.01] uint16_t
  float tDOP;    // [*0.01] uint16_t
  float vDOP;    // [*0.01] uint16_t
  float hDOP;    // [*0.01] uint16_t
  float nDOP;    // [*0.01] uint16_t
  float eDOP;    // [*0.01] uint16_t
} gps_ubx_dop_t;

// Navigation Position Velocity Time Solution
typedef struct gps_ubx_pvt_t {
  uint64_t _timestamp;
  uint32_t iTOW;       // [ms]
  uint16_t year;       // [y]
  uint8_t month;       // [month]
  uint8_t day;         // [day]
  uint8_t hour;        // [hour]
  uint8_t min;         // [min]
  uint8_t sec;         // [second]
  uint8_t valid;       // []
  uint32_t tAcc;       // [ns]
  int32_t nano;        // [ns]
  uint8_t fixType;     // []
  uint8_t flags;       // []
  uint8_t flags2;      // []
  uint8_t numSV;       // []
  double lon;          // [deg*e-7] int32_t
  double lat;          // [deg*e-7] int32_t
  int32_t height;      // [mm]
  int32_t hMSL;        // [mm]
  uint32_t hAcc;       // [mm]
  uint32_t vAcc;       // [mm]
  int32_t velN;        // [mm/s]
  int32_t velE;        // [mm/s]
  int32_t velD;        // [mm/s]
  int32_t gSpeed;      // [mm/s]
  double headMot;      // [deg*e-5] int32_t
  uint32_t sAcc;       // [mm/s]
  double headAcc;      // [deg*e-5] uint32_t
  uint16_t pDOP;       // [*0.01]
  uint8_t reserved[6]; // []
  double headVeh;      // [deg*e-5] int32_t
  double magDec;       // [deg*e-2] int16_t
  double magAcc;       // [deg*e-2] uint16_t
} gps_ubx_pvt_t;

// High Precision Position Solution in ECEF
typedef struct gps_ubx_hpposecef_t {
  uint64_t _timestamp;
  uint8_t version;
  uint8_t reserved[3];
  uint32_t iTOW;     // [ms]
  double ecefX;      // [cm] int32_t
  double ecefY;      // [cm] int32_t
  double ecefZ;      // [cm] int32_t
  int8_t ecefXHp;    // [mm*0.1]
  int8_t ecefYHp;    // [mm*0.1]
  int8_t ecefZHp;    // [mm*0.1]
  uint8_t reserved2; // []
  float pAcc;        // [mm*0.1] uint32_t
} gps_ubx_hpposecef_t;

// High Precision Geodetic Position Solution
typedef struct gps_ubx_hpposllh_t {
  uint64_t _timestamp;
  uint8_t version;
  uint8_t reserved[3];
  uint32_t iTOW;
  double lon;      // [deg*e-7] int32_t
  double lat;      // [deg*e-7] int32_t
  float height;    // [mm] int32_t
  float hMSL;      // [mm] int32_t
  int8_t lonHp;    // [deg*e-9]
  int8_t latHp;    // [deg*e-9]
  int8_t heightHp; // [mm*0.1]
  int8_t hMSLHp;   // [mm*0.1]
  uint32_t hAcc;   // [mm*0.1]
  uint32_t vAcc;   // [mm*0.1]
} gps_ubx_hpposllh_t;

// FIELD(byte_offset, original_type, struct_type, scale, offset, unit, name)
#define GPS_UBX_RELPOSNED_FIELDS                                               \
  FIELD( 0, uint8_t, uint8_t, "%" PRIu8, 1, 0, "", version)                     \
  FIELD( 2, uint16_t, uint16_t, "%" PRIu16, 1, 0, "", refStationId)             \
  FIELD( 4, uint64_t, uint64_t, "%" PRIu64, 1, 0, "ms", iTOW)                   \
  FIELD( 8, int64_t, double, "%f", 0.01, 0, "m", relPosN)                       \
  FIELD(12, int64_t, double, "%f", 0.01, 0, "m", relPosE)                      \
  FIELD(16, int64_t, double, "%f", 0.01, 0, "m", relPosD)                      \
  FIELD(20, int64_t, double, "%f", 0.01, 0, "m", relPosLength)                 \
  FIELD(24, int64_t, double, "%f", 1e-5, 0, "deg", relPosHeading)              \
  FIELD(32, int8_t, double, "%f", 0.1, 0, "mm", relPosHPN)                     \
  FIELD(33, int8_t, double, "%f", 0.1, 0, "mm", relPosHPE)                     \
  FIELD(34, int8_t, double, "%f", 0.1, 0, "mm", relPosHPD)                     \
  FIELD(35, int8_t, double, "%f", 0.1, 0, "mm", relPosHPLength)                \
  FIELD(36, uint64_t, double, "%f", 0.1, 0, "mm", accN)                        \
  FIELD(40, uint64_t, double, "%f", 0.1, 0, "mm", accE)                        \
  FIELD(44, uint64_t, double, "%f", 0.1, 0, "mm", accD)                        \
  FIELD(48, uint64_t, double, "%f", 0.1, 0, "mm", accLength)                   \
  FIELD(52, uint64_t, double, "%f", 1e-5, 0, "deg", accHeading)                \
  FIELD(60, uint64_t, uint64_t, "%" PRIu64, 1, 0, "flags", flags)

typedef struct gps_ubx_relposned_t {
  uint64_t _timestamp;

#define FIELD(byte_offset, original_type, struct_type, formatter, scale,       \
              offset, unit, name)                                              \
  struct_type name;
  GPS_UBX_RELPOSNED_FIELDS
#undef FIELD

} gps_ubx_relposned_t;

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
int gps_match_message(gps_protocol_and_message *match, const char *buffer,
                      gps_protocol_type protocol);
gps_parse_result_t gps_parse_buffer(gps_parsed_data_t *data,
                                    gps_protocol_and_message *match,
                                    const char *buffer, uint64_t timestamp);

typedef struct gps_files_t {
  FILE *nmea[GPS_NMEA_TYPE_SIZE];
  FILE *ubx[GPS_UBX_TYPE_SIZE];
} gps_files_t;

void gps_get_message_name(gps_protocol_and_message *match, char *buff);
void gps_open_files(gps_files_t *files, const char *path);
void gps_close_files(gps_files_t *files);

void gps_to_file(gps_files_t *files, gps_parsed_data_t *data,
                 gps_protocol_and_message *match);
void gps_header_to_file(gps_files_t *files);