#include "gps.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

const char *gps_nmea_message_type_string(gps_nmea_message_type type) {
  switch (type) {
  case GPS_NMEA_TYPE_GGA:
    return "GGA";
    break;
  case GPS_NMEA_TYPE_VTG:
    return "VTG";
    break;
  case GPS_NMEA_TYPE_GSA:
    return "GSA";
    break;
  default:
    return "unkown NMEA message";
    break;
  }
}

const char *gps_ubx_message_type_string(gps_ubx_message_type type) {
  switch (type) {
  case GPS_UBX_TYPE_NAV_DOP:
    return "NAV_DOP";
    break;
  case GPS_UBX_TYPE_NAV_PVT:
    return "NAV_PVT";
    break;
  case GPS_UBX_TYPE_NAV_HPPOSECEF:
    return "NAV_HPPOSECEF";
    break;
  case GPS_UBX_TYPE_NAV_HPPOSLLH:
    return "NAV_HPPOSLLH";
    break;
  case GPS_UBX_TYPE_NAV_RELPOSNED:
    return "NAV_RELPOSNED";
    break;
  case GPS_UBX_TYPE_NAV_VELNED:
    return "NAV_VELNED";
    break;
  default:
    return "unknown UBX message";
    break;
  }
}

const char *gps_parse_result_string(gps_parse_result_t result) {
  switch (result) {
  case GPS_PARSE_RESULT_OK:
    return "OK";
    break;
  case GPS_PARSE_RESULT_NO_MATCH:
    return "NO_MATCH";
    break;
  case GPS_PARSE_RESULT_MESSAGE_EMPTY:
    return "MESSAGE_EMPTY";
    break;
  case GPS_PARSE_RESULT_MESAGE_LENGTH:
    return "MESAGE_LENGTH";
    break;
  case GPS_PARSE_RESULT_MESSAGE_UNDEFINED:
    return "MESSAGE_UNDEFINED";
    break;
  case GPS_PARSE_RESULT_FIELD_ERROR:
    return "FIELD_ERROR";
    break;
  case GPS_PARSE_RESULT_CHECKSUM:
    return "CHECKSUM";
    break;
  default:
    return "unknown PARSE RESULT";
    break;
  }
}

const char *gps_fix_state_string(uint8_t fix_state) {
  switch (fix_state) {
  case 0:
    return "FIX NOT AVAILABLE OR INVALID";
    break;
  case 1:
    return "Standard GPS (2D/3D)";
    break;
  case 2:
    return "DIFFERENTIAL GPS";
    break;
  case 3:
    return "RTK Fixed";
    break;
  case 4:
    return "RTK Float";
    break;
  case 5:
    return "DEAD RECKONING MODE FIX VALID";
    break;
  default:
    return "unknown FIX STATE";
    break;
  }
}
const char *gps_fix_mode_string(uint8_t fix_mode) {
  switch (fix_mode) {
  case 0:
    return "FIX NOT AVAILABLE";
    break;
  case 1:
    return "2D";
    break;
  case 2:
    return "3D";
    break;
  default:
    return "unknown FIX MODE";
    break;
  }
}

void rfc1145_checksum(int8_t *ch_a, int8_t *ch_b, const char *buffer, size_t size) {
  *ch_a = 0;
  *ch_b = 0;
  for (size_t i = 0; i < size; i++) {
    *ch_a = *ch_a + buffer[i];
    *ch_b = *ch_b + *ch_a;
  }
}

typedef struct gps_generic_ubx_message_t {
  char _class;
  char _id;
  uint16_t _size; // payload size
  int8_t _ck_a;
  int8_t _ck_b;
} gps_generic_ubx_message_t;

int gps_buffer_to_generic_ubx_message(gps_generic_ubx_message_t *message, const char buffer[GPS_MAX_LINE_SIZE]) {
  message->_class = buffer[0];
  message->_id = buffer[1];
  message->_size = buffer[2];
  message->_size += buffer[3] << 8;
  if (message->_size + 6 > GPS_MAX_LINE_SIZE) {
    printf("GPS: exceding max size\n");
    return -1;
  }
  message->_ck_a = buffer[4 + message->_size];
  message->_ck_b = buffer[4 + message->_size + 1];
  return 0;
}

int gps_ubx_check_checksum(const char *buffer) {
  int8_t ch_a, ch_b;
  gps_generic_ubx_message_t generic_msg;
  if (gps_buffer_to_generic_ubx_message(&generic_msg, buffer) == -1)
    return -1;
  rfc1145_checksum(&ch_a, &ch_b, buffer, generic_msg._size + 4);
  if (ch_a == generic_msg._ck_a && ch_b == generic_msg._ck_b)
    return 0;
  return -1;
}

int gps_match_message(gps_protocol_and_message *match, const char *buffer, gps_protocol_type protocol) {
  match->message = -1;
  if (protocol == GPS_PROTOCOL_TYPE_NMEA) {

    if (strncmp(buffer, "GGA", 3) == 0)
      match->message = GPS_NMEA_TYPE_GGA;
    else if (strncmp(buffer, "GSA", 3) == 0)
      match->message = GPS_NMEA_TYPE_GSA;
    else if (strncmp(buffer, "VTG", 3) == 0)
      match->message = GPS_NMEA_TYPE_VTG;

  } else if (protocol == GPS_PROTOCOL_TYPE_UBX) {
    if (buffer[0] == 1) { // match NAV message type
      for (int i = 0; i < GPS_UBX_TYPE_SIZE; i++) {
        if ((uint8_t)buffer[1] == gps_ubx_matches[i]) {
          match->message = i;
          break;
        }
      }
    }
  } else {
    protocol = GPS_PROTOCOL_TYPE_SIZE;
  }
  match->protocol = protocol;
  if (match->message == -1)
    return -1;

  return 0;
}

static inline double gps_deg_min_to_deg(double value) {
  int deg = floor(value / 100.0);
  return deg + (value - deg * 100.0) / 60.0;
}

gps_parse_result_t gps_parse_nmea_gga(gps_nmea_gga_t *data, const char *buffer) {
  char *ch;
  int index = -1;
  ch = strpbrk(buffer, ",");
  while (ch != NULL) {
    index++;
    switch (index) {
    case 1:
      memcpy(data->time, ch, 9);
      break;
    case 2:
      data->latitude = strtod(ch, NULL);
      data->latitude = gps_deg_min_to_deg(data->latitude);
      break;
    case 3:
      data->north_south = ch[0];
      break;
    case 4:
      data->longitude = strtod(ch, NULL);
      data->longitude = gps_deg_min_to_deg(data->longitude);
      break;
    case 5:
      data->east_ovest = ch[0];
      break;
    case 6:
      data->fix = strtol(ch, NULL, 10);
      break;
    case 7:
      data->satellites = strtol(ch, NULL, 10);
      break;
    case 8:
      data->horizontal_diluition_precision = strtod(ch, NULL);
      break;
    case 9:
      data->altitude = strtod(ch, NULL);
      break;
    case 11:
      // data->geoid_separation = strtod(ch, NULL);
      break;
    case 13:
      data->age_of_correction = strtod(ch, NULL);
      break;
      break;
    }
    ch = strpbrk(ch, ",");
    if (ch != NULL)
      ch += 1;
  }
  data->fix_state = gps_fix_state_string(data->fix);
  return GPS_PARSE_RESULT_OK;
}

gps_parse_result_t gps_parse_nmea_gsa(gps_nmea_gsa_t *data, const char *buffer) {
  char *ch;
  int index = -1;
  ch = strpbrk(buffer, ",");
  while (ch != NULL) {
    index++;
    switch (index) {
    case 1:
      data->mode = ch[0];
      break;
    case 2:
      break;
    case 15:
      data->position_diluition_precision = strtod(ch, NULL);
      break;
    case 16:
      data->horizontal_diluition_precision = strtod(ch, NULL);
      break;
    case 17:
      data->vertical_diluition_precision = strtod(ch, NULL);
      break;
    }
    ch = strpbrk(ch, ",");
    if (ch != NULL)
      ch += 1;
  }
  return GPS_PARSE_RESULT_OK;
}

gps_parse_result_t gps_parse_nmea_vtg(gps_nmea_vtg_t *data, const char *buffer) {
  char *ch;
  int index = -1;
  ch = strpbrk(buffer, ",");
  while (ch != NULL) {
    index++;
    switch (index) {
    case 1:
      data->course_over_ground_degrees = strtod(ch, NULL);
      break;
    case 3:
      data->course_over_ground_degrees_magnetic = strtod(ch, NULL);
      break;
    case 7:
      data->speed_kmh = strtod(ch, NULL);
      break;
    }
    ch = strpbrk(ch, ",");
    if (ch != NULL)
      ch += 1;
  }
  return GPS_PARSE_RESULT_OK;
}

void gps_parse_ubx_dop(gps_ubx_dop_t *data, uint8_t *buffer) {
  buffer += 4; // align to start of payload
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name)                           \
  data->name = ((struct_type)(*(original_type *)(buffer + byte_offset))) * scale + offset;
  GPS_UBX_DOP_FIELDS
#undef FIELD
}

void gps_parse_ubx_pvt(gps_ubx_pvt_t *data, uint8_t *buffer) {
  buffer += 4; // align to start of payload
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name)                           \
  data->name = ((struct_type)(*(original_type *)(buffer + byte_offset))) * scale + offset;
  GPS_UBX_PVT_FIELDS
#undef FIELD
}

void gps_parse_ubx_hpposecef(gps_ubx_hpposecef_t *data, uint8_t *buffer) {
  buffer += 4; // align to start of payload
#define FIELD( byte_offset, original_type, struct_type, formatter, scale, offset, unit, name)                           \
  data->name = ((struct_type)(*(original_type *)(buffer + byte_offset))) * scale + offset;
  GPS_UBX_HPPOSECEF_FIELDS
#undef FIELD
  data->ecefX += data->ecefXHp;
  data->ecefY += data->ecefYHp;
  data->ecefZ += data->ecefZHp;
}

void gps_parse_ubx_hpposllh(gps_ubx_hpposllh_t *data, uint8_t *buffer) {
  buffer += 4; // align to start of payload
#define FIELD( byte_offset, original_type, struct_type, formatter, scale, offset, unit, name)                           \
  data->name = ((struct_type)(*(original_type *)(buffer + byte_offset))) * scale + offset;
  GPS_UBX_HPPOSLLH_FIELDS
#undef FIELD
  // conversions
  data->lon += data->lonHp;
  data->lat += data->latHp;
  data->height += data->heightHp;
  data->hMSL += data->hMSLHp;
}

void gps_parse_ubx_relposned(gps_ubx_relposned_t *data, uint8_t *buffer) {
  buffer += 4; // align to start of payload
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name)                           \
  data->name = ((struct_type)(*(original_type *)(buffer + byte_offset))) * scale + offset;
  GPS_UBX_RELPOSNED_FIELDS
#undef FIELD
}

void gps_parse_ubx_velned(gps_ubx_velned_t *data, uint8_t *buffer) {
  buffer += 4; // align to start of payload
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name)                           \
  data->name = ((struct_type)(*(original_type *)(buffer + byte_offset))) * scale + offset;
  GPS_UBX_VELNED_FIELDS
#undef FIELD
}

void gps_nmea_fields_gga(FILE *out) {
  fprintf(out, "_timestamp,time,latitude,north_south,longitude,east_ovest,fix,"
               "satellites,horizontal_diluition_precision,fix_state,altitude,"
               "age_of_correction\n");
  fflush(out);
}

void gps_nmea_fields_vtg(FILE *out) {
  fprintf(out, "_timestamp,course_over_ground_degrees,course_over_ground_"
               "degrees_magnetic,speed_kmh\n");
  fflush(out);
}

void gps_nmea_fields_gsa(FILE *out) {
  fprintf(out, "_timestamp,mode,position_diluition_precision,horizontal_"
               "diluition_precision,vertical_diluition_precision\n");
  fflush(out);
}

void gps_ubx_fields_dop(FILE *out) {
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) "," #name
  fprintf(out, "_timestamp" GPS_UBX_DOP_FIELDS "\n");
#undef FIELD
  fflush(out);
}

void gps_ubx_fields_pvt(FILE *out) {
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) "," #name
  fprintf(out, "_timestamp" GPS_UBX_PVT_FIELDS "\n");
#undef FIELD
  fflush(out);
}

void gps_ubx_fields_hpposecef(FILE *out) {
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) "," #name
  fprintf(out, "_timestamp" GPS_UBX_HPPOSECEF_FIELDS "\n");
#undef FIELD
  fflush(out);
}

void gps_ubx_fields_hpposllh(FILE *out) {
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) "," #name
  fprintf(out, "_timestamp" GPS_UBX_HPPOSLLH_FIELDS "\n");
#undef FIELD
  fflush(out);
}

void gps_ubx_fields_relposned(FILE *out) {
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) "," #name
  fprintf(out, "_timestamp" GPS_UBX_RELPOSNED_FIELDS "\n");
#undef FIELD
  fflush(out);
}

void gps_ubx_fields_velned(FILE *out) {
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) "," #name
  fprintf(out, "_timestamp" GPS_UBX_VELNED_FIELDS "\n");
#undef FIELD
  fflush(out);
}

void gps_nmea_value_to_file_gga(FILE *out, gps_nmea_gga_t *data) {
  fprintf(out,
          "%" PRIu64 ",\"%s\""
          ",%.9f"
          ",%c"
          ",%.9f"
          ",%c"
          ",%" PRIu8 ",%" PRIu8 ",%.9f"
          ",\"%s\""
          ",%.9f"
          ",%.9f"
          "\n",
          data->_timestamp, data->time, data->latitude, data->north_south, data->longitude, data->east_ovest, data->fix,
          data->satellites, data->horizontal_diluition_precision, data->fix_state, data->altitude,
          data->age_of_correction);
  fflush(out);
}

void gps_nmea_value_to_file_vtg(FILE *out, gps_nmea_vtg_t *data) {
  fprintf(out,
          "%" PRIu64 ",%.9f"
          ",%.9f"
          ",%.9f"
          "\n",
          data->_timestamp, data->course_over_ground_degrees, data->course_over_ground_degrees_magnetic,
          data->speed_kmh);
  fflush(out);
}

void gps_nmea_value_to_file_gsa(FILE *out, gps_nmea_gsa_t *data) {
  fprintf(out,
          "%" PRIu64 ",%c"
          ",%.9f"
          ",%.9f"
          ",%.9f"
          "\n",
          data->_timestamp, data->mode, data->position_diluition_precision, data->horizontal_diluition_precision,
          data->vertical_diluition_precision);
  fflush(out);
}

void gps_ubx_value_to_file_dop(FILE *out, gps_ubx_dop_t *data) {
  fprintf(out,
          "%" PRIu64
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) "," formatter
              GPS_UBX_DOP_FIELDS
#undef FIELD
          "\n",
          data->_timestamp
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) , data->name
              GPS_UBX_DOP_FIELDS
#undef FIELD
  );
  fflush(out);
}

void gps_ubx_value_to_file_pvt(FILE *out, gps_ubx_pvt_t *data) {
  fprintf(out,
          "%" PRIu64
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) "," formatter
              GPS_UBX_PVT_FIELDS
#undef FIELD
          "\n",
          data->_timestamp
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) , data->name
              GPS_UBX_PVT_FIELDS
#undef FIELD
  );
  fflush(out);
}

void gps_ubx_value_to_file_hpposecef(FILE *out, gps_ubx_hpposecef_t *data) {
  fprintf(out,
          "%" PRIu64
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) "," formatter
              GPS_UBX_HPPOSECEF_FIELDS
#undef FIELD
          "\n",
          data->_timestamp
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) , data->name
              GPS_UBX_HPPOSECEF_FIELDS
#undef FIELD
  );
  fflush(out);
}

void gps_ubx_value_to_file_hpposllh(FILE *out, gps_ubx_hpposllh_t *data) {
  fprintf(out,
          "%" PRIu64
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) "," formatter
              GPS_UBX_HPPOSLLH_FIELDS
#undef FIELD
          "\n",
          data->_timestamp
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) , data->name
              GPS_UBX_HPPOSLLH_FIELDS
#undef FIELD
  );
  fflush(out);
}

void gps_ubx_value_to_file_relposned(FILE *out, gps_ubx_relposned_t *data) {
  fprintf(out,
          "%" PRIu64
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) "," formatter
              GPS_UBX_RELPOSNED_FIELDS
#undef FIELD
          "\n",
          data->_timestamp
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) , data->name
              GPS_UBX_RELPOSNED_FIELDS
#undef FIELD
  );
  fflush(out);
}

void gps_ubx_value_to_file_velned(FILE *out, gps_ubx_velned_t *data) {
  fprintf(out,
          "%" PRIu64
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) "," formatter
              GPS_UBX_VELNED_FIELDS
#undef FIELD
          "\n",
          data->_timestamp
#define FIELD(byte_offset, original_type, struct_type, formatter, scale, offset, unit, name) , data->name
              GPS_UBX_VELNED_FIELDS
#undef FIELD
  );
  fflush(out);
}

gps_parse_result_t gps_parse_buffer(gps_parsed_data_t *data, gps_protocol_and_message *match, const char *buffer,
                                    uint64_t timestamp) {
  if (match->protocol == GPS_PROTOCOL_TYPE_NMEA) {
    switch (match->message) {
    case GPS_NMEA_TYPE_GGA:
      data->gga._timestamp = timestamp;
      gps_parse_nmea_gga(&data->gga, buffer);
      break;
    case GPS_NMEA_TYPE_GSA:
      data->gsa._timestamp = timestamp;
      gps_parse_nmea_gsa(&data->gsa, buffer);
      break;
    case GPS_NMEA_TYPE_VTG:
      data->vtg._timestamp = timestamp;
      gps_parse_nmea_vtg(&data->vtg, buffer);
      break;
    default:
      return GPS_PARSE_RESULT_NO_MATCH;
      break;
    }
  } else if (match->protocol == GPS_PROTOCOL_TYPE_UBX) {
    if (gps_ubx_check_checksum(buffer) != 0)
      return GPS_PARSE_RESULT_CHECKSUM;
    switch (match->message) {
    case GPS_UBX_TYPE_NAV_DOP:
      data->dop._timestamp = timestamp;
      gps_parse_ubx_dop(&data->dop, (uint8_t *)buffer);
      break;
    case GPS_UBX_TYPE_NAV_PVT:
      data->pvt._timestamp = timestamp;
      gps_parse_ubx_pvt(&data->pvt, (uint8_t *)buffer);
      break;
    case GPS_UBX_TYPE_NAV_HPPOSECEF:
      data->hpposecef._timestamp = timestamp;
      gps_parse_ubx_hpposecef(&data->hpposecef, (uint8_t *)buffer);
      break;
    case GPS_UBX_TYPE_NAV_HPPOSLLH:
      data->hpposllh._timestamp = timestamp;
      gps_parse_ubx_hpposllh(&data->hpposllh, (uint8_t *)buffer);
      break;
    case GPS_UBX_TYPE_NAV_RELPOSNED:
      data->relposned._timestamp = timestamp;
      gps_parse_ubx_relposned(&data->relposned, (uint8_t *)buffer);
      break;
    case GPS_UBX_TYPE_NAV_VELNED:
      data->velned._timestamp = timestamp;
      gps_parse_ubx_velned(&data->velned, (uint8_t *)buffer);
      break;
    default:
      return GPS_PARSE_RESULT_NO_MATCH;
      break;
    }
  } else {
    return GPS_PARSE_RESULT_NO_MATCH;
  }
  return GPS_PARSE_RESULT_OK;
}

void gps_get_message_name(gps_protocol_and_message *match, char *buff) {
  switch (match->protocol) {
  case GPS_PROTOCOL_TYPE_NMEA:
    strcpy(buff, gps_nmea_message_type_string(match->message));
    break;
  case GPS_PROTOCOL_TYPE_UBX:
    strcpy(buff, gps_ubx_message_type_string(match->message));
    break;
  default:
    break;
  }
}

void gps_construct_filename(char *dest, const char *path, char *helper_buff, gps_protocol_and_message *match) {
  memset(dest, 0, strlen(dest));
  memset(helper_buff, 0, strlen(helper_buff));
  gps_get_message_name(match, helper_buff);
  strcat(dest, path);
  strcat(dest, "/GPS_");
  strcat(dest, helper_buff);
  strcat(dest, ".csv");
}

void gps_open_files(gps_files_t *files, const char *path) {
  const int message_size = 20;
  const int filepath_size = 100;
  char message[message_size];
  char filepath[filepath_size];
  gps_protocol_and_message match;
  match.protocol = GPS_PROTOCOL_TYPE_NMEA;
  for (int i = 0; i < GPS_NMEA_TYPE_SIZE; i++) {
    match.message = i;
    gps_construct_filename(filepath, path, message, &match);
    files->nmea[i] = fopen(filepath, "w");
  }
  match.protocol = GPS_PROTOCOL_TYPE_UBX;
  for (int i = 0; i < GPS_UBX_TYPE_SIZE; i++) {
    match.message = i;
    gps_construct_filename(filepath, path, message, &match);
    files->ubx[i] = fopen(filepath, "w");
  }
}

void gps_close_files(gps_files_t *files) {
  for (int i = 0; i < GPS_NMEA_TYPE_SIZE; i++) {
    fclose(files->nmea[i]);
  }
  for (int i = 0; i < GPS_UBX_TYPE_SIZE; i++) {
    fclose(files->ubx[i]);
  }
}

void gps_to_file(gps_files_t *files, gps_parsed_data_t *data, gps_protocol_and_message *match) {
  switch (match->protocol) {
  case GPS_PROTOCOL_TYPE_NMEA:
    switch (match->message) {
    case GPS_NMEA_TYPE_GGA:
      gps_nmea_value_to_file_gga(files->nmea[GPS_NMEA_TYPE_GGA], &data->gga);
      break;
    case GPS_NMEA_TYPE_VTG:
      gps_nmea_value_to_file_vtg(files->nmea[GPS_NMEA_TYPE_VTG], &data->vtg);
      break;
    case GPS_NMEA_TYPE_GSA:
      gps_nmea_value_to_file_gsa(files->nmea[GPS_NMEA_TYPE_GSA], &data->gsa);
      break;
    default:
      break;
    }
    break;
  case GPS_PROTOCOL_TYPE_UBX:
    switch (match->message) {
    case GPS_UBX_TYPE_NAV_DOP:
      gps_ubx_value_to_file_dop(files->ubx[GPS_UBX_TYPE_NAV_DOP], &data->dop);
      break;
    case GPS_UBX_TYPE_NAV_PVT:
      gps_ubx_value_to_file_pvt(files->ubx[GPS_UBX_TYPE_NAV_PVT], &data->pvt);
      break;
    case GPS_UBX_TYPE_NAV_HPPOSECEF:
      gps_ubx_value_to_file_hpposecef(files->ubx[GPS_UBX_TYPE_NAV_HPPOSECEF], &data->hpposecef);
      break;
    case GPS_UBX_TYPE_NAV_HPPOSLLH:
      gps_ubx_value_to_file_hpposllh(files->ubx[GPS_UBX_TYPE_NAV_HPPOSLLH], &data->hpposllh);
      break;
    case GPS_UBX_TYPE_NAV_RELPOSNED:
      gps_ubx_value_to_file_relposned(files->ubx[GPS_UBX_TYPE_NAV_RELPOSNED], &data->relposned);
      break;
    case GPS_UBX_TYPE_NAV_VELNED:
      gps_ubx_value_to_file_velned(files->ubx[GPS_UBX_TYPE_NAV_VELNED], &data->velned);
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}

void gps_header_to_file(gps_files_t *files) {
  // NMEA
  gps_nmea_fields_gga(files->nmea[GPS_NMEA_TYPE_GGA]);
  gps_nmea_fields_vtg(files->nmea[GPS_NMEA_TYPE_VTG]);
  gps_nmea_fields_gsa(files->nmea[GPS_NMEA_TYPE_GSA]);
  // UBX
  gps_ubx_fields_dop(files->ubx[GPS_UBX_TYPE_NAV_DOP]);
  gps_ubx_fields_pvt(files->ubx[GPS_UBX_TYPE_NAV_PVT]);
  gps_ubx_fields_hpposecef(files->ubx[GPS_UBX_TYPE_NAV_HPPOSECEF]);
  gps_ubx_fields_hpposllh(files->ubx[GPS_UBX_TYPE_NAV_HPPOSLLH]);
  gps_ubx_fields_relposned(files->ubx[GPS_UBX_TYPE_NAV_RELPOSNED]);
  gps_ubx_fields_velned(files->ubx[GPS_UBX_TYPE_NAV_VELNED]);
}