#pragma once

#include "gps.pb.h"
#include <string>
#include <unordered_map>
extern "C" {
#include "gps.h"
}

#include "common_types.h"

static std::unordered_map<std::string, uint64_t> timers_gps;

typedef struct gps_proto_pack {
  // NMEA
  circular_buffer<gps_nmea_gga_t> gga;
  circular_buffer<gps_nmea_vtg_t> vtg;
  circular_buffer<gps_nmea_gsa_t> gsa;
  // UBX
  circular_buffer<gps_ubx_dop_t> dop;
  circular_buffer<gps_ubx_pvt_t> pvt;
  circular_buffer<gps_ubx_hpposecef_t> hpposecef;
  circular_buffer<gps_ubx_hpposllh_t> hpposllh;
  circular_buffer<gps_ubx_relposned_t> relposned;
  circular_buffer<gps_ubx_velned_t> velened;

  std::unordered_map<std::string, uint64_t> timers;
} gps_proto_pack;

void gps_proto_serialize_from_match(gps_protocol_and_message &match, gps::GpsPack *proto, gps_parsed_data_t *data,
                                    uint64_t &timestamp, uint64_t downsample_rate);
void gps_proto_deserialize(gps::GpsPack *proto, network_enums *net_enums, network_signals *net_signals,
                           network_strings *net_strings, uint64_t resample_us);

void gps_serialize_gga(gps::GGA *proto, gps_nmea_gga_t *data);
void gps_serialize_vtg(gps::VTG *proto, gps_nmea_vtg_t *data);
void gps_serialize_gsa(gps::GSA *proto, gps_nmea_gsa_t *data);

void gps_serialize_dop(gps::NAV_DOP *proto, gps_ubx_dop_t *data);
void gps_serialize_pvt(gps::NAV_PVT *proto, gps_ubx_pvt_t *data);
void gps_serialize_hpposecef(gps::NAV_HPPOSECEF *proto, gps_ubx_hpposecef_t *data);
void gps_serialize_hpposllh(gps::NAV_HPPOSLLH *proto, gps_ubx_hpposllh_t *data);
void gps_serialize_relposned(gps::NAV_RELPOSNED *proto, gps_ubx_relposned_t *data);
void gps_serialize_velned(gps::NAV_VELNED *proto, gps_ubx_velned_t *data);