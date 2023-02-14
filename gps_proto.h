#pragma once

#include "gps.pb.h"
extern "C" {
#include "gps.h"
}

#include "common_types.h"

// typedef struct gps_proto_pack{
//     // NMEA
//     canlib_circular_buffer<gps_nmea_gga_t, 1000> gga;
//     canlib_circular_buffer<gps_nmea_vtg_t, 1000> vtg;
//     canlib_circular_buffer<gps_nmea_gsa_t, 1000> gsa;
//     // UBX
//     canlib_circular_buffer<gps_ubx_dop_t, 1000> dop;
//     canlib_circular_buffer<gps_ubx_pvt_t, 1000> pvt;
//     canlib_circular_buffer<gps_ubx_hpposecef_t, 1000> hpposecef;
//     canlib_circular_buffer<gps_ubx_hpposllh_t, 1000> hpposllh;
// }gps_proto_pack;

void gps_proto_serialize_from_match(gps_protocol_and_message &match,
                                    gps::GpsPack *proto,
                                    gps_parsed_data_t *data);
void gps_proto_deserialize(gps::GpsPack *proto, network_enums *net_enums,
                           network_signals *net_signals,
                           network_strings *net_strings, uint64_t resample_us);

void gps_serialize_gga(gps::GGA *proto, gps_nmea_gga_t *data);
void gps_serialize_vtg(gps::VTG *proto, gps_nmea_vtg_t *data);
void gps_serialize_gsa(gps::GSA *proto, gps_nmea_gsa_t *data);

void gps_serialize_dop(gps::NAV_DOP *proto, gps_ubx_dop_t *data);
void gps_serialize_pvt(gps::NAV_PVT *proto, gps_ubx_pvt_t *data);
void gps_serialize_hpposecef(gps::NAV_HPPOSECEF *proto,
                             gps_ubx_hpposecef_t *data);
void gps_serialize_hpposllh(gps::NAV_HPPOSLLH *proto, gps_ubx_hpposllh_t *data);