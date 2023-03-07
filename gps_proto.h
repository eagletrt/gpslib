#pragma once

#include <unordered_map>
#include <string>
#include "gps.pb.h"
extern "C" {
    #include "gps.h"
}

#include "common_types.h"

#ifndef CANLIB_CIRCULAR_BUFFER_SIZE
#define CANLIB_CIRCULAR_BUFFER_SIZE 2000
#endif//CANLIB_CIRCULAR_BUFFER_SIZE
#include "../can/proto/primary/cpp/mapping.h"

static std::unordered_map<std::string, uint64_t> timers_gps;

typedef struct gps_proto_pack{
    // NMEA
    canlib_circular_buffer<gps_nmea_gga_t, CANLIB_CIRCULAR_BUFFER_SIZE> gga;
    canlib_circular_buffer<gps_nmea_vtg_t, CANLIB_CIRCULAR_BUFFER_SIZE> vtg;
    canlib_circular_buffer<gps_nmea_gsa_t, CANLIB_CIRCULAR_BUFFER_SIZE> gsa;
    // UBX
    canlib_circular_buffer<gps_ubx_dop_t, CANLIB_CIRCULAR_BUFFER_SIZE> dop;
    canlib_circular_buffer<gps_ubx_pvt_t, CANLIB_CIRCULAR_BUFFER_SIZE> pvt;
    canlib_circular_buffer<gps_ubx_hpposecef_t, CANLIB_CIRCULAR_BUFFER_SIZE> hpposecef;
    canlib_circular_buffer<gps_ubx_hpposllh_t, CANLIB_CIRCULAR_BUFFER_SIZE> hpposllh;
    std::unordered_map<std::string, uint64_t> timers;
}gps_proto_pack;

void gps_mapping_adaptor_construct(gps_proto_pack& pack, mapping_adaptor& mapping_map);
void gps_proto_serialize_from_match(gps_protocol_and_message& match, gps::GpsPack* proto, gps_parsed_data_t* data, uint64_t &timestamp, uint64_t downsample_rate);
void gps_proto_deserialize(gps::GpsPack *proto, network_enums *net_enums,
                           network_signals *net_signals,
                           network_strings *net_strings, uint64_t resample_us);

void gps_serialize_gga(gps::GGA* proto, gps_nmea_gga_t* data);
void gps_serialize_vtg(gps::VTG* proto, gps_nmea_vtg_t* data);
void gps_serialize_gsa(gps::GSA* proto, gps_nmea_gsa_t* data);

void gps_serialize_dop(gps::NAV_DOP* proto, gps_ubx_dop_t* data);
void gps_serialize_pvt(gps::NAV_PVT* proto, gps_ubx_pvt_t* data);
void gps_serialize_hpposecef(gps::NAV_HPPOSECEF* proto, gps_ubx_hpposecef_t* data);
void gps_serialize_hpposllh(gps::NAV_HPPOSLLH* proto, gps_ubx_hpposllh_t* data);