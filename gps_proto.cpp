#include "gps_proto.h"
#include <string.h>
#include <functional>

void gps_proto_serialize_from_match(gps_protocol_and_message& match, gps::GpsPack* proto, gps_parsed_data_t* data, uint64_t &timestamp, uint64_t downsample_rate){
    switch (match.protocol)
    {
        case GPS_PROTOCOL_TYPE_NMEA:
            switch (match.message)
            {
                case GPS_NMEA_TYPE_GGA:
                    if((1.0e6 / downsample_rate) > (timestamp - timers_gps["gga"])) break;
                    gps_serialize_gga(proto->add_gga(), &data->gga);
                    timers_gps["gga"] = timestamp;
                    break;
                case GPS_NMEA_TYPE_VTG:
                    if((1.0e6 / downsample_rate) > (timestamp - timers_gps["vtg"])) break;
                    gps_serialize_vtg(proto->add_vtg(), &data->vtg);
                    timers_gps["vtg"] = timestamp;
                    break;
                case GPS_NMEA_TYPE_GSA:
                    if((1.0e6 / downsample_rate) > (timestamp - timers_gps["gsa"])) break;
                    gps_serialize_gsa(proto->add_gsa(), &data->gsa);
                    timers_gps["gsa"] = timestamp;
                    break;
                default:
                    break;
            }
        break;
        case GPS_PROTOCOL_TYPE_UBX:
            switch (match.message)
            {
                case GPS_UBX_TYPE_NAV_DOP:
                    if((1.0e6 / downsample_rate) > (timestamp - timers_gps["dop"])) break;
                    gps_serialize_dop(proto->add_dop(), &data->dop);
                    timers_gps["dop"] = timestamp;
                break;
                case GPS_UBX_TYPE_NAV_PVT:
                    if((1.0e6 / downsample_rate) > (timestamp - timers_gps["pvt"])) break;
                    gps_serialize_pvt(proto->add_pvt(), &data->pvt);
                    timers_gps["pvt"] = timestamp;
                break;
                case GPS_UBX_TYPE_NAV_HPPOSECEF:
                    if((1.0e6 / downsample_rate) > (timestamp - timers_gps["hpposecef"])) break;
                    gps_serialize_hpposecef(proto->add_hpposecef(), &data->hpposecef);
                    timers_gps["hpposecef"] = timestamp;
                break;
                case GPS_UBX_TYPE_NAV_HPPOSLLH:
                    if((1.0e6 / downsample_rate) > (timestamp - timers_gps["hpposllh"])) break;
                    gps_serialize_hpposllh(proto->add_hpposllh(), &data->hpposllh);
                    timers_gps["hpposllh"] = timestamp;
                break;
                default:
                    break;
            }
        break;
        default:
        break;
    }
}

void gps_proto_deserialize(gps::GpsPack *proto, network_enums *net_enums,
                           network_signals *net_signals,
                           network_strings *net_strings, uint64_t resample_us) {
  char buffer[1024];
  for (int i = 0; i < proto->gga_size(); i++) {
#ifdef CANLIB_TIMESTAMP
    static uint64_t last_timestamp = 0;
    if (proto->gga(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->gga(i)._inner_timestamp();
    (*net_signals)["GGA"]["_timestamp"].push(proto->gga(i)._inner_timestamp());
#endif // CANLIB_TIMESTAMP
    (*net_strings)["GGA"]["time"].push(
        std::string(proto->gga(i).time().c_str(), 9));
    (*net_signals)["GGA"]["latitude"].push(proto->gga(i).latitude());
    (*net_strings)["GGA"]["north_south"].push(proto->gga(i).north_south());
    (*net_signals)["GGA"]["longitude"].push(proto->gga(i).longitude());
    (*net_strings)["GGA"]["east_ovest"].push(proto->gga(i).east_ovest());
    (*net_signals)["GGA"]["fix"].push(proto->gga(i).fix());
    (*net_signals)["GGA"]["satellites"].push(proto->gga(i).satellites());
    (*net_signals)["GGA"]["horizontal_diluition_precision"].push(
        proto->gga(i).horizontal_diluition_precision());
    (*net_enums)["GGA"]["fix_state"].push(proto->gga(i).fix());
    (*net_strings)["GGA"]["fix_state"].push(FIX_STATE[proto->gga(i).fix()]);
    (*net_signals)["GGA"]["altitude"].push(proto->gga(i).altitude());
    (*net_signals)["GGA"]["age_of_correction"].push(
        proto->gga(i).age_of_correction());
  }
  for (int i = 0; i < proto->vtg_size(); i++) {
#ifdef CANLIB_TIMESTAMP
    static uint64_t last_timestamp = 0;
    if (proto->vtg(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->vtg(i)._inner_timestamp();
    (*net_signals)["VTG"]["_timestamp"].push(proto->vtg(i)._inner_timestamp());
#endif // CANLIB_TIMESTAMP
    (*net_signals)["VTG"]["course_over_ground_degrees"].push(
        proto->vtg(i).course_over_ground_degrees());
    (*net_signals)["VTG"]["course_over_ground_degrees_magnetic"].push(
        proto->vtg(i).course_over_ground_degrees_magnetic());
    (*net_signals)["VTG"]["speed_kmh"].push(proto->vtg(i).speed_kmh());
  }
  for (int i = 0; i < proto->gsa_size(); i++) {
#ifdef CANLIB_TIMESTAMP
    static uint64_t last_timestamp = 0;
    if (proto->gsa(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->gsa(i)._inner_timestamp();
    (*net_signals)["GSA"]["_timestamp"].push(proto->gsa(i)._inner_timestamp());
#endif // CANLIB_TIMESTAMP
    (*net_strings)["GSA"]["mode"].push(proto->gsa(i).mode());
    (*net_signals)["GSA"]["position_diluition_precision"].push(
        proto->gsa(i).position_diluition_precision());
    (*net_signals)["GSA"]["horizontal_diluition_precision"].push(
        proto->gsa(i).horizontal_diluition_precision());
    (*net_signals)["GSA"]["vertical_diluition_precision"].push(
        proto->gsa(i).vertical_diluition_precision());
  }

  for (int i = 0; i < proto->dop_size(); i++) {
#ifdef CANLIB_TIMESTAMP
    static uint64_t last_timestamp = 0;
    if (proto->dop(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->dop(i)._inner_timestamp();
#endif // CANLIB_TIMESTAMP
    (*net_signals)["DOP"]["_timestamp"].push(proto->dop(i)._inner_timestamp());
    (*net_signals)["DOP"]["iTOW"].push(proto->dop(i).itow());
    (*net_signals)["DOP"]["gDOP"].push(proto->dop(i).gdop());
    (*net_signals)["DOP"]["pDOP"].push(proto->dop(i).pdop());
    (*net_signals)["DOP"]["tDOP"].push(proto->dop(i).tdop());
    (*net_signals)["DOP"]["vDOP"].push(proto->dop(i).vdop());
    (*net_signals)["DOP"]["hDOP"].push(proto->dop(i).hdop());
    (*net_signals)["DOP"]["nDOP"].push(proto->dop(i).ndop());
    (*net_signals)["DOP"]["eDOP"].push(proto->dop(i).edop());
  }
  for (int i = 0; i < proto->pvt_size(); i++) {
#ifdef CANLIB_TIMESTAMP
    static uint64_t last_timestamp = 0;
    if (proto->pvt(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->pvt(i)._inner_timestamp();
#endif // CANLIB_TIMESTAMP
    (*net_signals)["PVT"]["_timestamp"].push(proto->pvt(i)._inner_timestamp());
    (*net_signals)["PVT"]["iTOW"].push(proto->pvt(i).itow());
    (*net_signals)["PVT"]["year"].push(proto->pvt(i).year());
    (*net_signals)["PVT"]["month"].push(proto->pvt(i).month());
    (*net_signals)["PVT"]["day"].push(proto->pvt(i).day());
    (*net_signals)["PVT"]["hour"].push(proto->pvt(i).hour());
    (*net_signals)["PVT"]["min"].push(proto->pvt(i).min());
    (*net_signals)["PVT"]["sec"].push(proto->pvt(i).sec());
    (*net_signals)["PVT"]["valid"].push(proto->pvt(i).valid());
    (*net_signals)["PVT"]["tAcc"].push(proto->pvt(i).tacc());
    (*net_signals)["PVT"]["nano"].push(proto->pvt(i).nano());
    (*net_signals)["PVT"]["fixType"].push(proto->pvt(i).fixtype());
    (*net_signals)["PVT"]["flags"].push(proto->pvt(i).flags());
    (*net_signals)["PVT"]["flags2"].push(proto->pvt(i).flags2());
    (*net_signals)["PVT"]["numSV"].push(proto->pvt(i).numsv());
    (*net_signals)["PVT"]["lon"].push(proto->pvt(i).lon());
    (*net_signals)["PVT"]["lat"].push(proto->pvt(i).lat());
    (*net_signals)["PVT"]["height"].push(proto->pvt(i).height());
    (*net_signals)["PVT"]["hMSL"].push(proto->pvt(i).hmsl());
    (*net_signals)["PVT"]["hAcc"].push(proto->pvt(i).hacc());
    (*net_signals)["PVT"]["vAcc"].push(proto->pvt(i).vacc());
    (*net_signals)["PVT"]["velN"].push(proto->pvt(i).veln());
    (*net_signals)["PVT"]["velE"].push(proto->pvt(i).vele());
    (*net_signals)["PVT"]["velD"].push(proto->pvt(i).veld());
    (*net_signals)["PVT"]["gSpeed"].push(proto->pvt(i).gspeed());
    (*net_signals)["PVT"]["headMot"].push(proto->pvt(i).headmot());
    (*net_signals)["PVT"]["sAcc"].push(proto->pvt(i).sacc());
    (*net_signals)["PVT"]["headAcc"].push(proto->pvt(i).headacc());
    (*net_signals)["PVT"]["pDOP"].push(proto->pvt(i).pdop());
    (*net_signals)["PVT"]["headVeh"].push(proto->pvt(i).headveh());
    (*net_signals)["PVT"]["magDec"].push(proto->pvt(i).magdec());
    (*net_signals)["PVT"]["magAcc"].push(proto->pvt(i).magacc());
  }
  for (int i = 0; i < proto->hpposecef_size(); i++) {
#ifdef CANLIB_TIMESTAMP
    static uint64_t last_timestamp = 0;
    if (proto->hpposecef(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->hpposecef(i)._inner_timestamp();
#endif // CANLIB_TIMESTAMP
    (*net_signals)["HPPOSECEF"]["_timestamp"].push(
        proto->hpposecef(i)._inner_timestamp());
    (*net_signals)["HPPOSECEF"]["version"].push(proto->hpposecef(i).version());
    (*net_signals)["HPPOSECEF"]["iTOW"].push(proto->hpposecef(i).itow());
    (*net_signals)["HPPOSECEF"]["ecefX"].push(proto->hpposecef(i).ecefx());
    (*net_signals)["HPPOSECEF"]["ecefY"].push(proto->hpposecef(i).ecefy());
    (*net_signals)["HPPOSECEF"]["ecefZ"].push(proto->hpposecef(i).ecefz());
    (*net_signals)["HPPOSECEF"]["ecefXHp"].push(proto->hpposecef(i).ecefxhp());
    (*net_signals)["HPPOSECEF"]["ecefYHp"].push(proto->hpposecef(i).ecefyhp());
    (*net_signals)["HPPOSECEF"]["ecefZHp"].push(proto->hpposecef(i).ecefzhp());
    (*net_signals)["HPPOSECEF"]["pAcc"].push(proto->hpposecef(i).pacc());
  }
  for (int i = 0; i < proto->hpposllh_size(); i++) {
#ifdef CANLIB_TIMESTAMP
    static uint64_t last_timestamp = 0;
    if (proto->hpposllh(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->hpposllh(i)._inner_timestamp();
#endif // CANLIB_TIMESTAMP
    (*net_signals)["HPPOSLLH"]["_timestamp"].push(
        proto->hpposllh(i)._inner_timestamp());
    (*net_signals)["HPPOSLLH"]["version"].push(proto->hpposllh(i).version());
    (*net_signals)["HPPOSLLH"]["iTOW"].push(proto->hpposllh(i).itow());
    (*net_signals)["HPPOSLLH"]["lon"].push(proto->hpposllh(i).lon());
    (*net_signals)["HPPOSLLH"]["lat"].push(proto->hpposllh(i).lat());
    (*net_signals)["HPPOSLLH"]["height"].push(proto->hpposllh(i).height());
    (*net_signals)["HPPOSLLH"]["hMSL"].push(proto->hpposllh(i).hmsl());
    (*net_signals)["HPPOSLLH"]["lonHp"].push(proto->hpposllh(i).lonhp());
    (*net_signals)["HPPOSLLH"]["latHp"].push(proto->hpposllh(i).lathp());
    (*net_signals)["HPPOSLLH"]["heightHp"].push(proto->hpposllh(i).heighthp());
    (*net_signals)["HPPOSLLH"]["hMSLHp"].push(proto->hpposllh(i).hmslhp());
    (*net_signals)["HPPOSLLH"]["hAcc"].push(proto->hpposllh(i).hacc());
    (*net_signals)["HPPOSLLH"]["vAcc"].push(proto->hpposllh(i).vacc());
  }
}

void gps_serialize_gga(gps::GGA* proto, gps_nmea_gga_t* data){
    proto->set__inner_timestamp(data->_timestamp);
    proto->set_time(data->time);
    proto->set_latitude(data->latitude);
    //proto->set_north_south(std::string(data->north_south, 1));
    proto->set_longitude(data->longitude);
    //proto->set_east_ovest(std::string(data->east_ovest, 1));
    proto->set_fix(data->fix);
    proto->set_satellites(data->satellites);
    proto->set_horizontal_diluition_precision(data->horizontal_diluition_precision);
    proto->set_altitude(data->altitude);
    proto->set_age_of_correction(data->age_of_correction);
}
void gps_serialize_vtg(gps::VTG* proto, gps_nmea_vtg_t* data){
    proto->set__inner_timestamp(data->_timestamp);
    proto->set_course_over_ground_degrees(data->course_over_ground_degrees);
    proto->set_course_over_ground_degrees_magnetic(data->course_over_ground_degrees_magnetic);
    proto->set_speed_kmh(data->speed_kmh);
}
void gps_serialize_gsa(gps::GSA* proto, gps_nmea_gsa_t* data){
    proto->set__inner_timestamp(data->_timestamp);
    proto->set_mode(std::string(data->mode, 1));
    proto->set_position_diluition_precision(data->position_diluition_precision);
    proto->set_horizontal_diluition_precision(data->horizontal_diluition_precision);
    proto->set_vertical_diluition_precision(data->vertical_diluition_precision);
}

void gps_serialize_dop(gps::NAV_DOP* proto, gps_ubx_dop_t* data){
    proto->set__inner_timestamp(data->_timestamp);
    proto->set_itow(data->iTOW);
    proto->set_gdop(data->gDOP);
    proto->set_pdop(data->pDOP);
    proto->set_tdop(data->tDOP);
    proto->set_vdop(data->vDOP);
    proto->set_hdop(data->hDOP);
    proto->set_ndop(data->nDOP);
    proto->set_edop(data->eDOP);
}
void gps_serialize_pvt(gps::NAV_PVT* proto, gps_ubx_pvt_t* data){
    proto->set__inner_timestamp(data->_timestamp);
    proto->set_itow(data->iTOW);
    proto->set_year(data->year);
    proto->set_month(data->month);
    proto->set_day(data->day);
    proto->set_hour(data->hour);
    proto->set_min(data->min);
    proto->set_sec(data->sec);
    proto->set_valid(data->valid);
    proto->set_tacc(data->tAcc);
    proto->set_nano(data->nano);
    proto->set_fixtype(data->fixType);
    proto->set_flags(data->flags);
    proto->set_flags2(data->flags2);
    proto->set_numsv(data->numSV);
    proto->set_lon(data->lon);
    proto->set_lat(data->lat);
    proto->set_height(data->height);
    proto->set_hmsl(data->hMSL);
    proto->set_hacc(data->hAcc);
    proto->set_vacc(data->vAcc);
    proto->set_veln(data->velN);
    proto->set_vele(data->velE);
    proto->set_veld(data->velD);
    proto->set_gspeed(data->gSpeed);
    proto->set_headmot(data->headMot);
    proto->set_sacc(data->sAcc);
    proto->set_headacc(data->headAcc);
    proto->set_pdop(data->pDOP);
    proto->set_headveh(data->headVeh);
    proto->set_magdec(data->magDec);
    proto->set_magacc(data->magAcc);
}
void gps_serialize_hpposecef(gps::NAV_HPPOSECEF* proto, gps_ubx_hpposecef_t* data){
    proto->set__inner_timestamp(data->_timestamp);
    proto->set_version(data->version);
    proto->set_itow(data->iTOW);
    proto->set_ecefx(data->ecefX);
    proto->set_ecefy(data->ecefY);
    proto->set_ecefz(data->ecefZ);
    proto->set_ecefxhp(data->ecefXHp);
    proto->set_ecefyhp(data->ecefYHp);
    proto->set_ecefzhp(data->ecefZHp);
    proto->set_pacc(data->pAcc);
}
void gps_serialize_hpposllh(gps::NAV_HPPOSLLH* proto, gps_ubx_hpposllh_t* data){
    proto->set__inner_timestamp(data->_timestamp);
    proto->set_version(data->version);
    proto->set_itow(data->iTOW);
    proto->set_lon(data->lon);
    proto->set_lat(data->lat);
    proto->set_height(data->height);
    proto->set_hmsl(data->hMSL);
    proto->set_lonhp(data->lonHp);
    proto->set_lathp(data->latHp);
    proto->set_heighthp(data->heightHp);
    proto->set_hmslhp(data->hMSLHp);
    proto->set_hacc(data->hAcc);
    proto->set_vacc(data->vAcc);
}

void gps_mapping_adaptor_construct(gps_proto_pack& pack, mapping_adaptor& mapping_map){
    mapping_map["GPS_GGA"].size = std::bind(&canlib_circular_buffer<gps_nmea_gga_t, CANLIB_CIRCULAR_BUFFER_SIZE>::size, &pack.gga);
    mapping_map["GPS_GGA"].offset = std::bind(&canlib_circular_buffer<gps_nmea_gga_t, CANLIB_CIRCULAR_BUFFER_SIZE>::offset, &pack.gga);
    mapping_map["GPS_GGA"].stride = sizeof(gps_nmea_gga_t);
    mapping_map["GPS_GGA"].field["_timestamp"].value._uint64 = &pack.gga.start()._timestamp;
    mapping_map["GPS_GGA"].field["_timestamp"].type = mapping_type_uint64;
    mapping_map["GPS_GGA"].field["latitude"].value._float64 = &pack.gga.start().latitude;
    mapping_map["GPS_GGA"].field["latitude"].type = mapping_type_float64;
    mapping_map["GPS_GGA"].field["longitude"].value._float64 = &pack.gga.start().longitude;
    mapping_map["GPS_GGA"].field["longitude"].type = mapping_type_float64;
    mapping_map["GPS_GGA"].field["fix"].value._uint8 = &pack.gga.start().fix;
    mapping_map["GPS_GGA"].field["fix"].type = mapping_type_uint8;
    mapping_map["GPS_GGA"].field["satellites"].value._uint8 = &pack.gga.start().satellites;
    mapping_map["GPS_GGA"].field["satellites"].type = mapping_type_uint8;
    mapping_map["GPS_GGA"].field["horizontal_diluition_precision"].value._float64  = &pack.gga.start().horizontal_diluition_precision;
    mapping_map["GPS_GGA"].field["horizontal_diluition_precision"].type = mapping_type_float64;
    mapping_map["GPS_GGA"].field["altitude"].value._float64 = &pack.gga.start().altitude;
    mapping_map["GPS_GGA"].field["altitude"].type = mapping_type_float64;
    mapping_map["GPS_GGA"].field["age_of_correction"].value._float64 = &pack.gga.start().age_of_correction;
    mapping_map["GPS_GGA"].field["age_of_correction"].type = mapping_type_float64;

    mapping_map["GPS_VTG"].size = std::bind(&canlib_circular_buffer<gps_nmea_vtg_t, CANLIB_CIRCULAR_BUFFER_SIZE>::size, &pack.vtg);
    mapping_map["GPS_VTG"].offset = std::bind(&canlib_circular_buffer<gps_nmea_vtg_t, CANLIB_CIRCULAR_BUFFER_SIZE>::offset, &pack.vtg);
    mapping_map["GPS_VTG"].stride = sizeof(gps_nmea_vtg_t);
    mapping_map["GPS_VTG"].field["_timestamp"].value._uint64 = &pack.vtg.start()._timestamp;
    mapping_map["GPS_VTG"].field["_timestamp"].type = mapping_type_uint64;
    mapping_map["GPS_VTG"].field["course_over_ground_degrees"].value._float64 = &pack.vtg.start().course_over_ground_degrees;
    mapping_map["GPS_VTG"].field["course_over_ground_degrees"].type = mapping_type_float64;
    mapping_map["GPS_VTG"].field["course_over_ground_degrees_magnetic"].value._float64 = &pack.vtg.start().course_over_ground_degrees_magnetic;
    mapping_map["GPS_VTG"].field["course_over_ground_degrees_magnetic"].type = mapping_type_float64;
    mapping_map["GPS_VTG"].field["speed_kmh"].value._float64 = &pack.vtg.start().speed_kmh;
    mapping_map["GPS_VTG"].field["speed_kmh"].type = mapping_type_float64;

    mapping_map["GPS_GSA"].size = std::bind(&canlib_circular_buffer<gps_nmea_gsa_t, CANLIB_CIRCULAR_BUFFER_SIZE>::size, &pack.gsa);
    mapping_map["GPS_GSA"].offset = std::bind(&canlib_circular_buffer<gps_nmea_gsa_t, CANLIB_CIRCULAR_BUFFER_SIZE>::offset, &pack.gsa);
    mapping_map["GPS_GSA"].stride = sizeof(gps_nmea_gsa_t);
    mapping_map["GPS_GSA"].field["_timestamp"].value._uint64 = &pack.gsa.start()._timestamp;
    mapping_map["GPS_GSA"].field["_timestamp"].type = mapping_type_uint64;
    mapping_map["GPS_GSA"].field["position_diluition_precision"].value._float64 = &pack.gsa.start().position_diluition_precision;
    mapping_map["GPS_GSA"].field["position_diluition_precision"].type = mapping_type_float64;
    mapping_map["GPS_GSA"].field["horizontal_diluition_precision"].value._float64 = &pack.gsa.start().horizontal_diluition_precision;
    mapping_map["GPS_GSA"].field["horizontal_diluition_precision"].type = mapping_type_float64;
    mapping_map["GPS_GSA"].field["vertical_diluition_precision"].value._float64 = &pack.gsa.start().vertical_diluition_precision;
    mapping_map["GPS_GSA"].field["vertical_diluition_precision"].type = mapping_type_float64;

    mapping_map["GPS_NAV_DOP"].size = std::bind(&canlib_circular_buffer<gps_ubx_dop_t, CANLIB_CIRCULAR_BUFFER_SIZE>::size, &pack.dop);
    mapping_map["GPS_NAV_DOP"].offset = std::bind(&canlib_circular_buffer<gps_ubx_dop_t, CANLIB_CIRCULAR_BUFFER_SIZE>::offset, &pack.dop);
    mapping_map["GPS_NAV_DOP"].stride = sizeof(gps_ubx_dop_t);
    mapping_map["GPS_NAV_DOP"].field["_timestamp"].value._uint64 = &pack.dop.start()._timestamp;
    mapping_map["GPS_NAV_DOP"].field["_timestamp"].type = mapping_type_uint64;
    mapping_map["GPS_NAV_DOP"].field["iTOW"].value._uint32 = &pack.dop.start().iTOW;
    mapping_map["GPS_NAV_DOP"].field["iTOW"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_DOP"].field["gDOP"].value._float32 = &pack.dop.start().gDOP;
    mapping_map["GPS_NAV_DOP"].field["gDOP"].type = mapping_type_float32;
    mapping_map["GPS_NAV_DOP"].field["pDOP"].value._float32 = &pack.dop.start().pDOP;
    mapping_map["GPS_NAV_DOP"].field["pDOP"].type = mapping_type_float32;
    mapping_map["GPS_NAV_DOP"].field["tDOP"].value._float32 = &pack.dop.start().tDOP;
    mapping_map["GPS_NAV_DOP"].field["tDOP"].type = mapping_type_float32;
    mapping_map["GPS_NAV_DOP"].field["vDOP"].value._float32 = &pack.dop.start().vDOP;
    mapping_map["GPS_NAV_DOP"].field["vDOP"].type = mapping_type_float32;
    mapping_map["GPS_NAV_DOP"].field["hDOP"].value._float32 = &pack.dop.start().hDOP;
    mapping_map["GPS_NAV_DOP"].field["hDOP"].type = mapping_type_float32;
    mapping_map["GPS_NAV_DOP"].field["nDOP"].value._float32 = &pack.dop.start().nDOP;
    mapping_map["GPS_NAV_DOP"].field["nDOP"].type = mapping_type_float32;
    mapping_map["GPS_NAV_DOP"].field["eDOP"].value._float32 = &pack.dop.start().eDOP;
    mapping_map["GPS_NAV_DOP"].field["eDOP"].type = mapping_type_float32;

    mapping_map["GPS_NAV_PVT"].size = std::bind(&canlib_circular_buffer<gps_ubx_pvt_t, CANLIB_CIRCULAR_BUFFER_SIZE>::size, &pack.pvt);
    mapping_map["GPS_NAV_PVT"].offset = std::bind(&canlib_circular_buffer<gps_ubx_pvt_t, CANLIB_CIRCULAR_BUFFER_SIZE>::offset, &pack.pvt);
    mapping_map["GPS_NAV_PVT"].stride = sizeof(gps_ubx_pvt_t);
    mapping_map["GPS_NAV_PVT"].field["_timestamp"].value._uint64  = &pack.pvt.start()._timestamp;
    mapping_map["GPS_NAV_PVT"].field["_timestamp"].type = mapping_type_uint64;
    mapping_map["GPS_NAV_PVT"].field["iTOW"].value._uint32  = &pack.pvt.start().iTOW;
    mapping_map["GPS_NAV_PVT"].field["iTOW"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_PVT"].field["year"].value._uint16  = &pack.pvt.start().year;
    mapping_map["GPS_NAV_PVT"].field["year"].type = mapping_type_uint16;
    mapping_map["GPS_NAV_PVT"].field["month"].value._uint8  = &pack.pvt.start().month;
    mapping_map["GPS_NAV_PVT"].field["month"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["day"].value._uint8  = &pack.pvt.start().day;
    mapping_map["GPS_NAV_PVT"].field["day"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["hour"].value._uint8  = &pack.pvt.start().hour;
    mapping_map["GPS_NAV_PVT"].field["hour"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["min"].value._uint8  = &pack.pvt.start().min;
    mapping_map["GPS_NAV_PVT"].field["min"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["sec"].value._uint8  = &pack.pvt.start().sec;
    mapping_map["GPS_NAV_PVT"].field["sec"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["valid"].value._uint8  = &pack.pvt.start().valid;
    mapping_map["GPS_NAV_PVT"].field["valid"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["tAcc"].value._uint32  = &pack.pvt.start().tAcc;
    mapping_map["GPS_NAV_PVT"].field["tAcc"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_PVT"].field["nano"].value._int32  = &pack.pvt.start().nano;
    mapping_map["GPS_NAV_PVT"].field["nano"].type = mapping_type_int32;
    mapping_map["GPS_NAV_PVT"].field["fixType"].value._uint8  = &pack.pvt.start().fixType;
    mapping_map["GPS_NAV_PVT"].field["fixType"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["flags"].value._uint8  = &pack.pvt.start().flags;
    mapping_map["GPS_NAV_PVT"].field["flags"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["flags2"].value._uint8  = &pack.pvt.start().flags2;
    mapping_map["GPS_NAV_PVT"].field["flags2"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["numSV"].value._uint8  = &pack.pvt.start().numSV;
    mapping_map["GPS_NAV_PVT"].field["numSV"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_PVT"].field["lon"].value._float64  = &pack.pvt.start().lon;
    mapping_map["GPS_NAV_PVT"].field["lon"].type = mapping_type_float64;
    mapping_map["GPS_NAV_PVT"].field["lat"].value._float64  = &pack.pvt.start().lat;
    mapping_map["GPS_NAV_PVT"].field["lat"].type = mapping_type_float64;
    mapping_map["GPS_NAV_PVT"].field["height"].value._int32  = &pack.pvt.start().height;
    mapping_map["GPS_NAV_PVT"].field["height"].type = mapping_type_int32;
    mapping_map["GPS_NAV_PVT"].field["hMSL"].value._int32  = &pack.pvt.start().hMSL;
    mapping_map["GPS_NAV_PVT"].field["hMSL"].type = mapping_type_int32;
    mapping_map["GPS_NAV_PVT"].field["hAcc"].value._uint32  = &pack.pvt.start().hAcc;
    mapping_map["GPS_NAV_PVT"].field["hAcc"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_PVT"].field["vAcc"].value._uint32  = &pack.pvt.start().vAcc;
    mapping_map["GPS_NAV_PVT"].field["vAcc"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_PVT"].field["velN"].value._int32  = &pack.pvt.start().velN;
    mapping_map["GPS_NAV_PVT"].field["velN"].type = mapping_type_int32;
    mapping_map["GPS_NAV_PVT"].field["velE"].value._int32  = &pack.pvt.start().velE;
    mapping_map["GPS_NAV_PVT"].field["velE"].type = mapping_type_int32;
    mapping_map["GPS_NAV_PVT"].field["velD"].value._int32  = &pack.pvt.start().velD;
    mapping_map["GPS_NAV_PVT"].field["velD"].type = mapping_type_int32;
    mapping_map["GPS_NAV_PVT"].field["gSpeed"].value._int32  = &pack.pvt.start().gSpeed;
    mapping_map["GPS_NAV_PVT"].field["gSpeed"].type = mapping_type_int32;
    mapping_map["GPS_NAV_PVT"].field["headMot"].value._float64  = &pack.pvt.start().headMot;
    mapping_map["GPS_NAV_PVT"].field["headMot"].type = mapping_type_float64;
    mapping_map["GPS_NAV_PVT"].field["sAcc"].value._uint32  = &pack.pvt.start().sAcc;
    mapping_map["GPS_NAV_PVT"].field["sAcc"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_PVT"].field["headAcc"].value._float64  = &pack.pvt.start().headAcc;
    mapping_map["GPS_NAV_PVT"].field["headAcc"].type = mapping_type_float64;
    mapping_map["GPS_NAV_PVT"].field["pDOP"].value._uint16  = &pack.pvt.start().pDOP;
    mapping_map["GPS_NAV_PVT"].field["pDOP"].type = mapping_type_uint16;
    mapping_map["GPS_NAV_PVT"].field["headVeh"].value._float64  = &pack.pvt.start().headVeh;
    mapping_map["GPS_NAV_PVT"].field["headVeh"].type = mapping_type_float64;
    mapping_map["GPS_NAV_PVT"].field["magDec"].value._float64  = &pack.pvt.start().magDec;
    mapping_map["GPS_NAV_PVT"].field["magDec"].type = mapping_type_float64;
    mapping_map["GPS_NAV_PVT"].field["magAcc"].value._float64  = &pack.pvt.start().magAcc;
    mapping_map["GPS_NAV_PVT"].field["magAcc"].type = mapping_type_float64;

    mapping_map["GPS_NAV_HPPOSECEF"].size = std::bind(&canlib_circular_buffer<gps_ubx_hpposecef_t, CANLIB_CIRCULAR_BUFFER_SIZE>::size, &pack.hpposecef);
    mapping_map["GPS_NAV_HPPOSECEF"].offset = std::bind(&canlib_circular_buffer<gps_ubx_hpposecef_t, CANLIB_CIRCULAR_BUFFER_SIZE>::offset, &pack.hpposecef);
    mapping_map["GPS_NAV_HPPOSECEF"].stride = sizeof(gps_ubx_hpposecef_t);
    mapping_map["GPS_NAV_HPPOSECEF"].field["_timestamp"].value._uint64 = &pack.hpposecef.start()._timestamp;
    mapping_map["GPS_NAV_HPPOSECEF"].field["_timestamp"].type = mapping_type_uint64;
    mapping_map["GPS_NAV_HPPOSECEF"].field["version"].value._uint8 = &pack.hpposecef.start().version;
    mapping_map["GPS_NAV_HPPOSECEF"].field["version"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_HPPOSECEF"].field["iTOW"].value._uint32 = &pack.hpposecef.start().iTOW;
    mapping_map["GPS_NAV_HPPOSECEF"].field["iTOW"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefX"].value._float64 = &pack.hpposecef.start().ecefX;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefX"].type = mapping_type_float64;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefY"].value._float64 = &pack.hpposecef.start().ecefY;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefY"].type = mapping_type_float64;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefZ"].value._float64 = &pack.hpposecef.start().ecefZ;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefZ"].type = mapping_type_float64;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefXHp"].value._int8 = &pack.hpposecef.start().ecefXHp;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefXHp"].type = mapping_type_int8;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefYHp"].value._int8 = &pack.hpposecef.start().ecefYHp;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefYHp"].type = mapping_type_int8;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefZHp"].value._int8 = &pack.hpposecef.start().ecefZHp;
    mapping_map["GPS_NAV_HPPOSECEF"].field["ecefZHp"].type = mapping_type_int8;
    mapping_map["GPS_NAV_HPPOSECEF"].field["pAcc"].value._float32 = &pack.hpposecef.start().pAcc;
    mapping_map["GPS_NAV_HPPOSECEF"].field["pAcc"].type = mapping_type_float32;

    mapping_map["GPS_NAV_HPPOSLLH"].size = std::bind(&canlib_circular_buffer<gps_ubx_hpposllh_t, CANLIB_CIRCULAR_BUFFER_SIZE>::size, &pack.hpposllh);
    mapping_map["GPS_NAV_HPPOSLLH"].offset = std::bind(&canlib_circular_buffer<gps_ubx_hpposllh_t, CANLIB_CIRCULAR_BUFFER_SIZE>::offset, &pack.hpposllh);
    mapping_map["GPS_NAV_HPPOSLLH"].stride = sizeof(gps_ubx_hpposllh_t);
    mapping_map["GPS_NAV_HPPOSLLH"].field["_timestamp"].value._uint64 = &pack.hpposllh.start()._timestamp;
    mapping_map["GPS_NAV_HPPOSLLH"].field["_timestamp"].type = mapping_type_uint64;
    mapping_map["GPS_NAV_HPPOSLLH"].field["version"].value._uint8 = &pack.hpposllh.start().version;
    mapping_map["GPS_NAV_HPPOSLLH"].field["version"].type = mapping_type_uint8;
    mapping_map["GPS_NAV_HPPOSLLH"].field["iTOW"].value._uint32 = &pack.hpposllh.start().iTOW;
    mapping_map["GPS_NAV_HPPOSLLH"].field["iTOW"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_HPPOSLLH"].field["lon"].value._float64 = &pack.hpposllh.start().lon;
    mapping_map["GPS_NAV_HPPOSLLH"].field["lon"].type = mapping_type_float64;
    mapping_map["GPS_NAV_HPPOSLLH"].field["lat"].value._float64 = &pack.hpposllh.start().lat;
    mapping_map["GPS_NAV_HPPOSLLH"].field["lat"].type = mapping_type_float64;
    mapping_map["GPS_NAV_HPPOSLLH"].field[" height"].value._float32 = &pack.hpposllh.start(). height;
    mapping_map["GPS_NAV_HPPOSLLH"].field[" height"].type = mapping_type_float32;
    mapping_map["GPS_NAV_HPPOSLLH"].field[" hMSL"].value._float32 = &pack.hpposllh.start(). hMSL;
    mapping_map["GPS_NAV_HPPOSLLH"].field[" hMSL"].type = mapping_type_float32;
    mapping_map["GPS_NAV_HPPOSLLH"].field["lonHp"].value._int8 = &pack.hpposllh.start().lonHp;
    mapping_map["GPS_NAV_HPPOSLLH"].field["lonHp"].type = mapping_type_int8;
    mapping_map["GPS_NAV_HPPOSLLH"].field["latHp"].value._int8 = &pack.hpposllh.start().latHp;
    mapping_map["GPS_NAV_HPPOSLLH"].field["latHp"].type = mapping_type_int8;
    mapping_map["GPS_NAV_HPPOSLLH"].field["heightHp"].value._int8 = &pack.hpposllh.start().heightHp;
    mapping_map["GPS_NAV_HPPOSLLH"].field["heightHp"].type = mapping_type_int8;
    mapping_map["GPS_NAV_HPPOSLLH"].field["hMSLHp"].value._int8 = &pack.hpposllh.start().hMSLHp;
    mapping_map["GPS_NAV_HPPOSLLH"].field["hMSLHp"].type = mapping_type_int8;
    mapping_map["GPS_NAV_HPPOSLLH"].field["hAcc"].value._uint32 = &pack.hpposllh.start().hAcc;
    mapping_map["GPS_NAV_HPPOSLLH"].field["hAcc"].type = mapping_type_uint32;
    mapping_map["GPS_NAV_HPPOSLLH"].field["vAcc"].value._uint32 = &pack.hpposllh.start().vAcc;
    mapping_map["GPS_NAV_HPPOSLLH"].field["vAcc"].type = mapping_type_uint32;
}