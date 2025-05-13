#include "gps_proto.h"

#include <string.h>

#include <functional>

void gps_proto_serialize_from_match(gps_protocol_and_message &match, gps::GpsPack *proto, gps_parsed_data_t *data,
                                    uint64_t &timestamp, uint64_t downsample_rate) {
  switch (match.protocol) {
    case GPS_PROTOCOL_TYPE_NMEA:
      switch (match.message) {
        case GPS_NMEA_TYPE_GGA:
          if (downsample_rate && (1.0e6 / downsample_rate) > (timestamp - timers_gps["gga"])) break;
          gps_serialize_gga(proto->add_gga(), &data->gga);
          timers_gps["gga"] = timestamp;
          break;
        case GPS_NMEA_TYPE_VTG:
          if (downsample_rate && (1.0e6 / downsample_rate) > (timestamp - timers_gps["vtg"])) break;
          gps_serialize_vtg(proto->add_vtg(), &data->vtg);
          timers_gps["vtg"] = timestamp;
          break;
        case GPS_NMEA_TYPE_GSA:
          if (downsample_rate && (1.0e6 / downsample_rate) > (timestamp - timers_gps["gsa"])) break;
          gps_serialize_gsa(proto->add_gsa(), &data->gsa);
          timers_gps["gsa"] = timestamp;
          break;
        default:
          break;
      }
      break;
    case GPS_PROTOCOL_TYPE_UBX:
      switch (match.message) {
        case GPS_UBX_TYPE_NAV_DOP:
          if (downsample_rate && (1.0e6 / downsample_rate) > (timestamp - timers_gps["dop"])) break;
          gps_serialize_dop(proto->add_dop(), &data->dop);
          timers_gps["dop"] = timestamp;
          break;
        case GPS_UBX_TYPE_NAV_PVT:
          if (downsample_rate && (1.0e6 / downsample_rate) > (timestamp - timers_gps["pvt"])) break;
          gps_serialize_pvt(proto->add_pvt(), &data->pvt);
          timers_gps["pvt"] = timestamp;
          break;
        case GPS_UBX_TYPE_NAV_HPPOSECEF:
          if (downsample_rate && (1.0e6 / downsample_rate) > (timestamp - timers_gps["hpposecef"])) break;
          gps_serialize_hpposecef(proto->add_hpposecef(), &data->hpposecef);
          timers_gps["hpposecef"] = timestamp;
          break;
        case GPS_UBX_TYPE_NAV_HPPOSLLH:
          if (downsample_rate && (1.0e6 / downsample_rate) > (timestamp - timers_gps["hpposllh"])) break;
          gps_serialize_hpposllh(proto->add_hpposllh(), &data->hpposllh);
          timers_gps["hpposllh"] = timestamp;
          break;
        case GPS_UBX_TYPE_NAV_RELPOSNED:
          if (downsample_rate && (1.0e6 / downsample_rate) > (timestamp - timers_gps["relposned"])) break;
          gps_serialize_relposned(proto->add_relposned(), &data->relposned);
          timers_gps["relposned"] = timestamp;
          break;
        case GPS_UBX_TYPE_NAV_VELNED:
          if (downsample_rate && (1.0e6 / downsample_rate) > (timestamp - timers_gps["velned"])) break;
          gps_serialize_velned(proto->add_velned(), &data->velned);
          timers_gps["velned"] = timestamp;
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

void gps_serialize_gga(gps::GGA *proto, gps_nmea_gga_t *data) {
  proto->set__inner_timestamp(data->_timestamp);
  proto->set_time(data->time);
  proto->set_latitude(data->latitude);
  // proto->set_north_south(std::string(data->north_south, 1));
  proto->set_longitude(data->longitude);
  // proto->set_east_ovest(std::string(data->east_ovest, 1));
  proto->set_fix(data->fix);
  proto->set_satellites(data->satellites);
  proto->set_horizontal_diluition_precision(data->horizontal_diluition_precision);
  proto->set_altitude(data->altitude);
  proto->set_age_of_correction(data->age_of_correction);
}
void gps_serialize_vtg(gps::VTG *proto, gps_nmea_vtg_t *data) {
  proto->set__inner_timestamp(data->_timestamp);
  proto->set_course_over_ground_degrees(data->course_over_ground_degrees);
  proto->set_course_over_ground_degrees_magnetic(data->course_over_ground_degrees_magnetic);
  proto->set_speed_kmh(data->speed_kmh);
}
void gps_serialize_gsa(gps::GSA *proto, gps_nmea_gsa_t *data) {
  proto->set__inner_timestamp(data->_timestamp);
  proto->set_mode(std::string(data->mode, 1));
  proto->set_position_diluition_precision(data->position_diluition_precision);
  proto->set_horizontal_diluition_precision(data->horizontal_diluition_precision);
  proto->set_vertical_diluition_precision(data->vertical_diluition_precision);
}

void gps_serialize_dop(gps::NAV_DOP *proto, gps_ubx_dop_t *data) {
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
void gps_serialize_pvt(gps::NAV_PVT *proto, gps_ubx_pvt_t *data) {
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
void gps_serialize_hpposecef(gps::NAV_HPPOSECEF *proto, gps_ubx_hpposecef_t *data) {
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
void gps_serialize_hpposllh(gps::NAV_HPPOSLLH *proto, gps_ubx_hpposllh_t *data) {
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
void gps_serialize_relposned(gps::NAV_RELPOSNED *proto, gps_ubx_relposned_t *data) {
  proto->set__inner_timestamp(data->_timestamp);
  proto->set_version(data->version);
  proto->set_refstationid(data->refStationId);
  proto->set_itow(data->iTOW);
  proto->set_relposn(data->relPosN);
  proto->set_relpose(data->relPosE);
  proto->set_relposd(data->relPosD);
  proto->set_relposlength(data->relPosLength);
  proto->set_relposheading(data->relPosHeading);
  proto->set_relposhpn(data->relPosHPN);
  proto->set_relposhpe(data->relPosHPE);
  proto->set_relposhpd(data->relPosHPD);
  proto->set_relposhplength(data->relPosHPLength);
  proto->set_accn(data->accN);
  proto->set_acce(data->accE);
  proto->set_accd(data->accD);
  proto->set_acclength(data->accLength);
  proto->set_accheading(data->accHeading);
  proto->set_flags(data->flags);
}
void gps_serialize_velned(gps::NAV_VELNED *proto, gps_ubx_velned_t *data) {
  proto->set__inner_timestamp(data->_timestamp);
  proto->set_itow(data->iTOW);
  proto->set_veln(data->velN);
  proto->set_vele(data->velE);
  proto->set_veld(data->velD);
  proto->set_speed(data->speed);
  proto->set_gspeed(data->gSpeed);
  proto->set_heading(data->heading);
  proto->set_sacc(data->sAcc);
  proto->set_cacc(data->cAcc);
}
