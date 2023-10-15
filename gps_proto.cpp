#include "gps_proto.h"
#include <functional>
#include <string.h>

void gps_proto_serialize_from_match(gps_protocol_and_message& match, gps::GpsPack* proto, gps_parsed_data_t* data, uint64_t &timestamp, uint64_t downsample_rate){
    switch (match.protocol)
    {
        case GPS_PROTOCOL_TYPE_NMEA:
    switch (match.message) {
    case GPS_NMEA_TYPE_GGA:
      if (downsample_rate &&
          (1.0e6 / downsample_rate) > (timestamp - timers_gps["gga"]))
        break;
      gps_serialize_gga(proto->add_gga(), &data->gga);
      timers_gps["gga"] = timestamp;
      break;
    case GPS_NMEA_TYPE_VTG:
      if (downsample_rate &&
          (1.0e6 / downsample_rate) > (timestamp - timers_gps["vtg"]))
        break;
      gps_serialize_vtg(proto->add_vtg(), &data->vtg);
      timers_gps["vtg"] = timestamp;
      break;
    case GPS_NMEA_TYPE_GSA:
      if (downsample_rate &&
          (1.0e6 / downsample_rate) > (timestamp - timers_gps["gsa"]))
        break;
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
      if (downsample_rate &&
          (1.0e6 / downsample_rate) > (timestamp - timers_gps["dop"]))
        break;
      gps_serialize_dop(proto->add_dop(), &data->dop);
      timers_gps["dop"] = timestamp;
      break;
    case GPS_UBX_TYPE_NAV_PVT:
      if (downsample_rate &&
          (1.0e6 / downsample_rate) > (timestamp - timers_gps["pvt"]))
        break;
      gps_serialize_pvt(proto->add_pvt(), &data->pvt);
      timers_gps["pvt"] = timestamp;
      break;
    case GPS_UBX_TYPE_NAV_HPPOSECEF:
      if (downsample_rate &&
          (1.0e6 / downsample_rate) > (timestamp - timers_gps["hpposecef"]))
        break;
      gps_serialize_hpposecef(proto->add_hpposecef(), &data->hpposecef);
      timers_gps["hpposecef"] = timestamp;
      break;
    case GPS_UBX_TYPE_NAV_HPPOSLLH:
      if (downsample_rate &&
          (1.0e6 / downsample_rate) > (timestamp - timers_gps["hpposllh"]))
        break;
      gps_serialize_hpposllh(proto->add_hpposllh(), &data->hpposllh);
      timers_gps["hpposllh"] = timestamp;
      break;
    case GPS_UBX_TYPE_NAV_RELPOSNED:
      if (downsample_rate &&
          (1.0e6 / downsample_rate) > (timestamp - timers_gps["relposned"]))
        break;
      gps_serialize_relposned(proto->add_relposned(), &data->relposned);
      timers_gps["relposned"] = timestamp;
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
  for (int i = 0; i < proto->gga_size(); i++) {
    static uint64_t last_timestamp = 0;
    if (proto->gga(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->gga(i)._inner_timestamp();
    (*net_signals)["GGA"]["_timestamp"].push(proto->gga(i)._inner_timestamp());
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
    (*net_strings)["GGA"]["fix_state"].push(
        gps_fix_state_string(proto->gga(i).fix()));
    (*net_signals)["GGA"]["altitude"].push(proto->gga(i).altitude());
    (*net_signals)["GGA"]["age_of_correction"].push(
        proto->gga(i).age_of_correction());
  }
  for (int i = 0; i < proto->vtg_size(); i++) {
    static uint64_t last_timestamp = 0;
    if (proto->vtg(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->vtg(i)._inner_timestamp();
    (*net_signals)["VTG"]["_timestamp"].push(proto->vtg(i)._inner_timestamp());
    (*net_signals)["VTG"]["course_over_ground_degrees"].push(
        proto->vtg(i).course_over_ground_degrees());
    (*net_signals)["VTG"]["course_over_ground_degrees_magnetic"].push(
        proto->vtg(i).course_over_ground_degrees_magnetic());
    (*net_signals)["VTG"]["speed_kmh"].push(proto->vtg(i).speed_kmh());
  }
  for (int i = 0; i < proto->gsa_size(); i++) {
    static uint64_t last_timestamp = 0;
    if (proto->gsa(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->gsa(i)._inner_timestamp();
    (*net_signals)["GSA"]["_timestamp"].push(proto->gsa(i)._inner_timestamp());
    (*net_strings)["GSA"]["mode"].push(proto->gsa(i).mode());
    (*net_signals)["GSA"]["position_diluition_precision"].push(
        proto->gsa(i).position_diluition_precision());
    (*net_signals)["GSA"]["horizontal_diluition_precision"].push(
        proto->gsa(i).horizontal_diluition_precision());
    (*net_signals)["GSA"]["vertical_diluition_precision"].push(
        proto->gsa(i).vertical_diluition_precision());
  }

  for (int i = 0; i < proto->dop_size(); i++) {
    static uint64_t last_timestamp = 0;
    if (proto->dop(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->dop(i)._inner_timestamp();
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
    static uint64_t last_timestamp = 0;
    if (proto->pvt(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->pvt(i)._inner_timestamp();
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
    static uint64_t last_timestamp = 0;
    if (proto->hpposecef(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->hpposecef(i)._inner_timestamp();
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
    static uint64_t last_timestamp = 0;
    if (proto->hpposllh(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->hpposllh(i)._inner_timestamp();
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
  for (int i = 0; i < proto->relposned_size(); i++) {
    static uint64_t last_timestamp = 0;
    if (proto->relposned(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->relposned(i)._inner_timestamp();
    (*net_signals)["RELPOSNED"]["_timestamp"].push(
        proto->relposned(i)._inner_timestamp());
    (*net_signals)["RELPOSNED"]["version"].push(proto->relposned(i).version());
    (*net_signals)["RELPOSNED"]["refStationId"].push(proto->relposned(i).refstationid());
    (*net_signals)["RELPOSNED"]["iTOW"].push(proto->relposned(i).itow());
    (*net_signals)["RELPOSNED"]["relPosN"].push(proto->relposned(i).relposn());
    (*net_signals)["RELPOSNED"]["relPosE"].push(proto->relposned(i).relpose());
    (*net_signals)["RELPOSNED"]["relPosD"].push(proto->relposned(i).relposd());
    (*net_signals)["RELPOSNED"]["relPosLength"].push(proto->relposned(i).relposlength());
    (*net_signals)["RELPOSNED"]["relPosHeading"].push(proto->relposned(i).relposheading());
    (*net_signals)["RELPOSNED"]["relPosHPN"].push(proto->relposned(i).relposhpn());
    (*net_signals)["RELPOSNED"]["relPosHPE"].push(proto->relposned(i).relposhpe());
    (*net_signals)["RELPOSNED"]["relPosHPD"].push(proto->relposned(i).relposhpd());
    (*net_signals)["RELPOSNED"]["relPosHPLength"].push(proto->relposned(i).relposhplength());
    (*net_signals)["RELPOSNED"]["accN"].push(proto->relposned(i).accn());
    (*net_signals)["RELPOSNED"]["accE"].push(proto->relposned(i).acce());
    (*net_signals)["RELPOSNED"]["accD"].push(proto->relposned(i).accd());
    (*net_signals)["RELPOSNED"]["accLength"].push(proto->relposned(i).acclength());
    (*net_signals)["RELPOSNED"]["accHeading"].push(proto->relposned(i).accheading());
    (*net_signals)["RELPOSNED"]["flags    "].push(proto->relposned(i).flags());
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
  proto->set_horizontal_diluition_precision(
      data->horizontal_diluition_precision);
  proto->set_altitude(data->altitude);
  proto->set_age_of_correction(data->age_of_correction);
}
void gps_serialize_vtg(gps::VTG *proto, gps_nmea_vtg_t *data) {
  proto->set__inner_timestamp(data->_timestamp);
  proto->set_course_over_ground_degrees(data->course_over_ground_degrees);
  proto->set_course_over_ground_degrees_magnetic(
      data->course_over_ground_degrees_magnetic);
  proto->set_speed_kmh(data->speed_kmh);
}
void gps_serialize_gsa(gps::GSA *proto, gps_nmea_gsa_t *data) {
  proto->set__inner_timestamp(data->_timestamp);
  proto->set_mode(std::string(data->mode, 1));
  proto->set_position_diluition_precision(data->position_diluition_precision);
  proto->set_horizontal_diluition_precision(
      data->horizontal_diluition_precision);
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
void gps_serialize_hpposecef(gps::NAV_HPPOSECEF *proto,
                             gps_ubx_hpposecef_t *data) {
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
void gps_serialize_hpposllh(gps::NAV_HPPOSLLH *proto,
                            gps_ubx_hpposllh_t *data) {
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
void gps_serialize_relposned(gps::NAV_RELPOSNED *proto,
                             gps_ubx_relposned_t *data) {
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