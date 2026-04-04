#include "gps_proto.h"

#include <string.h>
#include <atomic>

#include <functional>

void gps_proto_serialize_from_match(gps_protocol_and_message &match,
                                    gps::GpsPack *proto,
                                    gps_parsed_data_t *data,
                                    uint64_t &timestamp,
                                    uint64_t downsample_rate) {
  switch (match.protocol) {
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
        case GPS_UBX_TYPE_NAV_VELNED:
          if (downsample_rate &&
              (1.0e6 / downsample_rate) > (timestamp - timers_gps["velned"]))
            break;
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

void gps_proto_deserialize(gps::GpsPack *proto, network_enums *net_enums,
                           network_signals *net_signals,
                           network_strings *net_strings, uint64_t resample_us) {
  std::string antenna_name = proto->antenna_name();

  // static to avoid reinitialization on every call
  static std::atomic<uint32_t> unnamed_counter{0};

  // fallback in case antenna name is not set
  // we want to avoid all messages being grouped together in the same key
  if (antenna_name.empty()) {
    antenna_name =
        std::string("GPS_") + std::to_string(unnamed_counter.fetch_add(1) + 1);
  }

  // map to keep track of last timestamps for each message type to implement
  // downsampling
  static std::unordered_map<std::string, uint64_t> last_timestamps;

  // lambda to check if a sample should be skipped based on the resample_us
  // parameter
  auto shouldSkipSample = [&](const char *message_key,
                              uint64_t sample_timestamp) -> bool {
    if (resample_us == 0) {
      return false;
    }
    const std::string key = antenna_name + "_" + message_key;
    uint64_t &last_timestamp = last_timestamps[key];
    if (last_timestamp != 0 && sample_timestamp >= last_timestamp &&
        (sample_timestamp - last_timestamp) < resample_us) {
      return true;
    }
    last_timestamp = sample_timestamp;
    return false;
  };

  for (int i = 0; i < proto->gga_size(); i++) {
    if (shouldSkipSample("GGA", proto->gga(i)._inner_timestamp())) continue;
    (*net_signals)[antenna_name + "_GGA"]["_timestamp"].push(
        proto->gga(i)._inner_timestamp());
    (*net_strings)[antenna_name + "_GGA"]["time"].push(
        std::string(proto->gga(i).time().c_str(), 9));
    (*net_signals)[antenna_name + "_GGA"]["latitude"].push(
        proto->gga(i).latitude());
    (*net_strings)[antenna_name + "_GGA"]["north_south"].push(
        proto->gga(i).north_south());
    (*net_signals)[antenna_name + "_GGA"]["longitude"].push(
        proto->gga(i).longitude());
    (*net_strings)[antenna_name + "_GGA"]["east_ovest"].push(
        proto->gga(i).east_ovest());
    (*net_signals)[antenna_name + "_GGA"]["fix"].push(proto->gga(i).fix());
    (*net_signals)[antenna_name + "_GGA"]["satellites"].push(
        proto->gga(i).satellites());
    (*net_signals)[antenna_name + "_GGA"]["horizontal_diluition_precision"]
        .push(proto->gga(i).horizontal_diluition_precision());
    (*net_enums)[antenna_name + "_GGA"]["fix_state"].push(proto->gga(i).fix());
    (*net_strings)[antenna_name + "_GGA"]["fix_state"].push(
        gps_fix_state_string(proto->gga(i).fix()));
    (*net_signals)[antenna_name + "_GGA"]["altitude"].push(
        proto->gga(i).altitude());
    (*net_signals)[antenna_name + "_GGA"]["age_of_correction"].push(
        proto->gga(i).age_of_correction());
  }
  for (int i = 0; i < proto->vtg_size(); i++) {
    if (shouldSkipSample("VTG", proto->vtg(i)._inner_timestamp())) continue;
    (*net_signals)[antenna_name + "_VTG"]["_timestamp"].push(
        proto->vtg(i)._inner_timestamp());
    (*net_signals)[antenna_name + "_VTG"]["course_over_ground_degrees"].push(
        proto->vtg(i).course_over_ground_degrees());
    (*net_signals)[antenna_name + "_VTG"]["course_over_ground_degrees_magnetic"]
        .push(proto->vtg(i).course_over_ground_degrees_magnetic());
    (*net_signals)[antenna_name + "_VTG"]["speed_kmh"].push(
        proto->vtg(i).speed_kmh());
  }
  for (int i = 0; i < proto->gsa_size(); i++) {
    if (shouldSkipSample("GSA", proto->gsa(i)._inner_timestamp())) continue;
    (*net_signals)[antenna_name + "_GSA"]["_timestamp"].push(
        proto->gsa(i)._inner_timestamp());
    (*net_strings)[antenna_name + "_GSA"]["mode"].push(proto->gsa(i).mode());
    (*net_signals)[antenna_name + "_GSA"]["position_diluition_precision"].push(
        proto->gsa(i).position_diluition_precision());
    (*net_signals)[antenna_name + "_GSA"]["horizontal_diluition_precision"]
        .push(proto->gsa(i).horizontal_diluition_precision());
    (*net_signals)[antenna_name + "_GSA"]["vertical_diluition_precision"].push(
        proto->gsa(i).vertical_diluition_precision());
  }

  for (int i = 0; i < proto->dop_size(); i++) {
    if (shouldSkipSample("DOP", proto->dop(i)._inner_timestamp())) continue;
    (*net_signals)[antenna_name + "_DOP"]["_timestamp"].push(
        proto->dop(i)._inner_timestamp());
    (*net_signals)[antenna_name + "_DOP"]["iTOW"].push(proto->dop(i).itow());
    (*net_signals)[antenna_name + "_DOP"]["gDOP"].push(proto->dop(i).gdop());
    (*net_signals)[antenna_name + "_DOP"]["pDOP"].push(proto->dop(i).pdop());
    (*net_signals)[antenna_name + "_DOP"]["tDOP"].push(proto->dop(i).tdop());
    (*net_signals)[antenna_name + "_DOP"]["vDOP"].push(proto->dop(i).vdop());
    (*net_signals)[antenna_name + "_DOP"]["hDOP"].push(proto->dop(i).hdop());
    (*net_signals)[antenna_name + "_DOP"]["nDOP"].push(proto->dop(i).ndop());
    (*net_signals)[antenna_name + "_DOP"]["eDOP"].push(proto->dop(i).edop());
  }
  for (int i = 0; i < proto->pvt_size(); i++) {
    if (shouldSkipSample("PVT", proto->pvt(i)._inner_timestamp())) continue;
    (*net_signals)[antenna_name + "_PVT"]["_timestamp"].push(
        proto->pvt(i)._inner_timestamp());
    (*net_signals)[antenna_name + "_PVT"]["iTOW"].push(proto->pvt(i).itow());
    (*net_signals)[antenna_name + "_PVT"]["year"].push(proto->pvt(i).year());
    (*net_signals)[antenna_name + "_PVT"]["month"].push(proto->pvt(i).month());
    (*net_signals)[antenna_name + "_PVT"]["day"].push(proto->pvt(i).day());
    (*net_signals)[antenna_name + "_PVT"]["hour"].push(proto->pvt(i).hour());
    (*net_signals)[antenna_name + "_PVT"]["min"].push(proto->pvt(i).min());
    (*net_signals)[antenna_name + "_PVT"]["sec"].push(proto->pvt(i).sec());
    (*net_signals)[antenna_name + "_PVT"]["valid"].push(proto->pvt(i).valid());
    (*net_signals)[antenna_name + "_PVT"]["tAcc"].push(proto->pvt(i).tacc());
    (*net_signals)[antenna_name + "_PVT"]["nano"].push(proto->pvt(i).nano());
    (*net_signals)[antenna_name + "_PVT"]["fixType"].push(
        proto->pvt(i).fixtype());
    (*net_signals)[antenna_name + "_PVT"]["flags"].push(proto->pvt(i).flags());
    (*net_signals)[antenna_name + "_PVT"]["flags2"].push(
        proto->pvt(i).flags2());
    (*net_signals)[antenna_name + "_PVT"]["numSV"].push(proto->pvt(i).numsv());
    (*net_signals)[antenna_name + "_PVT"]["lon"].push(proto->pvt(i).lon());
    (*net_signals)[antenna_name + "_PVT"]["lat"].push(proto->pvt(i).lat());
    (*net_signals)[antenna_name + "_PVT"]["height"].push(
        proto->pvt(i).height());
    (*net_signals)[antenna_name + "_PVT"]["hMSL"].push(proto->pvt(i).hmsl());
    (*net_signals)[antenna_name + "_PVT"]["hAcc"].push(proto->pvt(i).hacc());
    (*net_signals)[antenna_name + "_PVT"]["vAcc"].push(proto->pvt(i).vacc());
    (*net_signals)[antenna_name + "_PVT"]["velN"].push(proto->pvt(i).veln());
    (*net_signals)[antenna_name + "_PVT"]["velE"].push(proto->pvt(i).vele());
    (*net_signals)[antenna_name + "_PVT"]["velD"].push(proto->pvt(i).veld());
    (*net_signals)[antenna_name + "_PVT"]["gSpeed"].push(
        proto->pvt(i).gspeed());
    (*net_signals)[antenna_name + "_PVT"]["headMot"].push(
        proto->pvt(i).headmot());
    (*net_signals)[antenna_name + "_PVT"]["sAcc"].push(proto->pvt(i).sacc());
    (*net_signals)[antenna_name + "_PVT"]["headAcc"].push(
        proto->pvt(i).headacc());
    (*net_signals)[antenna_name + "_PVT"]["pDOP"].push(proto->pvt(i).pdop());
    (*net_signals)[antenna_name + "_PVT"]["headVeh"].push(
        proto->pvt(i).headveh());
    (*net_signals)[antenna_name + "_PVT"]["magDec"].push(
        proto->pvt(i).magdec());
    (*net_signals)[antenna_name + "_PVT"]["magAcc"].push(
        proto->pvt(i).magacc());
  }
  for (int i = 0; i < proto->hpposecef_size(); i++) {
    if (shouldSkipSample("HPPOSECEF", proto->hpposecef(i)._inner_timestamp()))
      continue;
    (*net_signals)[antenna_name + "_HPPOSECEF"]["_timestamp"].push(
        proto->hpposecef(i)._inner_timestamp());
    (*net_signals)[antenna_name + "_HPPOSECEF"]["version"].push(
        proto->hpposecef(i).version());
    (*net_signals)[antenna_name + "_HPPOSECEF"]["iTOW"].push(
        proto->hpposecef(i).itow());
    (*net_signals)[antenna_name + "_HPPOSECEF"]["ecefX"].push(
        proto->hpposecef(i).ecefx());
    (*net_signals)[antenna_name + "_HPPOSECEF"]["ecefY"].push(
        proto->hpposecef(i).ecefy());
    (*net_signals)[antenna_name + "_HPPOSECEF"]["ecefZ"].push(
        proto->hpposecef(i).ecefz());
    (*net_signals)[antenna_name + "_HPPOSECEF"]["ecefXHp"].push(
        proto->hpposecef(i).ecefxhp());
    (*net_signals)[antenna_name + "_HPPOSECEF"]["ecefYHp"].push(
        proto->hpposecef(i).ecefyhp());
    (*net_signals)[antenna_name + "_HPPOSECEF"]["ecefZHp"].push(
        proto->hpposecef(i).ecefzhp());
    (*net_signals)[antenna_name + "_HPPOSECEF"]["pAcc"].push(
        proto->hpposecef(i).pacc());
  }
  for (int i = 0; i < proto->hpposllh_size(); i++) {
    if (shouldSkipSample("HPPOSLLH", proto->hpposllh(i)._inner_timestamp()))
      continue;
    (*net_signals)[antenna_name + "_HPPOSLLH"]["_timestamp"].push(
        proto->hpposllh(i)._inner_timestamp());
    (*net_signals)[antenna_name + "_HPPOSLLH"]["version"].push(
        proto->hpposllh(i).version());
    (*net_signals)[antenna_name + "_HPPOSLLH"]["iTOW"].push(
        proto->hpposllh(i).itow());
    (*net_signals)[antenna_name + "_HPPOSLLH"]["lon"].push(
        proto->hpposllh(i).lon());
    (*net_signals)[antenna_name + "_HPPOSLLH"]["lat"].push(
        proto->hpposllh(i).lat());
    (*net_signals)[antenna_name + "_HPPOSLLH"]["height"].push(
        proto->hpposllh(i).height());
    (*net_signals)[antenna_name + "_HPPOSLLH"]["hMSL"].push(
        proto->hpposllh(i).hmsl());
    (*net_signals)[antenna_name + "_HPPOSLLH"]["lonHp"].push(
        proto->hpposllh(i).lonhp());
    (*net_signals)[antenna_name + "_HPPOSLLH"]["latHp"].push(
        proto->hpposllh(i).lathp());
    (*net_signals)[antenna_name + "_HPPOSLLH"]["heightHp"].push(
        proto->hpposllh(i).heighthp());
    (*net_signals)[antenna_name + "_HPPOSLLH"]["hMSLHp"].push(
        proto->hpposllh(i).hmslhp());
    (*net_signals)[antenna_name + "_HPPOSLLH"]["hAcc"].push(
        proto->hpposllh(i).hacc());
    (*net_signals)[antenna_name + "_HPPOSLLH"]["vAcc"].push(
        proto->hpposllh(i).vacc());
  }
  for (int i = 0; i < proto->relposned_size(); i++) {
    if (shouldSkipSample("RELPOSNED", proto->relposned(i)._inner_timestamp()))
      continue;
    (*net_signals)[antenna_name + "_RELPOSNED"]["_timestamp"].push(
        proto->relposned(i)._inner_timestamp());
    (*net_signals)[antenna_name + "_RELPOSNED"]["version"].push(
        proto->relposned(i).version());
    (*net_signals)[antenna_name + "_RELPOSNED"]["refStationId"].push(
        proto->relposned(i).refstationid());
    (*net_signals)[antenna_name + "_RELPOSNED"]["iTOW"].push(
        proto->relposned(i).itow());
    (*net_signals)[antenna_name + "_RELPOSNED"]["relPosN"].push(
        proto->relposned(i).relposn());
    (*net_signals)[antenna_name + "_RELPOSNED"]["relPosE"].push(
        proto->relposned(i).relpose());
    (*net_signals)[antenna_name + "_RELPOSNED"]["relPosD"].push(
        proto->relposned(i).relposd());
    (*net_signals)[antenna_name + "_RELPOSNED"]["relPosLength"].push(
        proto->relposned(i).relposlength());
    (*net_signals)[antenna_name + "_RELPOSNED"]["relPosHeading"].push(
        proto->relposned(i).relposheading());
    (*net_signals)[antenna_name + "_RELPOSNED"]["relPosHPN"].push(
        proto->relposned(i).relposhpn());
    (*net_signals)[antenna_name + "_RELPOSNED"]["relPosHPE"].push(
        proto->relposned(i).relposhpe());
    (*net_signals)[antenna_name + "_RELPOSNED"]["relPosHPD"].push(
        proto->relposned(i).relposhpd());
    (*net_signals)[antenna_name + "_RELPOSNED"]["relPosHPLength"].push(
        proto->relposned(i).relposhplength());
    (*net_signals)[antenna_name + "_RELPOSNED"]["accN"].push(
        proto->relposned(i).accn());
    (*net_signals)[antenna_name + "_RELPOSNED"]["accE"].push(
        proto->relposned(i).acce());
    (*net_signals)[antenna_name + "_RELPOSNED"]["accD"].push(
        proto->relposned(i).accd());
    (*net_signals)[antenna_name + "_RELPOSNED"]["accLength"].push(
        proto->relposned(i).acclength());
    (*net_signals)[antenna_name + "_RELPOSNED"]["accHeading"].push(
        proto->relposned(i).accheading());
    (*net_signals)[antenna_name + "_RELPOSNED"]["flags    "].push(
        proto->relposned(i).flags());
  }
  for (int i = 0; i < proto->velned_size(); i++) {
    if (shouldSkipSample("VELNED", proto->velned(i)._inner_timestamp()))
      continue;
    (*net_signals)[antenna_name + "_VELNED"]["_timestamp"].push(
        proto->velned(i)._inner_timestamp());
    (*net_signals)[antenna_name + "_VELNED"]["iTOW"].push(
        proto->velned(i).itow());
    (*net_signals)[antenna_name + "_VELNED"]["velN"].push(
        proto->velned(i).veln());
    (*net_signals)[antenna_name + "_VELNED"]["velE"].push(
        proto->velned(i).vele());
    (*net_signals)[antenna_name + "_VELNED"]["velD"].push(
        proto->velned(i).veld());
    (*net_signals)[antenna_name + "_VELNED"]["speed"].push(
        proto->velned(i).speed());
    (*net_signals)[antenna_name + "_VELNED"]["gSpeed"].push(
        proto->velned(i).gspeed());
    (*net_signals)[antenna_name + "_VELNED"]["heading"].push(
        proto->velned(i).heading());
    (*net_signals)[antenna_name + "_VELNED"]["sAcc"].push(
        proto->velned(i).sacc());
    (*net_signals)[antenna_name + "_VELNED"]["cAcc"].push(
        proto->velned(i).cacc());
  }
  for (int i = 0; i < proto->heading_size(); i++) {
    if (shouldSkipSample("HEADING", proto->heading(i)._inner_timestamp()))
      continue;
    (*net_signals)[antenna_name + "_HEADING"]["_timestamp"].push(
        proto->heading(i)._inner_timestamp());
    (*net_signals)[antenna_name + "_HEADING"]["iTOW"].push(
        proto->heading(i).itow());
    (*net_signals)[antenna_name + "_HEADING"]["heading"].push(
        proto->heading(i).heading());
    (*net_signals)[antenna_name + "_HEADING"]["heading_stddev"].push(
        proto->heading(i).heading_stddev());
    (*net_signals)[antenna_name + "_HEADING"]["baseline"].push(
        proto->heading(i).baseline());
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

void gps_serialize_heading(gps::HEADING *proto, gps_heading_t *data) {
  proto->set__inner_timestamp(data->_timestamp);
  proto->set_itow(data->iTOW);
  proto->set_heading(data->heading);
  proto->set_heading_stddev(data->heading_stddev);
  proto->set_baseline(data->baseline);
}