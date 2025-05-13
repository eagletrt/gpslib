#pragma once

#include <string>
#include <unordered_map>

#include "gps.pb.h"
extern "C" {
#include "gps.h"
}

#include "common_types.h"

#ifndef CANLIB_CIRCULAR_BUFFER
#define CANLIB_CIRCULAR_BUFFER
namespace Helper {
template <bool FITS8, bool FITS16>
struct Index {
  using Type = uint32_t;
};

template <>
struct Index<false, true> {
  using Type = uint16_t;
};

template <>
struct Index<true, true> {
  using Type = uint8_t;
};
}  // namespace Helper

template <typename T, size_t S, typename IT = typename Helper::Index<(S <= UINT8_MAX), (S <= UINT16_MAX)>::Type>
class canlib_circular_buffer {
 public:
  static constexpr IT capacity = static_cast<IT>(S);

  using index_t = IT;

  constexpr canlib_circular_buffer();
  canlib_circular_buffer(const canlib_circular_buffer &) = delete;
  canlib_circular_buffer(canlib_circular_buffer &&) = delete;
  canlib_circular_buffer &operator=(const canlib_circular_buffer &) = delete;
  canlib_circular_buffer &operator=(canlib_circular_buffer &&) = delete;

  bool unshift(T value);
  bool push(T value);
  T shift();
  T pop();
  const T &start() const;
  T inline first() const;
  T inline last() const;
  const T &operator[](IT index) const;
  IT inline size() const;
  IT inline available() const;
  bool inline empty() const;
  bool inline full() const;
  void inline clear();
  size_t inline offset() const;

 private:
  T buffer[S];
  T *head;
  T *tail;
  size_t _offset;
#ifndef CIRCULAR_BUFFER_INT_SAFE
  IT count;
#else
  volatile IT count;
#endif
};

template <typename T, size_t S, typename IT>
constexpr canlib_circular_buffer<T, S, IT>::canlib_circular_buffer()
    : head(buffer), tail(buffer), count(0), _offset(0) {}

template <typename T, size_t S, typename IT>
bool canlib_circular_buffer<T, S, IT>::unshift(T value) {
  if (head == buffer) {
    head = buffer + capacity;
  }
  *--head = value;
  if (count == capacity) {
    if (tail-- == buffer) {
      tail = buffer + capacity - 1;
    }
    return false;
  } else {
    if (count++ == 0) {
      tail = head;
    }
    return true;
  }
}

template <typename T, size_t S, typename IT>
bool canlib_circular_buffer<T, S, IT>::push(T value) {
  if (++tail == buffer + capacity) {
    tail = buffer;
  }
  *tail = value;
  if (count == capacity) {
    if (++head == buffer + capacity) {
      head = buffer;
    }
    _offset = (_offset + 1) % capacity;
    return false;
  } else {
    if (count++ == 0) {
      head = tail;
    }
    return true;
  }
}

template <typename T, size_t S, typename IT>
T canlib_circular_buffer<T, S, IT>::shift() {
  if (count == 0) return *head;
  T result = *head++;
  if (head >= buffer + capacity) {
    head = buffer;
  }
  count--;
  return result;
}

template <typename T, size_t S, typename IT>
T canlib_circular_buffer<T, S, IT>::pop() {
  if (count == 0) return *tail;
  T result = *tail--;
  if (tail < buffer) {
    tail = buffer + capacity - 1;
  }
  count--;
  return result;
}

template <typename T, size_t S, typename IT>
T inline canlib_circular_buffer<T, S, IT>::first() const {
  return *head;
}

template <typename T, size_t S, typename IT>
T inline canlib_circular_buffer<T, S, IT>::last() const {
  return *tail;
}

template <typename T, size_t S, typename IT>
const T &canlib_circular_buffer<T, S, IT>::start() const {
  return buffer[1];
}

template <typename T, size_t S, typename IT>
const T &canlib_circular_buffer<T, S, IT>::operator[](IT index) const {
  if (index >= count) return *tail;
  return *(buffer + ((head - buffer + index) % capacity));
}

template <typename T, size_t S, typename IT>
IT inline canlib_circular_buffer<T, S, IT>::size() const {
  return count;
}

template <typename T, size_t S, typename IT>
IT inline canlib_circular_buffer<T, S, IT>::available() const {
  return capacity - count;
}

template <typename T, size_t S, typename IT>
bool inline canlib_circular_buffer<T, S, IT>::empty() const {
  return count == 0;
}

template <typename T, size_t S, typename IT>
bool inline canlib_circular_buffer<T, S, IT>::full() const {
  return count == capacity;
}

template <typename T, size_t S, typename IT>
void inline canlib_circular_buffer<T, S, IT>::clear() {
  head = tail = buffer;
  count = 0;
}

template <typename T, size_t S, typename IT>
size_t inline canlib_circular_buffer<T, S, IT>::offset() const {
  return _offset;
}

#endif  // CANLIB_CIRCULAR_BUFFER

#ifndef CANLIB_CIRCULAR_BUFFER_SIZE
#define CANLIB_CIRCULAR_BUFFER_SIZE 2000
#endif  // CANLIB_CIRCULAR_BUFFER_SIZE

static std::unordered_map<std::string, uint64_t> timers_gps;

typedef struct gps_proto_pack {
  // NMEA
  canlib_circular_buffer<gps_nmea_gga_t, CANLIB_CIRCULAR_BUFFER_SIZE> gga;
  canlib_circular_buffer<gps_nmea_vtg_t, CANLIB_CIRCULAR_BUFFER_SIZE> vtg;
  canlib_circular_buffer<gps_nmea_gsa_t, CANLIB_CIRCULAR_BUFFER_SIZE> gsa;
  // UBX
  canlib_circular_buffer<gps_ubx_dop_t, CANLIB_CIRCULAR_BUFFER_SIZE> dop;
  canlib_circular_buffer<gps_ubx_pvt_t, CANLIB_CIRCULAR_BUFFER_SIZE> pvt;
  canlib_circular_buffer<gps_ubx_hpposecef_t, CANLIB_CIRCULAR_BUFFER_SIZE> hpposecef;
  canlib_circular_buffer<gps_ubx_hpposllh_t, CANLIB_CIRCULAR_BUFFER_SIZE> hpposllh;
  canlib_circular_buffer<gps_ubx_relposned_t, CANLIB_CIRCULAR_BUFFER_SIZE> relposned;
  canlib_circular_buffer<gps_ubx_velned_t, CANLIB_CIRCULAR_BUFFER_SIZE> velened;

  std::unordered_map<std::string, uint64_t> timers;
} gps_proto_pack;

void gps_proto_serialize_from_match(gps_protocol_and_message &match, gps::GpsPack *proto, gps_parsed_data_t *data,
                                    uint64_t &timestamp, uint64_t downsample_rate);

template <Uint64Buffer uint_buffer, FloatBuffer double_buffer, StringBuffer string_buffer>
void gps_proto_deserialize(gps::GpsPack *proto, net_enums<uint_buffer> &net_enums,
                           net_signals<double_buffer> &net_signals, net_strings<string_buffer> &net_strings,
                           uint64_t resample_us);

void gps_serialize_gga(gps::GGA *proto, gps_nmea_gga_t *data);
void gps_serialize_vtg(gps::VTG *proto, gps_nmea_vtg_t *data);
void gps_serialize_gsa(gps::GSA *proto, gps_nmea_gsa_t *data);

void gps_serialize_dop(gps::NAV_DOP *proto, gps_ubx_dop_t *data);
void gps_serialize_pvt(gps::NAV_PVT *proto, gps_ubx_pvt_t *data);
void gps_serialize_hpposecef(gps::NAV_HPPOSECEF *proto, gps_ubx_hpposecef_t *data);
void gps_serialize_hpposllh(gps::NAV_HPPOSLLH *proto, gps_ubx_hpposllh_t *data);
void gps_serialize_relposned(gps::NAV_RELPOSNED *proto, gps_ubx_relposned_t *data);
void gps_serialize_velned(gps::NAV_VELNED *proto, gps_ubx_velned_t *data);

template <Uint64Buffer uint_buffer, FloatBuffer double_buffer, StringBuffer string_buffer>
void gps_proto_deserialize(gps::GpsPack *proto, net_enums<uint_buffer> &net_enums,
                           net_signals<double_buffer> &net_signals, net_strings<string_buffer> &net_strings,
                           uint64_t resample_us) {
  for (int i = 0; i < proto->gga_size(); i++) {
    static uint64_t last_timestamp = 0;
    if (proto->gga(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->gga(i)._inner_timestamp();
    pushImpl(net_signals["GGA"]["_timestamp"], proto->gga(i)._inner_timestamp());
    pushImpl(net_strings["GGA"]["time"], std::string(proto->gga(i).time().c_str(), 9));
    pushImpl(net_signals["GGA"]["latitude"], proto->gga(i).latitude());
    pushImpl(net_strings["GGA"]["north_south"], proto->gga(i).north_south());
    pushImpl(net_signals["GGA"]["longitude"], proto->gga(i).longitude());
    pushImpl(net_strings["GGA"]["east_ovest"], proto->gga(i).east_ovest());
    pushImpl(net_signals["GGA"]["fix"], proto->gga(i).fix());
    pushImpl(net_signals["GGA"]["satellites"], proto->gga(i).satellites());
    pushImpl(net_signals["GGA"]["horizontal_diluition_precision"], proto->gga(i).horizontal_diluition_precision());
    pushImpl(net_enums["GGA"]["fix_state"], proto->gga(i).fix());
    pushImpl(net_strings["GGA"]["fix_state"], gps_fix_state_string(proto->gga(i).fix()));
    pushImpl(net_signals["GGA"]["altitude"], proto->gga(i).altitude());
    pushImpl(net_signals["GGA"]["age_of_correction"], proto->gga(i).age_of_correction());
  }
  for (int i = 0; i < proto->vtg_size(); i++) {
    static uint64_t last_timestamp = 0;
    if (proto->vtg(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->vtg(i)._inner_timestamp();
    pushImpl(net_signals["VTG"]["_timestamp"], proto->vtg(i)._inner_timestamp());
    pushImpl(net_signals["VTG"]["course_over_ground_degrees"], proto->vtg(i).course_over_ground_degrees());
    pushImpl(net_signals["VTG"]["course_over_ground_degrees_magnetic"],
             proto->vtg(i).course_over_ground_degrees_magnetic());
    pushImpl(net_signals["VTG"]["speed_kmh"], proto->vtg(i).speed_kmh());
  }
  for (int i = 0; i < proto->gsa_size(); i++) {
    static uint64_t last_timestamp = 0;
    if (proto->gsa(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->gsa(i)._inner_timestamp();
    pushImpl(net_signals["GSA"]["_timestamp"], proto->gsa(i)._inner_timestamp());
    pushImpl(net_strings["GSA"]["mode"], proto->gsa(i).mode());
    pushImpl(net_signals["GSA"]["position_diluition_precision"], proto->gsa(i).position_diluition_precision());
    pushImpl(net_signals["GSA"]["horizontal_diluition_precision"], proto->gsa(i).horizontal_diluition_precision());
    pushImpl(net_signals["GSA"]["vertical_diluition_precision"], proto->gsa(i).vertical_diluition_precision());
  }

  for (int i = 0; i < proto->dop_size(); i++) {
    static uint64_t last_timestamp = 0;
    if (proto->dop(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->dop(i)._inner_timestamp();

    pushImpl(net_signals["DOP"]["_timestamp"], proto->dop(i)._inner_timestamp());
    pushImpl(net_signals["DOP"]["iTOW"], proto->dop(i).itow());
    pushImpl(net_signals["DOP"]["gDOP"], proto->dop(i).gdop());
    pushImpl(net_signals["DOP"]["pDOP"], proto->dop(i).pdop());
    pushImpl(net_signals["DOP"]["tDOP"], proto->dop(i).tdop());
    pushImpl(net_signals["DOP"]["vDOP"], proto->dop(i).vdop());
    pushImpl(net_signals["DOP"]["hDOP"], proto->dop(i).hdop());
    pushImpl(net_signals["DOP"]["nDOP"], proto->dop(i).ndop());
    pushImpl(net_signals["DOP"]["eDOP"], proto->dop(i).edop());
  }
  for (int i = 0; i < proto->pvt_size(); i++) {
    static uint64_t last_timestamp = 0;
    if (proto->pvt(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->pvt(i)._inner_timestamp();
    pushImpl(net_signals["PVT"]["_timestamp"], proto->pvt(i)._inner_timestamp());
    pushImpl(net_signals["PVT"]["iTOW"], proto->pvt(i).itow());
    pushImpl(net_signals["PVT"]["year"], proto->pvt(i).year());
    pushImpl(net_signals["PVT"]["month"], proto->pvt(i).month());
    pushImpl(net_signals["PVT"]["day"], proto->pvt(i).day());
    pushImpl(net_signals["PVT"]["hour"], proto->pvt(i).hour());
    pushImpl(net_signals["PVT"]["min"], proto->pvt(i).min());
    pushImpl(net_signals["PVT"]["sec"], proto->pvt(i).sec());
    pushImpl(net_signals["PVT"]["valid"], proto->pvt(i).valid());
    pushImpl(net_signals["PVT"]["tAcc"], proto->pvt(i).tacc());
    pushImpl(net_signals["PVT"]["nano"], proto->pvt(i).nano());
    pushImpl(net_signals["PVT"]["fixType"], proto->pvt(i).fixtype());
    pushImpl(net_signals["PVT"]["flags"], proto->pvt(i).flags());
    pushImpl(net_signals["PVT"]["flags2"], proto->pvt(i).flags2());
    pushImpl(net_signals["PVT"]["numSV"], proto->pvt(i).numsv());
    pushImpl(net_signals["PVT"]["lon"], proto->pvt(i).lon());
    pushImpl(net_signals["PVT"]["lat"], proto->pvt(i).lat());
    pushImpl(net_signals["PVT"]["height"], proto->pvt(i).height());
    pushImpl(net_signals["PVT"]["hMSL"], proto->pvt(i).hmsl());
    pushImpl(net_signals["PVT"]["hAcc"], proto->pvt(i).hacc());
    pushImpl(net_signals["PVT"]["vAcc"], proto->pvt(i).vacc());
    pushImpl(net_signals["PVT"]["velN"], proto->pvt(i).veln());
    pushImpl(net_signals["PVT"]["velE"], proto->pvt(i).vele());
    pushImpl(net_signals["PVT"]["velD"], proto->pvt(i).veld());
    pushImpl(net_signals["PVT"]["gSpeed"], proto->pvt(i).gspeed());
    pushImpl(net_signals["PVT"]["headMot"], proto->pvt(i).headmot());
    pushImpl(net_signals["PVT"]["sAcc"], proto->pvt(i).sacc());
    pushImpl(net_signals["PVT"]["headAcc"], proto->pvt(i).headacc());
    pushImpl(net_signals["PVT"]["pDOP"], proto->pvt(i).pdop());
    pushImpl(net_signals["PVT"]["headVeh"], proto->pvt(i).headveh());
    pushImpl(net_signals["PVT"]["magDec"], proto->pvt(i).magdec());
    pushImpl(net_signals["PVT"]["magAcc"], proto->pvt(i).magacc());
  }
  for (int i = 0; i < proto->hpposecef_size(); i++) {
    static uint64_t last_timestamp = 0;
    if (proto->hpposecef(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->hpposecef(i)._inner_timestamp();
    pushImpl(net_signals["HPPOSECEF"]["_timestamp"], proto->hpposecef(i)._inner_timestamp());
    pushImpl(net_signals["HPPOSECEF"]["version"], proto->hpposecef(i).version());
    pushImpl(net_signals["HPPOSECEF"]["iTOW"], proto->hpposecef(i).itow());
    pushImpl(net_signals["HPPOSECEF"]["ecefX"], proto->hpposecef(i).ecefx());
    pushImpl(net_signals["HPPOSECEF"]["ecefY"], proto->hpposecef(i).ecefy());
    pushImpl(net_signals["HPPOSECEF"]["ecefZ"], proto->hpposecef(i).ecefz());
    pushImpl(net_signals["HPPOSECEF"]["ecefXHp"], proto->hpposecef(i).ecefxhp());
    pushImpl(net_signals["HPPOSECEF"]["ecefYHp"], proto->hpposecef(i).ecefyhp());
    pushImpl(net_signals["HPPOSECEF"]["ecefZHp"], proto->hpposecef(i).ecefzhp());
    pushImpl(net_signals["HPPOSECEF"]["pAcc"], proto->hpposecef(i).pacc());
  }
  for (int i = 0; i < proto->hpposllh_size(); i++) {
    static uint64_t last_timestamp = 0;
    if (proto->hpposllh(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->hpposllh(i)._inner_timestamp();
    pushImpl(net_signals["HPPOSLLH"]["_timestamp"], proto->hpposllh(i)._inner_timestamp());
    pushImpl(net_signals["HPPOSLLH"]["version"], proto->hpposllh(i).version());
    pushImpl(net_signals["HPPOSLLH"]["iTOW"], proto->hpposllh(i).itow());
    pushImpl(net_signals["HPPOSLLH"]["lon"], proto->hpposllh(i).lon());
    pushImpl(net_signals["HPPOSLLH"]["lat"], proto->hpposllh(i).lat());
    pushImpl(net_signals["HPPOSLLH"]["height"], proto->hpposllh(i).height());
    pushImpl(net_signals["HPPOSLLH"]["hMSL"], proto->hpposllh(i).hmsl());
    pushImpl(net_signals["HPPOSLLH"]["lonHp"], proto->hpposllh(i).lonhp());
    pushImpl(net_signals["HPPOSLLH"]["latHp"], proto->hpposllh(i).lathp());
    pushImpl(net_signals["HPPOSLLH"]["heightHp"], proto->hpposllh(i).heighthp());
    pushImpl(net_signals["HPPOSLLH"]["hMSLHp"], proto->hpposllh(i).hmslhp());
    pushImpl(net_signals["HPPOSLLH"]["hAcc"], proto->hpposllh(i).hacc());
    pushImpl(net_signals["HPPOSLLH"]["vAcc"], proto->hpposllh(i).vacc());
  }
  for (int i = 0; i < proto->relposned_size(); i++) {
    static uint64_t last_timestamp = 0;
    if (proto->relposned(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->relposned(i)._inner_timestamp();
    pushImpl(net_signals["RELPOSNED"]["_timestamp"], proto->relposned(i)._inner_timestamp());
    pushImpl(net_signals["RELPOSNED"]["version"], proto->relposned(i).version());
    pushImpl(net_signals["RELPOSNED"]["refStationId"], proto->relposned(i).refstationid());
    pushImpl(net_signals["RELPOSNED"]["iTOW"], proto->relposned(i).itow());
    pushImpl(net_signals["RELPOSNED"]["relPosN"], proto->relposned(i).relposn());
    pushImpl(net_signals["RELPOSNED"]["relPosE"], proto->relposned(i).relpose());
    pushImpl(net_signals["RELPOSNED"]["relPosD"], proto->relposned(i).relposd());
    pushImpl(net_signals["RELPOSNED"]["relPosLength"], proto->relposned(i).relposlength());
    pushImpl(net_signals["RELPOSNED"]["relPosHeading"], proto->relposned(i).relposheading());
    pushImpl(net_signals["RELPOSNED"]["relPosHPN"], proto->relposned(i).relposhpn());
    pushImpl(net_signals["RELPOSNED"]["relPosHPE"], proto->relposned(i).relposhpe());
    pushImpl(net_signals["RELPOSNED"]["relPosHPD"], proto->relposned(i).relposhpd());
    pushImpl(net_signals["RELPOSNED"]["relPosHPLength"], proto->relposned(i).relposhplength());
    pushImpl(net_signals["RELPOSNED"]["accN"], proto->relposned(i).accn());
    pushImpl(net_signals["RELPOSNED"]["accE"], proto->relposned(i).acce());
    pushImpl(net_signals["RELPOSNED"]["accD"], proto->relposned(i).accd());
    pushImpl(net_signals["RELPOSNED"]["accLength"], proto->relposned(i).acclength());
    pushImpl(net_signals["RELPOSNED"]["accHeading"], proto->relposned(i).accheading());
    pushImpl(net_signals["RELPOSNED"]["flags    "], proto->relposned(i).flags());
  }
  for (int i = 0; i < proto->velned_size(); i++) {
    static uint64_t last_timestamp = 0;
    if (proto->velned(i)._inner_timestamp() - last_timestamp < resample_us)
      continue;
    else
      last_timestamp = proto->velned(i)._inner_timestamp();
    pushImpl(net_signals["VELNED"]["_timestamp"], proto->velned(i)._inner_timestamp());
    pushImpl(net_signals["VELNED"]["iTOW"], proto->velned(i).itow());
    pushImpl(net_signals["VELNED"]["velN"], proto->velned(i).veln());
    pushImpl(net_signals["VELNED"]["velE"], proto->velned(i).vele());
    pushImpl(net_signals["VELNED"]["velD"], proto->velned(i).veld());
    pushImpl(net_signals["VELNED"]["speed"], proto->velned(i).speed());
    pushImpl(net_signals["VELNED"]["gSpeed"], proto->velned(i).gspeed());
    pushImpl(net_signals["VELNED"]["heading"], proto->velned(i).heading());
    pushImpl(net_signals["VELNED"]["sAcc"], proto->velned(i).sacc());
    pushImpl(net_signals["VELNED"]["cAcc"], proto->velned(i).cacc());
  }
}
