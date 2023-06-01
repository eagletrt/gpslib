#pragma once

#include <unordered_map>
#include <string>
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

template <typename T, size_t S,
          typename IT =
              typename Helper::Index<(S <= UINT8_MAX), (S <= UINT16_MAX)>::Type>
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
  const T& start() const;
  T inline first() const;
  T inline last() const;
  const T& operator[](IT index) const;
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
const T& canlib_circular_buffer<T, S, IT>::start() const {
  return buffer[1];
}

template <typename T, size_t S, typename IT>
const T& canlib_circular_buffer<T, S, IT>::operator[](IT index) const {
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

#endif // CANLIB_CIRCULAR_BUFFER

#ifndef CANLIB_CIRCULAR_BUFFER_SIZE
#define CANLIB_CIRCULAR_BUFFER_SIZE 2000
#endif//CANLIB_CIRCULAR_BUFFER_SIZE

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