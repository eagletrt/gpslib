#ifndef GPSLIB_COMMON_TYPES_H
#define GPSLIB_COMMON_TYPES_H

#ifndef CANLIB_CIRCULAR_BUFFER
#define CANLIB_CIRCULAR_BUFFER
#include <inttypes.h>
#include <stdlib.h>

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
    : head(buffer), tail(buffer), _offset(0), count(0) {}

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

#ifndef CANLIB_PROTO_INTERFACE_TYPES
#define CANLIB_PROTO_INTERFACE_TYPES

#include <concepts>
#include <string>
#include <unordered_map>

/**
 *  Use network_<> to get all the values from the protobuffer.
 *  Every network can be consensed into one network_<> as all the
 *  messages names are unique.
 **/
template <typename T, typename U>
concept HasValueTypeOf = requires { typename T::value_type; } && std::same_as<typename T::value_type, U>;

template <typename T>
concept Pushable = requires(T container, typename T::value_type value) {
  { container.push(value) } -> std::same_as<void>;
};
template <typename T>
concept PushBackable = requires(T container, typename T::value_type value) {
  { container.push_back(value) } -> std::same_as<void>;
};

template <typename T>
concept Buffer = requires { typename T::value_type; } && (Pushable<T> || PushBackable<T>);

template <typename T>
concept Uint64Buffer = Buffer<T> && HasValueTypeOf<T, uint64_t>;
template <typename T>
concept FloatBuffer = Buffer<T> && std::is_floating_point_v<typename T::value_type>;
template <typename T>
concept StringBuffer = Buffer<T> && HasValueTypeOf<T, std::string>;

using field_name = std::string;
using messages_name = std::string;

// structure contains all the messages with a enum value associated
template <Uint64Buffer uint_buffer>
using msg_enums = std::unordered_map<field_name, uint_buffer>;
template <Uint64Buffer uint_buffer>
using net_enums = std::unordered_map<messages_name, msg_enums<uint_buffer>>;

// structure contains all the messages with a signal associated
template <FloatBuffer double_buffer>
using msg_signals = std::unordered_map<field_name, double_buffer>;
template <FloatBuffer double_buffer>
using net_signals = std::unordered_map<messages_name, msg_signals<double_buffer>>;

// structure contains all the messages with a string associated
template <StringBuffer string_buffer>
using msg_strings = std::unordered_map<field_name, string_buffer>;
template <StringBuffer string_buffer>
using net_strings = std::unordered_map<messages_name, msg_strings<string_buffer>>;

template <PushBackable T>
void pushImpl(T &circOrVec, typename T::value_type &val) {
  circOrVec.push_back(val);
}

template <Pushable T>
void pushImpl(T &circOrVec, typename T::value_type &val) {
  circOrVec.push(val);
}

#endif  // CANLIB_PROTO_INTERFACE_TYPES

#endif  // GPSLIB_COMMON_TYPES_H
