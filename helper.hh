#ifndef __HELPERFUNCTS__
#define __HELPERFUNCTS__
#include <string>
#include <sstream>
#include <vector>
#include <cstdint>
#include <iostream>

static const char KNRM[] = "\x1B[0m";
static const char KRED[] = "\x1B[31m";
static const char KGRN[] = "\x1B[32m";
static const char KYEL[] = "\x1B[33m";
static const char KBLU[] = "\x1B[34m";
static const char KMAG[] = "\x1B[35m";
static const char KCYN[] = "\x1B[36m";
static const char KWHT[] = "\x1B[37m";


template <typename T, unsigned B>
inline T signextend(const T x) {
  struct {T x:B;} s;
  return s.x = x;
}

template <typename T, typename std::enable_if<std::is_integral<T>::value,T>::type* = nullptr>
T roundToPgSz(T x, T pgsz) {
  return ((x + pgsz -1) / pgsz) * pgsz;
}

template <typename T, typename std::enable_if<std::is_integral<T>::value,T>::type* = nullptr>
bool isPow2(T x) {
  return (((x-1)&x) == 0);
}

template <typename T, typename std::enable_if<std::is_integral<T>::value,T>::type* = nullptr>
T nextPow2(T x) {
  T y = 1;
  while(y < x) {
    y *= 2;
  }
  return y;
}

template <typename T, typename std::enable_if<std::is_integral<T>::value,T>::type* = nullptr>
T ln2(T x) {
  T y = 1, l =0;
  while(y < x) {
    l++;
    y *= 2;
  }
  return l;
}

template <typename T, typename std::enable_if<std::is_integral<T>::value,T>::type* = nullptr>
T mod(T x, T y) {
  if(isPow2(y)) {
    return x & (y-1);
  }
  return x % y;
}


void dbt_backtrace();

#ifndef UNREACHABLE
#define UNREACHABLE() {				\
    __builtin_unreachable();			\
  }
#endif

#define die() {								\
    std::cerr << __PRETTY_FUNCTION__ << " @ " << __FILE__ << ":"	\
	      << __LINE__ << " called die\n";				\
    abort();								\
  }


#ifndef print_var
#define print_var(x) std::cout <<  #x << " = " << x << "\n";
#endif

double timestamp();

uint32_t update_crc(uint32_t crc, uint8_t *buf, size_t len);
uint32_t crc32(uint8_t *buf, size_t len);

int32_t remapIOFlags(int32_t flags);

template <class T>
std::string toString(T x) {
  return std::to_string(x);
}

template <class T>
std::string toStringHex(T x) {
  std::stringstream ss;
  ss << std::hex << x;
  return ss.str();
}

template <typename T, typename std::enable_if<std::is_integral<T>::value, T>::type* = nullptr>
bool extractBit(T x, uint32_t b) {
  return (x >> b) & 0x1;
}

template <typename T, typename std::enable_if<std::is_integral<T>::value, T>::type* = nullptr>
T setBit(T x, T v, uint32_t b) {
  T t = static_cast<T>(1) << b;
  T tt = (~t) & x;
  t  &= ~(v-1);
  return (tt | t);
}

#define INTEGRAL_ENABLE_IF(SZ,T) typename std::enable_if<std::is_integral<T>::value and (sizeof(T)==SZ),T>::type* = nullptr

template <typename T, INTEGRAL_ENABLE_IF(1,T)>
T bswap(T x) {
  return x;
}

template <bool EL, typename T, INTEGRAL_ENABLE_IF(2,T)> 
T bswap(T x) {
  static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "must be little endian machine");
  if(EL) 
    return x;
  else
  return  __builtin_bswap16(x);
}

template <bool EL, typename T, INTEGRAL_ENABLE_IF(4,T)>
T bswap(T x) {
  static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "must be little endian machine");
  if(EL)
    return x;
  else 
    return  __builtin_bswap32(x);
}

template <bool EL, typename T, INTEGRAL_ENABLE_IF(8,T)> 
T bswap(T x) {
  static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "must be little endian machine");
  if(EL)
    return x;
  else 
    return  __builtin_bswap64(x);
}

#undef INTEGRAL_ENABLE_IF




#endif
