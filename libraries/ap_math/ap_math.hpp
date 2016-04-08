#ifndef QUAN_SUBHUB_AP_MATH_HPP_INCLUDED
#define QUAN_SUBHUB_AP_MATH_HPP_INCLUDED

#include <cstdint>

#define DEG_TO_RAD_DOUBLE 0.0174532925199432954743716805978692718781530857086181640625  // equals to (M_PI / 180.0)
#define DEG_TO_RAD 0.017453292519943295769236907684886f

#define RAD_TO_DEG_DOUBLE 57.29577951308232286464772187173366546630859375
#define RAD_TO_DEG 57.295779513082320876798154814105f

// radians -> degrees
inline float degrees(float rad) {
	return rad * RAD_TO_DEG;
}
static inline float radians(float deg) {
	return deg * DEG_TO_RAD;
}
#define ToDeg(x) degrees(x)
#define ToRad(x) radians(x)	// *pi/180

int32_t wrap_360_cd(int32_t angle_cd);

uint16_t crc16_ccitt(const uint8_t *buf, uint32_t len, uint16_t crc);

float safe_sqrt(float v);

#endif // QUAN_SUBHUB_AP_MATH_HPP_INCLUDED
