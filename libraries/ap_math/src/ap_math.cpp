#include <cmath>
#include <ap_math/ap_math.hpp>

int32_t wrap_360_cd(int32_t error)
{
    if (error > 360000 || error < -360000) {
        // for very large numbers use modulus
        error = error % 36000;
    }
    while (error >= 36000) error -= 36000;
    while (error < 0) error += 36000;
    return error;
}

float safe_sqrt(float v)
{

    using std::isnan;

    float ret = sqrtf(v);
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}