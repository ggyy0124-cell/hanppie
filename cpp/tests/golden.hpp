// Golden vectors generated from the Python reference implementation.
#ifndef HANPPIE_TEST_GOLDEN_HPP
#define HANPPIE_TEST_GOLDEN_HPP

#include <cstdint>

namespace golden {

struct GoldenEntry {
    const char* kind;
    int sender, receiver, attr, cmdset, cmdid;
    const char* payload;
    const char* flags;
};
struct ChassisVector {
    double x, y, z;
    const char* hex;
};
struct GimbalVector {
    double pitch, yaw;
    const char* hex;
};
struct LedVector {
    std::uint32_t component;
    int red, green, blue;
    const char* effect;
    const char* hex;
};

#include "golden_vectors.inc"

}  // namespace golden

#define GOLDEN_COUNT(array) (sizeof(array) / sizeof((array)[0]))

#endif  // HANPPIE_TEST_GOLDEN_HPP
