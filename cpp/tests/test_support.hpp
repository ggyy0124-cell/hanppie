// Tiny dependency-free test harness for the Hanppie C++ SDK.
#ifndef HANPPIE_TEST_SUPPORT_HPP
#define HANPPIE_TEST_SUPPORT_HPP

#include <chrono>
#include <cstdio>
#include <exception>
#include <sstream>
#include <string>
#include <vector>

#include "hanppie/protocol.hpp"

namespace test {

struct Case {
    const char* name;
    void (*run)();
};

inline std::vector<Case>& registry() {
    static std::vector<Case> cases;
    return cases;
}

struct Registrar {
    Registrar(const char* name, void (*run)()) {
        Case item = {name, run};
        registry().push_back(item);
    }
};

struct Failure {
    std::string message;
};

inline void fail(const char* file, int line, const std::string& message) {
    std::ostringstream out;
    out << file << ":" << line << ": " << message;
    Failure failure = {out.str()};
    throw failure;
}

template <typename A, typename B>
void checkEqual(const A& actual, const B& expected, const char* expression, const char* file,
                int line) {
    if (!(actual == expected)) {
        std::ostringstream out;
        out << "CHECK_EQ(" << expression << ") actual=" << actual << " expected=" << expected;
        fail(file, line, out.str());
    }
}

inline void checkEqual(std::uint8_t actual, int expected, const char* expression,
                       const char* file, int line) {
    checkEqual(static_cast<int>(actual), expected, expression, file, line);
}

inline int runAll() {
    int failed = 0;
    for (std::size_t i = 0; i < registry().size(); ++i) {
        const Case& item = registry()[i];
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        try {
            item.run();
            long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - start)
                               .count();
            std::printf("[ OK ] %s (%lld ms)\n", item.name, ms);
        } catch (const Failure& failure) {
            ++failed;
            std::printf("[FAIL] %s\n       %s\n", item.name, failure.message.c_str());
        } catch (const std::exception& error) {
            ++failed;
            std::printf("[FAIL] %s\n       unexpected exception: %s\n", item.name, error.what());
        }
    }
    std::printf("%u tests, %d failed\n", static_cast<unsigned>(registry().size()), failed);
    return failed == 0 ? 0 : 1;
}

// Apply the robot broadcast XOR stream to a decoded 24-byte broadcast.
inline hanppie::Bytes encodeBroadcast(const hanppie::Bytes& decoded) {
    hanppie::Bytes encoded;
    std::uint8_t key = 7;
    for (std::size_t i = 0; i < decoded.size(); ++i) {
        encoded.push_back(static_cast<std::uint8_t>(decoded[i] ^ key));
        key = static_cast<std::uint8_t>(((key + 7) ^ 178) & 0xFF);
    }
    return encoded;
}

}  // namespace test

#define TEST(name)                                                  \
    static void test_##name();                                      \
    static ::test::Registrar registrar_##name(#name, &test_##name); \
    static void test_##name()

#define CHECK(condition) \
    do {                 \
        if (!(condition)) ::test::fail(__FILE__, __LINE__, "CHECK(" #condition ")"); \
    } while (0)

#define CHECK_EQ(actual, expected) \
    ::test::checkEqual((actual), (expected), #actual ", " #expected, __FILE__, __LINE__)

#define CHECK_THROWS(expression, type)                                                    \
    do {                                                                                  \
        bool thrown = false;                                                              \
        try {                                                                             \
            (void)(expression);                                                           \
        } catch (const type&) {                                                           \
            thrown = true;                                                                \
        }                                                                                 \
        if (!thrown) ::test::fail(__FILE__, __LINE__, "expected " #type " from " #expression); \
    } while (0)

#endif  // HANPPIE_TEST_SUPPORT_HPP
