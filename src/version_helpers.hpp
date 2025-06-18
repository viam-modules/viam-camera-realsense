#pragma once
#ifndef VERSION_HELPERS_HPP
#define VERSION_HELPERS_HPP
#include <cstdint>
#include <iosfwd>
#include <iostream>
#include <string>

struct tm;

namespace viam {
namespace realsense {

struct from {
    std::ostringstream _ss;

    from() = default;

    template <class T>
    explicit from(const T &val) {
        _ss << val;
    }

    // Specialty conversion: like std::to_string; fixed, high-precision (6)
    // Trims ending 0s and reverts to non-fixed notation if '0.' is the result...
    explicit from(double val, int precision = 6);

    template <class T>
    from &operator<<(const T &val) {
        _ss << val;
        return *this;
    }

    std::string str() const { return _ss.str(); }
    operator std::string() const { return _ss.str(); }

    // Returns an empty string if 'time' is null or the format requires too many
    // characters. See strftime for format specifiers.
    static std::string datetime(tm const *time, char const *format = "%Y-%m-%d-%H_%M_%S");
    static std::string datetime(char const *format = "%Y-%m-%d-%H_%M_%S");
};

inline std::ostream &operator<<(std::ostream &os, from const &f) {
    // TODO c++20: use .rdbuf()->view()
    return os << f.str();
}

// Software versions have four numeric components making up a "version string":
//     "MAJOR.MINOR.PATCH[.BUILD]"
// Note that the BUILD is optional...
//
struct version {
    typedef uint16_t sub_type;
    typedef uint64_t number_type;

    number_type number;

    constexpr version() : number(0) {}

    explicit version(number_type n) : number(n) {}

    version(sub_type major, sub_type minor, sub_type patch, sub_type build);

    explicit version(const char *);
    explicit version(const std::string &str) : version(str.c_str()) {}

    bool is_valid() const { return (number != 0); }
    operator bool() const { return is_valid(); }

    void clear() { number = 0; }

    sub_type get_major() const { return sub_type(number >> (8 * 6)); }
    sub_type get_minor() const { return sub_type(number >> (8 * 4)); }
    sub_type get_patch() const { return sub_type(number >> (8 * 2)); }
    sub_type get_build() const { return sub_type(number); }

    version without_build() const { return version(number & ~0xFFFFULL); }

    bool operator<=(version const &other) const { return number <= other.number; }
    bool operator==(version const &other) const { return number == other.number; }
    bool operator>(version const &other) const { return number > other.number; }
    bool operator!=(version const &other) const { return number != other.number; }
    bool operator>=(version const &other) const { return number >= other.number; }
    bool operator<(version const &other) const { return number < other.number; }
    bool is_between(version const &from, version const &until) const {
        return (from <= *this) && (*this <= until);
    }

    std::string to_string() const;
    operator std::string() const { return to_string(); }
};

std::ostream &operator<<(std::ostream &, version const &);

version::version(sub_type m, sub_type n, sub_type p, sub_type b)
    : version((uint64_t(m) << (8 * 6)) + (uint64_t(n) << (8 * 4)) + (uint64_t(p) << (8 * 2)) + b) {
    // Invalidate if any overflow
    if (m != get_major())
        number = 0;
    else if (n != get_minor())
        number = 0;
    else if (p != get_patch())
        number = 0;
    else if (b != get_build())
        number = 0;
}

version::version(char const *base) : version() {
    if (!base) return;
    unsigned major = 0;
    char const *ptr = base;
    while (*ptr != '.') {
        if (*ptr < '0' || *ptr > '9') return;  // If 0, unexpected; otherwise invalid
        major *= 10;
        major += *ptr - '0';
        if (major > 0xFFFF) return;  // Overflow
        ++ptr;
    }
    if (ptr == base) return;  // No major

    unsigned minor = 0;
    base = ++ptr;
    while (*ptr != '.') {
        if (*ptr < '0' || *ptr > '9') return;  // If 0, unexpected; otherwise invalid
        minor *= 10;
        minor += *ptr - '0';
        if (minor > 0xFFFF) return;  // Overflow
        ++ptr;
    }
    if (ptr == base) return;  // No minor

    unsigned patch = 0;
    base = ++ptr;
    while (*ptr != '.') {
        if (!*ptr) break;                      // Acceptable: no build
        if (*ptr < '0' || *ptr > '9') return;  // Invalid
        patch *= 10;
        patch += *ptr - '0';
        if (patch > 0xFFFF) return;  // Overflow
        ++ptr;
    }
    if (ptr == base) return;  // No patch

    unsigned build = 0;
    if (*ptr) {
        base = ++ptr;
        while (*ptr) {
            if (*ptr < '0' || *ptr > '9') return;  // Invalid
            build *= 10;
            build += *ptr - '0';
            if (build > 0xFFFF) return;  // Overflow
            ++ptr;
        }
        if (ptr == base) return;  // No build, but there was a period at the end...!?
    }

    number = version(major, minor, patch, build).number;
}

std::string version::to_string() const { return from() << *this; }

std::ostream &operator<<(std::ostream &os, version const &v) {
    os << v.get_major() << '.' << v.get_minor() << '.' << v.get_patch();
    if (v.get_build()) os << '.' << v.get_build();
    return os;
}

}  // namespace realsense
}  // namespace viam
#endif  // VERSION_HELPERS_HPP
