// Self-contained MD5 (RFC 1321) used for protocol digests. Internal header.
#ifndef HANPPIE_MD5_HPP
#define HANPPIE_MD5_HPP

#include <cstddef>
#include <cstdint>

#include "hanppie/protocol.hpp"

namespace hanppie {

Bytes md5(const std::uint8_t* data, std::size_t size);

}  // namespace hanppie

#endif  // HANPPIE_MD5_HPP
