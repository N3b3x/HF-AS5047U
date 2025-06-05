#pragma once
#include <cstdint>

// This header provides default configuration values for the AS5047U driver.
// It can be generated from a Kconfig system or edited manually.

namespace AS5047U_CFG {
#ifdef CONFIG_AS5047U_FRAME_32
inline constexpr FrameFormat DEFAULT_FRAME_FORMAT = FrameFormat::SPI_32;
#elif defined(CONFIG_AS5047U_FRAME_24)
inline constexpr FrameFormat DEFAULT_FRAME_FORMAT = FrameFormat::SPI_24;
#else
inline constexpr FrameFormat DEFAULT_FRAME_FORMAT = FrameFormat::SPI_16;
#endif

#ifdef CONFIG_AS5047U_CRC_RETRIES
inline constexpr uint8_t CRC_RETRIES = CONFIG_AS5047U_CRC_RETRIES;
#else
inline constexpr uint8_t CRC_RETRIES = 0;
#endif
} // namespace AS5047U_CFG
