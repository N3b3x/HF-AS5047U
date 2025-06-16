/**
 * @file AS5047U.hpp
 * @brief Driver for AMS AS5047U Magnetic Rotary Position Sensor (C++21).
 *
 * This driver provides hardware-agnostic access to the AS5047U sensor over SPI,
 * supporting 16-bit, 24-bit, and 32-bit SPI frame formats. It implements all major
 * features described in the AS5047U datasheet, including absolute angle readout with/without DAEC,
 * velocity measurement, AGC and magnetic field diagnostics, ABI/UVW/PWM interface configuration,
 * error/status flag handling, full OTP programming sequence, dynamic angle error compensation,
 * adaptive filtering, and CRC calculation.
 *
 * The design uses a virtual `spiBus` interface to abstract SPI communication, so it can run on any
 * platform. It assumes the `spiBus` implementation handles synchronization for thread safety. This
 * driver is optimized for clarity and extensibility and has no direct hardware dependencies.
 */

#pragma once
#include "AS5047U_REGISTERS.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <bitset>
#include <cmath> // for M_PI and math functions
#include <cstdint>
#include <cstdio> // for printf

// Error flags from ERRFL register
enum class AS5047U_Error : uint16_t {
    None = 0,
    AgcWarning = 1 << 0,     ///< AGC reached minimum (0) or maximum (255) value
    MagHalf = 1 << 1,        ///< Magnetic field is half of regulated value (AGC=255)
    P2ramWarning = 1 << 2,   ///< ECC corrected 1 bit in P2RAM customer area
    P2ramError = 1 << 3,     ///< ECC detected 2+ uncorrectable errors in P2RAM
    FramingError = 1 << 4,   ///< SPI framing error
    CommandError = 1 << 5,   ///< Invalid SPI command received
    CrcError = 1 << 6,       ///< CRC error during SPI communication
    WatchdogError = 1 << 7,  ///< Internal oscillator or watchdog not working correctly
    OffCompError = 1 << 9,   ///< Internal offset compensation not finished
    CordicOverflow = 1 << 10 ///< CORDIC algorithm overflow
};

/**
 * @brief Supported SPI frame formats for AS5047U communication.
 */
enum class FrameFormat : uint8_t {
    SPI_16, /**< 16-bit frames (no CRC, high-throughput mode) */
    SPI_24, /**< 24-bit frames (includes 8-bit CRC for reliability) */
    SPI_32  /**< 32-bit frames (includes 8-bit CRC and 8-bit pad for daisy-chain) */
};

// -----------------------------------------------------------------------------
// Default configuration values
// -----------------------------------------------------------------------------
#include "AS5047U_config.hpp" // defines AS5047U_CFG namespace with defaults

/**
 * @brief AS5047U magnetic rotary sensor driver class.
 *
 * Provides high-level access to sensor features: reading angle (with or without DAEC compensation),
 * reading rotation velocity, retrieving AGC and magnitude diagnostics, configuring outputs (ABI,
 * UVW, PWM), handling error flags, and performing OTP programming for permanent configuration
 * storage.
 */
class AS5047U {
  public:
    /**
     * @brief Abstract SPI bus interface that hardware-specific implementations must provide.
     */
    class spiBus {
      public:
        virtual ~spiBus() = default;

        /**
         * @brief Perform a full-duplex SPI data transfer.
         *
         * Sends `len` bytes from `tx` and simultaneously receives `len` bytes into `rx`.
         * Implementations should assert the device's chip select for the duration of the transfer.
         *
         * @param tx Pointer to data to transmit (len bytes). If nullptr, zeros can be sent.
         * @param rx Pointer to buffer for received data (len bytes). If nullptr, received data can
         * be ignored.
         * @param len Number of bytes to transfer.
         */
        virtual void transfer(const uint8_t *tx, uint8_t *rx, std::size_t len) = 0;
    };

    //------------------------------------------------------------------
    // Constructor and destructor
    //------------------------------------------------------------------

    /**
     * @brief Construct a new AS5047U driver.
     * @param bus Reference to an spiBus implementation for SPI communication.
     * @param format SPI frame format to use (16-bit, 24-bit, or 32-bit). Default is 16-bit frames.
     */
    explicit AS5047U(AS5047U::spiBus &bus,
                     FrameFormat format = AS5047U_CFG::DEFAULT_FRAME_FORMAT) noexcept;

    ~AS5047U() = default;

    /** Compute the CRC8 value used by AS5047U SPI frames. */
    static constexpr uint8_t computeCRC8(uint16_t data16) {
        uint8_t crc = 0xC4;
        for (int i = 0; i < 16; ++i) {
            bool bit = ((data16 >> (15 - i)) & 1) ^ ((crc >> 7) & 1);
            crc = (crc << 1) ^ (bit ? 0x1D : 0x00);
        }
        return crc ^ 0xFF;
    }

    //------------------------------------------------------------------
    // High-level API
    //------------------------------------------------------------------
    /**
     * @brief Set the SPI frame format (16, 24, or 32-bit).
     * @param fmt The desired SPI frame format.
     */
    void setFrameFormat(FrameFormat fmt) noexcept;

    /**
     * @brief Read the 14-bit absolute angle with dynamic compensation (DAEC active).
     *  @param retries Number of retries on CRC/framing error (default 0 = no retry).
     *  @return The current angle in LSB (0-16383).
     */
    [[nodiscard]] uint16_t getAngle(uint8_t retries = AS5047U_CFG::CRC_RETRIES) const;

    /**
     * @brief Read the 14-bit absolute angle without dynamic compensation (raw angle).
     *  @param retries Number of retries on CRC/framing error (default 0 = no retry).
     *  @return The current raw angle in LSB (0-16383).
     */
    [[nodiscard]] uint16_t getRawAngle(uint8_t retries = AS5047U_CFG::CRC_RETRIES) const;

    /**
     * @brief Read the current rotational velocity (signed 14-bit).
     *  @param retries Number of retries on CRC/framing error (default 0 = no retry).
     *  @return The current velocity in LSB (signed 14-bit).
     */
    [[nodiscard]] int16_t getVelocity(uint8_t retries = AS5047U_CFG::CRC_RETRIES) const;

    /** @brief Helper constants and methods for velocity unit conversions. */
    struct Velocity {
        static constexpr double DEG_PER_LSB = 24.141;
        static constexpr double RAD_PER_LSB = DEG_PER_LSB * M_PI / 180.0;
        static constexpr double RPM_PER_LSB = DEG_PER_LSB * (60.0 / 360.0);
    };

    /** @brief Get rotational velocity in degrees per second.
     *  @param retries Number of retries on CRC/framing error (default 0 = no retry).
     */
    [[nodiscard]] double getVelocityDegPerSec(uint8_t retries = AS5047U_CFG::CRC_RETRIES) const;
    /** @brief Get rotational velocity in radians per second.
     *  @param retries Number of retries on CRC/framing error (default 0 = no retry).
     */
    [[nodiscard]] double getVelocityRadPerSec(uint8_t retries = AS5047U_CFG::CRC_RETRIES) const;
    /** @brief Get rotational velocity in revolutions per minute.
     *  @param retries Number of retries on CRC/framing error (default 0 = no retry).
     */
    [[nodiscard]] double getVelocityRPM(uint8_t retries = AS5047U_CFG::CRC_RETRIES) const;

    /**
     * @brief Read the current Automatic Gain Control (AGC) value (0-255).
     * @param retries Number of retries on CRC/framing error (default 0 = no retry).
     * @return The current AGC value.
     */
    [[nodiscard]] uint8_t getAGC(uint8_t retries = AS5047U_CFG::CRC_RETRIES) const;

    /**
     * @brief Read the current magnetic field magnitude (14-bit value).
     * @param retries Number of retries on CRC/framing error (default 0 = no retry).
     * @return The current magnetic field magnitude in LSB (0-16383).
     */
    [[nodiscard]] uint16_t getMagnitude(uint8_t retries = AS5047U_CFG::CRC_RETRIES) const;

    /**
     * @brief Read and clear error flags.
     * @param retries Number of retries on CRC/framing error (default 0 = no retry).
     * @return 16-bit error flag register (ERRFL). All flags clear after read.
     */
    [[nodiscard]] uint16_t getErrorFlags(uint8_t retries = AS5047U_CFG::CRC_RETRIES) const;

    /**
     * @brief Dump formatted status and diagnostics using printf
     */
    void dumpStatus() const;

    /**
     * @brief Get the currently configured soft zero position offset (14-bit).
     * @param retries Number of retries on CRC/framing error (default 0 = no retry).
     * @return The current zero position in LSB (0-16383).
     */
    [[nodiscard]] uint16_t getZeroPosition(uint8_t retries = AS5047U_CFG::CRC_RETRIES) const;

    /**
     * @brief Set a new zero reference position (soft offset).
     * @param angleLSB 14-bit angle value that should be treated as 0°.
     */
    bool setZeroPosition(uint16_t angleLSB, uint8_t retries = AS5047U_CFG::CRC_RETRIES);

    /**
     * @brief Define the rotation direction for increasing angle.
     * @param clockwise If true, clockwise rotation yields increasing angle (DIR=0).
     *                  If false, invert direction (DIR=1).
     * @param retries Number of retries on CRC/framing error (default 0 = no retry).
     * @return true if register write succeeded.
     */
    bool setDirection(bool clockwise, uint8_t retries = AS5047U_CFG::CRC_RETRIES);

    /**
     * @brief Set the ABI (incremental encoder) resolution.
     * @param resolutionBits Resolution in bits (10 to 14 bits).
     */
    bool setABIResolution(uint8_t resolutionBits, uint8_t retries = AS5047U_CFG::CRC_RETRIES);

    /**
     * @brief Set the number of pole pairs for UVW commutation outputs.
     * @param pairs Number of pole pairs (1-7).
     */
    bool setUVWPolePairs(uint8_t pairs, uint8_t retries = AS5047U_CFG::CRC_RETRIES);

    /**
     * @brief Set the index pulse width for ABI output.
     * @param lsbLen Index pulse length in LSB periods (3 or 1).
     */
    bool setIndexPulseLength(uint8_t lsbLen, uint8_t retries = AS5047U_CFG::CRC_RETRIES);

    /**
     * @brief Configure interface outputs (ABI, UVW) and PWM output.
     *
     * Enables/disables ABI and UVW outputs and configures PWM on the available pin:
     * - If `abi` is true and `uvw` false, ABI outputs are enabled and PWM (if enabled) is on W pin.
     * - If `uvw` is true and `abi` false, UVW outputs are enabled and PWM (if enabled) is on I pin.
     * - If both `abi` and `uvw` are true, both interfaces are active (PWM not available in this
     * mode).
     * - If both are false, all interfaces are disabled (PWM can still be enabled on W by default).
     *
     * @param abi Enable ABI (A, B, I) outputs.
     * @param uvw Enable UVW commutation outputs.
     * @param pwm Enable PWM output on the appropriate pin (W if UVW disabled, I if ABI disabled).
     */
    bool configureInterface(bool abi, bool uvw, bool pwm,
                            uint8_t retries = AS5047U_CFG::CRC_RETRIES);

    /** @brief Enable/disable Dynamic Angle Error Compensation (DAEC). */
    bool setDynamicAngleCompensation(bool enable, uint8_t retries = AS5047U_CFG::CRC_RETRIES);

    /** @brief Enable/disable the adaptive filter (Dynamic Filter System). */
    bool setAdaptiveFilter(bool enable, uint8_t retries = AS5047U_CFG::CRC_RETRIES);

    /** @brief Set adaptive filter parameters (K_min and K_max, 3-bit each). */
    bool setFilterParameters(uint8_t k_min, uint8_t k_max,
                             uint8_t retries = AS5047U_CFG::CRC_RETRIES);

    /** @brief Set temperature mode for 150°C operation (NOISESET bit).
     *
     * When enabled, the sensor supports full 150°C range (NOISESET=1),
     * but with increased noise. When disabled, lower noise but max 125°C.
     *
     * @param enable True for 150°C mode (NOISESET=1), false for low-noise (NOISESET=0).
     */
    bool set150CTemperatureMode(bool enable, uint8_t retries = AS5047U_CFG::CRC_RETRIES);

    /**
     * @brief Permanently program current settings into OTP memory.
     *
     * Performs the full OTP programming sequence:
     * - Writes current configuration (zero position, settings registers) to volatile registers.
     * - Reads current angle and sets it as zero reference.
     * - Enables ECC, reads and writes ECC checksum.
     * - Triggers OTP burn and waits for completion.
     * - Performs guard-band verification (clears registers, refreshes from OTP, verifies content).
     *
     * @return True if programming and verification succeeded, false otherwise.
     * @warning OTP can be programmed only once. Ensure proper supply voltage (3.3-3.5V for 3V mode,
     *          ~5V for 5V mode) and desired configuration before use.
     */
    bool programOTP();

    /**
     * @brief Set the daisy-chain pad byte for 32-bit SPI frames.
     *
     * In 32-bit frame mode, the first byte is used as a pad to allow multiple devices on the same
     * bus.
     * @param pad Pad byte value (0x00–0xFF) to send as the MSB of each 32-bit transfer.
     */
    void setPad(uint8_t pad) noexcept;

    /**
     * @brief Set incremental output hysteresis level.
     *
     * Controls the HYS field in SETTINGS3 to add output deadband and reduce chatter.
     * @param hys Hysteresis level:
     *   - SETTINGS3::Hysteresis::LSB_1 (0b00): 1 LSB deadband (~0.17°, default)
     *   - SETTINGS3::Hysteresis::LSB_2 (0b01): 2 LSB deadband (~0.35°)
     *   - SETTINGS3::Hysteresis::LSB_3 (0b10): 3 LSB deadband (~0.52°)
     *   - SETTINGS3::Hysteresis::NONE  (0b11): No hysteresis
     */
    bool setHysteresis(AS5047U_REG::SETTINGS3::Hysteresis hys,
                       uint8_t retries = AS5047U_CFG::CRC_RETRIES);

    /**
     * @brief Get current incremental output hysteresis setting.
     * @return Current Hysteresis enum value (0b00–0b11).
     */
    AS5047U_REG::SETTINGS3::Hysteresis getHysteresis() const;

    /**
     * @brief Select which angle register (0x3FFF) is returned on reads.
     *
     * Controls the Data_select bit in SETTINGS2.
     * @param src Angle output source:
     *   - SETTINGS2::AngleOutputSource::UseANGLECOM: read compensated angle (ANGLECOM)
     *   - SETTINGS2::AngleOutputSource::UseANGLEUNC: read raw angle (ANGLEUNC)
     */
    bool setAngleOutputSource(AS5047U_REG::SETTINGS2::AngleOutputSource src,
                              uint8_t retries = AS5047U_CFG::CRC_RETRIES);

    /**
     * @brief Get currently selected angle output source for 0x3FFF reads.
     * @return AngleOutputSource enum indicating ANGLECOM or ANGLEUNC.
     */
    AS5047U_REG::SETTINGS2::AngleOutputSource getAngleOutputSource() const;

    /**
     * @brief Read the full diagnostic register (DIA).
     * @return DIA register struct containing per-bit calibration and status flags.
     */
    AS5047U_REG::DIA getDiagnostics() const;

    /**
     * @brief Reads data from a specified register in the AS5047U sensor
     *
     * @tparam RegT The register type which must have an ADDRESS static member and be decodable
     * @return RegT The register object populated with the data read from the sensor
     *
     * This method reads the raw value from the specified register address and decodes
     * it into a strongly-typed register object.
     */
    template <typename RegT> RegT readReg() const {
        return decode<RegT>(readRegister(RegT::ADDRESS));
    }

    /**
     * @brief Writes data to a specified register in the AS5047U sensor
     *
     * @tparam RegT The register type which must have an ADDRESS static member and be encodable
     * @param reg The register object containing the data to be written
     * @param retries Number of retries on CRC/framing error (default 0 = no retry)
     * @return true if write succeeded, false on CRC/framing error
     */
    template <typename RegT>
    bool writeReg(const RegT &reg, uint8_t retries = AS5047U_CFG::CRC_RETRIES) {
        return writeRegister(RegT::ADDRESS, encode(reg), retries);
    }

    /**
     * @brief Retrieve and clear the accumulated sticky error flags.
     * @return Bitwise OR of AS5047U_Error enum flags since last call.
     */
    AS5047U_Error getStickyErrorFlags() const;

  private:
    //------------------------------------------------------------------
    // Low-level helpers
    //------------------------------------------------------------------
    // Low level register access helpers
    uint16_t rawReadRegister(uint16_t addr) const; ///< read register without updating sticky errors
    uint16_t readRegister(uint16_t addr) const;    ///< read register and refresh sticky errors
    bool writeRegister(uint16_t addr, uint16_t val, uint8_t retries) const;

    spiBus &spi;             ///< reference to user-supplied SPI driver
    FrameFormat frameFormat; ///< current SPI frame format
    uint8_t padByte{0};      ///< pad byte for SPI_32 daisy-chain indexing

    mutable std::atomic<uint16_t> stickyErrors{0}; ///< sticky error bits since last clear
    void updateStickyErrors(uint16_t errfl) const;

    // Helper functions implemented inline for templates
    template <typename RegT> static constexpr uint16_t encode(const RegT &r) {
        return r.value;
    }

    template <typename RegT> static constexpr RegT decode(uint16_t raw) {
        RegT r{};
        r.value = raw;
        return r;
    }
};

inline bool AS5047U::setDirection(bool clockwise, uint8_t retries) {
    auto s2 = readReg<AS5047U_REG::SETTINGS2>();
    s2.bits.DIR = clockwise ? 0 : 1;
    return writeReg(s2, retries);
}
