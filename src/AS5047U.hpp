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
 * The design uses a virtual `spiBus` interface to abstract SPI communication, so it can run on any platform. 
 * It assumes the `spiBus` implementation handles synchronization for thread safety. 
 * This driver is optimized for clarity and extensibility and has no direct hardware dependencies.
 */

#pragma once
#include <cstdint>
#include <array>
#include <algorithm>
#include <atomic>
#include <bitset>
#include "as5047u_registers_2.hpp"

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
     * @param rx Pointer to buffer for received data (len bytes). If nullptr, received data can be ignored.
     * @param len Number of bytes to transfer.
     */
    virtual void transfer(const uint8_t *tx, uint8_t *rx, std::size_t len) = 0;
};

/**
 * @brief AS5047U magnetic rotary sensor driver class.
 *
 * Provides high-level access to sensor features: reading angle (with or without DAEC compensation),
 * reading rotation velocity, retrieving AGC and magnitude diagnostics, configuring outputs (ABI, UVW, PWM),
 * handling error flags, and performing OTP programming for permanent configuration storage.
 */
class AS5047U {
public:
    /**
     * @brief Construct a new AS5047U driver.
     * @param bus Reference to an spiBus implementation for SPI communication.
     * @param format SPI frame format to use (16-bit, 24-bit, or 32-bit). Default is 16-bit frames.
     */
    explicit AS5047U(spiBus &bus, FrameFormat format = FrameFormat::SPI_16) noexcept;

    ~AS5047U() = default;

    //------------------------------------------------------------------
    // High-level API
    //------------------------------------------------------------------

    /**
     * @brief Supported SPI frame formats for AS5047U communication.
     */
    enum class FrameFormat : uint8_t {
        SPI_16, /**< 16-bit frames (no CRC, high-throughput mode) */
        SPI_24, /**< 24-bit frames (includes 8-bit CRC for reliability) */
        SPI_32  /**< 32-bit frames (includes 8-bit CRC and 8-bit pad for daisy-chain) */
    };
    
    /** @brief Set the SPI frame format (16, 24, or 32-bit). */
    void setFrameFormat(FrameFormat fmt) noexcept;

    /** @brief Read the 14-bit absolute angle with dynamic compensation (DAEC active). */
    [[nodiscard]] uint16_t getAngle();
    
    /** @brief Read the 14-bit absolute angle without DAEC (raw angle). */
    [[nodiscard]] uint16_t getRawAngle();
    
    /** @brief Read the current rotational velocity (signed 14-bit). */
    [[nodiscard]] int16_t getVelocity();
    
    /** @brief Read the current Automatic Gain Control (AGC) value (0-255). */
    [[nodiscard]] uint8_t getAGC();
    
    /** @brief Read the magnetic field magnitude (14-bit value). */
    [[nodiscard]] uint16_t getMagnitude();
    
    /**
     * @brief Read and clear error flags.
     * @return 16-bit error flag register (ERRFL). All flags clear after read.
     */
    [[nodiscard]] uint16_t getErrorFlags();
    
    /**
     * @brief Get the currently configured soft zero position offset (14-bit).
     */
    [[nodiscard]] uint16_t getZeroPosition() const;
    
    /**
     * @brief Set a new zero reference position (soft offset).
     * @param angleLSB 14-bit angle value that should be treated as 0°.
     */
    void setZeroPosition(uint16_t angleLSB);
    
    /**
     * @brief Define the rotation direction for increasing angle.
     * @param clockwise If true, clockwise rotation yields increasing angle (DIR=0). 
     *                  If false, invert direction (DIR=1).
     */
    void setDirection(bool clockwise);
    
    /**
     * @brief Set the ABI (incremental encoder) resolution.
     * @param resolutionBits Resolution in bits (10 to 14 bits).
     */
    void setABIResolution(uint8_t resolutionBits);
    
    /**
     * @brief Set the number of pole pairs for UVW commutation outputs.
     * @param pairs Number of pole pairs (1-7).
     */
    void setUVWPolePairs(uint8_t pairs);
    
    /**
     * @brief Set the index pulse width for ABI output.
     * @param lsbLen Index pulse length in LSB periods (3 or 1).
     */
    void setIndexPulseLength(uint8_t lsbLen);
    
    /**
     * @brief Configure interface outputs (ABI, UVW) and PWM output.
     *
     * Enables/disables ABI and UVW outputs and configures PWM on the available pin:
     * - If `abi` is true and `uvw` false, ABI outputs are enabled and PWM (if enabled) is on W pin.
     * - If `uvw` is true and `abi` false, UVW outputs are enabled and PWM (if enabled) is on I pin.
     * - If both `abi` and `uvw` are true, both interfaces are active (PWM not available in this mode).
     * - If both are false, all interfaces are disabled (PWM can still be enabled on W by default).
     *
     * @param abi Enable ABI (A, B, I) outputs.
     * @param uvw Enable UVW commutation outputs.
     * @param pwm Enable PWM output on the appropriate pin (W if UVW disabled, I if ABI disabled).
     */
    void configureInterface(bool abi, bool uvw, bool pwm);
    
    /** @brief Enable/disable Dynamic Angle Error Compensation (DAEC). */
    void setDynamicAngleCompensation(bool enable);
    
    /** @brief Enable/disable the adaptive filter (Dynamic Filter System). */
    void setAdaptiveFilter(bool enable);
    
    /** @brief Set adaptive filter parameters (K_min and K_max, 3-bit each). */
    void setFilterParameters(uint8_t k_min, uint8_t k_max);
    
    /** @brief Set temperature mode for 150°C operation (NOISESET bit).
     *
     * When enabled, the sensor supports full 150°C range (NOISESET=1),
     * but with increased noise. When disabled, lower noise but max 125°C.
     *
     * @param enable True for 150°C mode (NOISESET=1), false for low-noise (NOISESET=0).
     */
    void set150CTemperatureMode(bool enable);
    
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
     * @brief Reads data from a specified register in the AS5047U sensor
     *
     * @tparam RegT The register type which must have an ADDRESS static member and be decodable
     * @return RegT The register object populated with the data read from the sensor
     *
     * This method reads the raw value from the specified register address and decodes
     * it into a strongly-typed register object.
     */
    template <typename RegT>
    RegT readReg() { return decode<RegT>(readRegister(RegT::ADDRESS)); }

    /**
     * @brief Writes data to a specified register in the AS5047U sensor
     *
     * @tparam RegT The register type which must have an ADDRESS static member and be encodable
     * @param reg The register object containing the data to be written
     *
     * This method takes a register object, encodes it using the encode() function,
     * and writes the encoded value to the register's address using the writeRegister() method.
     */
    template <typename RegT>
    void writeReg(const RegT& reg) { writeRegister(RegT::ADDRESS, encode(reg)); }

    /**
     * @brief Retrieve and clear the sticky error bitfield (since last call).
     */
    AS5047U_Error getStickyErrorFlags();

private:

    //------------------------------------------------------------------
    // Low-level helpers
    //------------------------------------------------------------------
    uint16_t readRegister(uint16_t addr);
    void     writeRegister(uint16_t addr, uint16_t val);
    [[nodiscard]] uint8_t computeCRC8(uint16_t data16) const;

    //------------------------------------------------------------------
    // Register instances
    //------------------------------------------------------------------
    spiBus      &spi;         ///< reference to user-supplied SPI driver
    FrameFormat  frameFormat; ///< current SPI frame format
    // (All register instance variables removed; use stack/register struct access only)
    //------------------------------------------------------------------
    // Error flags for CRC and SPI errors
    enum class AS5047U_Error : uint8_t {
        None        = 0,
        CRC         = 1 << 0,
        Framing     = 1 << 1,
        Command     = 1 << 2,
        Watchdog    = 1 << 3,
        AGCWarning  = 1 << 4,
        MagHalf     = 1 << 5,
        P2RAMWarn   = 1 << 6,
        P2RAMError  = 1 << 7,
        // ... add more as needed ...
    };
    inline AS5047U_Error operator|(AS5047U_Error a, AS5047U_Error b) { return static_cast<AS5047U_Error>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b)); }
    inline AS5047U_Error& operator|=(AS5047U_Error& a, AS5047U_Error b) { a = a | b; return a; }
    std::atomic<uint8_t> stickyErrors {0};
    void updateStickyErrors(uint16_t errfl);
    //------------------------------------------------------------------
};