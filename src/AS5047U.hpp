/**
 * @file AS5047U_driver.hpp
 * @brief Driver for AMS AS5047U Magnetic Rotary Position Sensor (C++21).
 *
 * This driver provides hardware-agnostic access to the AS5047U sensor over SPI, 
 * supporting 16-bit, 24-bit, and 32-bit SPI frame formats. It implements all major 
 * features described in the AS5047U datasheet, including absolute angle readout with/without DAEC, 
 * velocity measurement, AGC and magnetic field diagnostics, ABI/UVW/PWM interface configuration, 
 * error/status flag handling, full OTP programming sequence, dynamic angle error compensation, adaptive filtering, and CRC calculation.
 *
 * The design uses a virtual `spiBus` interface to abstract SPI communication, so it can run on any platform. 
 * It assumes the `spiBus` implementation handles synchronization for thread safety. 
 * This driver is optimized for clarity and extensibility and has no direct hardware dependencies.
 */

#ifndef AS5047U_DRIVER_HPP
#define AS5047U_DRIVER_HPP

#include <cstdint>
#include <array>
#include <algorithm>

/**
 * @brief Abstract SPI bus interface that hardware-specific implementations must provide.
 */
class spiBus {
public:
    virtual ~spiBus() = default;
    /**
     * @brief Perform a full-duplex SPI data transfer.
     * 
     * Sends `length` bytes from `txBuffer` and simultaneously receives `length` bytes into `rxBuffer`. 
     * Implementations should assert the device's chip select for the duration of the transfer.
     * 
     * @param txBuffer Pointer to data to transmit (length bytes). If nullptr, zeros can be sent.
     * @param rxBuffer Pointer to buffer for received data (length bytes). If nullptr, received data can be ignored.
     * @param length Number of bytes to transfer.
     * @return true if transfer succeeds, false on bus error.
     */
    virtual bool transfer(const uint8_t *txBuffer, uint8_t *rxBuffer, size_t length) = 0;
};

/** 
 * @brief Supported SPI frame formats for AS5047U communication.
 */
enum class FrameFormat {
    SPI_16, /**< 16-bit frames (no CRC, high-throughput mode) */
    SPI_24, /**< 24-bit frames (includes 8-bit CRC for reliability) */
    SPI_32  /**< 32-bit frames (includes 8-bit CRC and 8-bit pad for daisy-chain) */
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
     * @param frameFormat SPI frame format to use (16-bit, 24-bit, or 32-bit). Default is 16-bit frames.
     */
    AS5047U(spiBus &bus, FrameFormat frameFormat = FrameFormat::SPI_16);

    ~AS5047U() = default;

    /** @brief Set the SPI frame format (16, 24, or 32-bit). */
    void setFrameFormat(FrameFormat format);

    /** @brief Read the 14-bit absolute angle with dynamic compensation (DAEC active). */
    uint16_t getAngle();

    /** @brief Read the 14-bit absolute angle without DAEC (raw angle). */
    uint16_t getRawAngle();

    /** @brief Read the current rotational velocity (signed 16-bit). */
    int16_t getVelocity();

    /** @brief Read the current Automatic Gain Control (AGC) value (0-255). */
    uint8_t getAGC();

    /** @brief Read the magnetic field magnitude (14-bit value). */
    uint16_t getMagnitude();

    /**
     * @brief Read and clear error flags.
     * @return 16-bit error flag register (ERRFL). All flags clear after read.
     */
    uint16_t getErrorFlags();

    /**
     * @brief Set a new zero reference position (soft offset).
     * @param angleOffset 14-bit angle value that should be treated as 0°.
     */
    void setZeroPosition(uint16_t angleOffset);

    /** @brief Get the currently configured soft zero position offset (14-bit). */
    uint16_t getZeroPosition() const;

    /**
     * @brief Define the rotation direction for increasing angle.
     * @param clockwise If true, clockwise rotation yields increasing angle (DIR=0). If false, invert direction (DIR=1).
     */
    void setDirection(bool clockwise);

    /**
     * @brief Set the ABI (incremental encoder) resolution.
     * @param resolution_bits Resolution in bits (10 to 14 bits).
     */
    void setABIResolution(uint8_t resolution_bits);

    /**
     * @brief Set the number of pole pairs for UVW commutation outputs.
     * @param polePairs Number of pole pairs (1-7).
     */
    void setUVWPolePairs(uint8_t polePairs);

    /**
     * @brief Set the index pulse width for ABI output.
     * @param pulseLengthLSB Index pulse length in LSB periods (3 or 1).
     */
    void setIndexPulseLength(uint8_t pulseLengthLSB);

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
    void setDynamicAngleCompensation(bool enabled);

    /** @brief Enable/disable the adaptive filter (Dynamic Filter System). */
    void setAdaptiveFilter(bool enabled);

    /** @brief Set adaptive filter parameters (K_min and K_max, 3-bit each). */
    void setFilterParameters(uint8_t k_min, uint8_t k_max);

    /** @brief Set noise performance mode for 3.3V operation at high temperature. */
    void setNoiseModeHigh(bool highNoise);

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
     * @warning OTP can be programmed only once. Ensure proper supply voltage (3.3-3.5V for 3V mode, ~5V for 5V mode) and desired configuration before use.
     */
    bool programOTP();

private:
    spiBus &spi;             ///< SPI bus interface (hardware abstraction).
    FrameFormat frameFormat; ///< Current SPI frame format mode.

    // Register address constants
    static constexpr uint16_t REG_NOP       = 0x0000;
    static constexpr uint16_t REG_ERRFL     = 0x0001;
    static constexpr uint16_t REG_PROG      = 0x0003;
    static constexpr uint16_t REG_DIA       = 0x3FF5;
    static constexpr uint16_t REG_AGC       = 0x3FF9;
    static constexpr uint16_t REG_SIN       = 0x3FFA;
    static constexpr uint16_t REG_COS       = 0x3FFB;
    static constexpr uint16_t REG_VEL       = 0x3FFC;
    static constexpr uint16_t REG_MAG       = 0x3FFD;
    static constexpr uint16_t REG_ANGLEUNC  = 0x3FFE;
    static constexpr uint16_t REG_ANGLECOM  = 0x3FFF;
    static constexpr uint16_t REG_ZPOSM     = 0x0016;
    static constexpr uint16_t REG_ZPOSL     = 0x0017;
    static constexpr uint16_t REG_SETTINGS1 = 0x0018;
    static constexpr uint16_t REG_SETTINGS2 = 0x0019;
    static constexpr uint16_t REG_SETTINGS3 = 0x001A;
    static constexpr uint16_t REG_DISABLE   = 0x0015;
    static constexpr uint16_t REG_ECC       = 0x001B;
    static constexpr uint16_t REG_ECCCHK    = 0x00D1;

    // Bit masks and positions for specific register fields (for reference)
    static constexpr uint16_t ERRFL_CRCERR     = 1 << 6;
    static constexpr uint16_t PROG_BIT_PROGEN  = 1 << 0;
    static constexpr uint16_t PROG_BIT_PROGOTP = 1 << 1;
    static constexpr uint16_t PROG_BIT_PROGVER = 1 << 2;
    static constexpr uint16_t PROG_BIT_OTPREF  = 1 << 3;

    // Internal helper methods:
    uint16_t readRegister(uint16_t address);
    void writeRegister(uint16_t address, uint16_t value);
    uint8_t computeCRC8(uint16_t data16) const;
};


#endif // AS5047U_DRIVER_HPP
