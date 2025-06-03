#include "AS5047U.hpp"
#include <iomanip> // for std::hex and formatting

// Inline function definitions
inline void AS5047U::setFrameFormat(FrameFormat format) noexcept {
    frameFormat = format;
}

// Constructor implementation
AS5047U::AS5047U(AS5047U::spiBus &bus, FrameFormat frameFormat) noexcept
    : spi(bus), frameFormat(frameFormat) {
    // No further initialization (use sensor defaults unless configured).
}

// ══════════════════════════════════════════════════════════════════════════════════════════
//                                PRIVATE HELPERS
// ══════════════════════════════════════════════════════════════════════════════════════════

// Helper: encode/decode register value

// Encoding/decoding helpers are defined inline in the header

// ══════════════════════════════════════════════════════════════════════════════════════════
//                                 PUBLIC HIGH-LEVEL API
// ══════════════════════════════════════════════════════════════════════════════════════════

uint16_t AS5047U::getAngle(uint8_t retries) const {
    uint16_t val = 0;
    constexpr uint16_t retryMask = static_cast<uint16_t>(AS5047U_Error::CrcError) | static_cast<uint16_t>(AS5047U_Error::FramingError);
    for (uint8_t i = 0; i <= retries; ++i) {
        val = readReg<AS5047U_REG::ANGLECOM>().bits.ANGLECOM_value;
        auto err = getStickyErrorFlags();
        if (!(static_cast<uint16_t>(err) & retryMask)) break;
    }
    return val;
}

uint16_t AS5047U::getRawAngle(uint8_t retries) const {
    uint16_t val = 0;
    constexpr uint16_t retryMask = static_cast<uint16_t>(AS5047U_Error::CrcError) | static_cast<uint16_t>(AS5047U_Error::FramingError);
    for (uint8_t i = 0; i <= retries; ++i) {
        val = readReg<AS5047U_REG::ANGLEUNC>().bits.ANGLEUNC_value;
        auto err = getStickyErrorFlags();
        if (!(static_cast<uint16_t>(err) & retryMask)) break;
    }
    return val;
}

int16_t AS5047U::getVelocity(uint8_t retries) const {
    int16_t val = 0;
    constexpr uint16_t retryMask = static_cast<uint16_t>(AS5047U_Error::CrcError) | static_cast<uint16_t>(AS5047U_Error::FramingError);
    for (uint8_t i = 0; i <= retries; ++i) {
        auto v = readReg<AS5047U_REG::VEL>().bits.VEL_value;
        val = static_cast<int16_t>((static_cast<int16_t>(v << 2)) >> 2);
        auto err = getStickyErrorFlags();
        if (!(static_cast<uint16_t>(err) & retryMask)) break;
    }
    return val;
}

double AS5047U::getVelocityDegPerSec(uint8_t retries) const {
    return getVelocity(retries) * Velocity::DEG_PER_LSB;
}

double AS5047U::getVelocityRadPerSec(uint8_t retries) const {
    return getVelocity(retries) * Velocity::RAD_PER_LSB;
}

double AS5047U::getVelocityRPM(uint8_t retries) const {
    return getVelocity(retries) * Velocity::RPM_PER_LSB;
}

uint8_t AS5047U::getAGC(uint8_t retries) const {
    uint8_t val = 0;
    constexpr uint16_t retryMask = static_cast<uint16_t>(AS5047U_Error::CrcError) | static_cast<uint16_t>(AS5047U_Error::FramingError);
    for (uint8_t i = 0; i <= retries; ++i) {
        val = readReg<AS5047U_REG::AGC>().bits.AGC_value;
        auto err = getStickyErrorFlags();
        if (!(static_cast<uint16_t>(err) & retryMask)) break;
    }
    return val;
}

uint16_t AS5047U::getMagnitude(uint8_t retries) const {
    uint16_t val = 0;
    constexpr uint16_t retryMask = static_cast<uint16_t>(AS5047U_Error::CrcError) | static_cast<uint16_t>(AS5047U_Error::FramingError);
    for (uint8_t i = 0; i <= retries; ++i) {
        val = readReg<AS5047U_REG::MAG>().bits.MAG_value;
        auto err = getStickyErrorFlags();
        if (!(static_cast<uint16_t>(err) & retryMask)) break;
    }
    return val;
}

uint16_t AS5047U::getErrorFlags(uint8_t retries) const {
    uint16_t val = 0;
    for (uint8_t i = 0; i <= retries; ++i) {
        val = readReg<AS5047U_REG::ERRFL>().value;
        if (val == 0) break;
    }
    return val;
}

uint16_t AS5047U::getZeroPosition(uint8_t retries) const {
    uint8_t m = 0;
    uint8_t l = 0;
    constexpr uint16_t retryMask = static_cast<uint16_t>(AS5047U_Error::CrcError) | static_cast<uint16_t>(AS5047U_Error::FramingError);
    
    // First read ZPOSM with retries
    for (uint8_t i = 0; i <= retries; ++i) {
        m = readReg<AS5047U_REG::ZPOSM>().bits.ZPOSM_bits;
        auto err = getStickyErrorFlags();
        if (!(static_cast<uint16_t>(err) & retryMask)) break;
    }
    
    // Then read ZPOSL with retries
    for (uint8_t i = 0; i <= retries; ++i) {
        l = readReg<AS5047U_REG::ZPOSL>().bits.ZPOSL_bits;
        auto err = getStickyErrorFlags();
        if (!(static_cast<uint16_t>(err) & retryMask)) break;
    }
    
    return static_cast<uint16_t>((m << 6) | l);
}

bool AS5047U::setZeroPosition(uint16_t angleLSB, uint8_t retries) {
    AS5047U_REG::ZPOSM m{}; m.bits.ZPOSM_bits = (angleLSB >> 6) & 0xFF;
    AS5047U_REG::ZPOSL l{}; l.bits.ZPOSL_bits = angleLSB & 0x3F;
    return writeReg(m, retries) && writeReg(l, retries);
}

bool AS5047U::setABIResolution(uint8_t resolutionBits, uint8_t retries) {
    resolutionBits = std::clamp(resolutionBits, uint8_t(10), uint8_t(14));
    auto s3 = readReg<AS5047U_REG::SETTINGS3>();
    s3.bits.ABIRES = static_cast<uint8_t>(resolutionBits - 10);
    return writeReg(s3, retries);
}

bool AS5047U::setUVWPolePairs(uint8_t pairs, uint8_t retries) {
    pairs = std::clamp(pairs, uint8_t(1), uint8_t(7));
    auto s3 = readReg<AS5047U_REG::SETTINGS3>();
    s3.bits.UVWPP = static_cast<uint8_t>(pairs - 1);
    return writeReg(s3, retries);
}

bool AS5047U::setIndexPulseLength(uint8_t lsbLen, uint8_t retries) {
    auto s2 = readReg<AS5047U_REG::SETTINGS2>();
    s2.bits.IWIDTH = (lsbLen == 1) ? 1 : 0;
    return writeReg(s2, retries);
}

// Truth table for configureInterface():
// | ABI | UVW | PWM | Pin 8 (I) | Pin 14 (W) |
// |-----|-----|-----|-----------|------------|
// |  1  |  0  |  0  |   I       |   A/B      |
// |  1  |  0  |  1  |   I       |   PWM      |
// |  0  |  1  |  0  |   UVW     |   W        |
// |  0  |  1  |  1  |   PWM     |   W        |
// |  1  |  1  |  x  |   I       |   W        |
// |  0  |  0  |  1  |   -       |   PWM      |
// |  0  |  0  |  0  |   -       |   -        |
//
bool AS5047U::configureInterface(bool abi, bool uvw, bool pwm, uint8_t retries) {
    auto dis = readReg<AS5047U_REG::DISABLE>();
    auto s2  = readReg<AS5047U_REG::SETTINGS2>();
    dis.bits.ABI_off = abi ? 0 : 1;
    dis.bits.UVW_off = uvw ? 0 : 1;
    if (abi && !uvw) {
        s2.bits.UVW_ABI = 0; s2.bits.PWMon = pwm;
    } else if (!abi && uvw) {
        s2.bits.UVW_ABI = 1; s2.bits.PWMon = pwm;
    } else {
        s2.bits.UVW_ABI = 0; s2.bits.PWMon = pwm;
    }
    return writeReg(dis, retries) && writeReg(s2, retries);
}

bool AS5047U::setDynamicAngleCompensation(bool enable, uint8_t retries) {
    auto s2 = readReg<AS5047U_REG::SETTINGS2>();
    s2.bits.DAECDIS = enable ? 0 : 1;
    return writeReg(s2, retries);
}

bool AS5047U::setAdaptiveFilter(bool enable, uint8_t retries) {
    auto dis = readReg<AS5047U_REG::DISABLE>();
    dis.bits.FILTER_disable = enable ? 0 : 1;
    return writeReg(dis, retries);
}

bool AS5047U::setFilterParameters(uint8_t k_min, uint8_t k_max, uint8_t retries) {
    k_min = std::min(k_min, uint8_t(7)); k_max = std::min(k_max, uint8_t(7));
    auto s1 = readReg<AS5047U_REG::SETTINGS1>();
    s1.bits.K_min = k_min; s1.bits.K_max = k_max;
    return writeReg(s1, retries);
}

bool AS5047U::set150CTemperatureMode(bool enable, uint8_t retries) {
    auto s2 = readReg<AS5047U_REG::SETTINGS2>();
    s2.bits.NOISESET = enable ? 1 : 0;
    return writeReg(s2, retries);
}

bool AS5047U::programOTP() {
    // Save current frame format and ensure we use CRC for OTP programming
    FrameFormat backup = frameFormat;
    if (frameFormat == FrameFormat::SPI_16) {
        frameFormat = FrameFormat::SPI_24;
    }
    
    // Set current angle as zero position
    setZeroPosition(getAngle());
    
    // Backup the volatile shadow registers that will be committed to OTP
    uint16_t volatileShadow[5];
    for (uint16_t a = 0x0016; a <= 0x001A; ++a) {
        volatileShadow[a-0x0016] = readRegister(a);
    }
    
    // Enable ECC and compute needed checksum
    auto ecc = readReg<AS5047U_REG::ECC>();
    ecc.bits.ECC_en = 1;
    writeReg(ecc);
    auto key = readReg<AS5047U_REG::ECC_Checksum>().bits.ECC_s;
    ecc.bits.ECC_chsum = key;
    writeReg(ecc);
    
    // Verify shadow registers are still correct
    for (uint16_t a = 0x0016; a <= 0x001A; ++a) {
        if (readRegister(a) != volatileShadow[a-0x0016]) { 
            frameFormat = backup; 
            return false; 
        }
    }
    
    // Begin OTP programming sequence
    AS5047U_REG::PROG p{};  
    p.bits.PROGEN = 1;  
    writeReg(p);
    p.bits.PROGOTP = 1;     
    writeReg(p);
    
    // Wait for programming to complete (timeout after ~15000 cycles)
    for (uint16_t i = 0; i < 15000; ++i) {
        if (readRegister(AS5047U_REG::PROG::ADDRESS) == 0x0001) { 
            frameFormat = backup; 
            
            // Guard-band verification: enable PROGVER and refresh OTPREF
            p.bits.PROGVER = 1; 
            writeReg(p);
            
            // Toggle OTPREF to reload from OTP
            p.bits.OTPREF = 1; 
            writeReg(p);
            p.bits.OTPREF = 0; 
            writeReg(p);
            
            // Verify shadow registers match OTP
            for (uint16_t a = 0x0016; a <= 0x001A; ++a) {
                if (readRegister(a) != volatileShadow[a-0x0016]) { 
                    frameFormat = backup; 
                    return false; 
                }
            }
            return true; 
        }
    }
    
    // Restore original frame format and return failure if timeout
    frameFormat = backup;
    return false;
}

void AS5047U::updateStickyErrors(uint16_t errfl) const {
    // Map ERRFL bits (0-10) to sticky error enum
    if (errfl & (1u << 0))  stickyErrors |= static_cast<uint16_t>(AS5047U_Error::AgcWarning);
    if (errfl & (1u << 1))  stickyErrors |= static_cast<uint16_t>(AS5047U_Error::MagHalf);
    if (errfl & (1u << 2))  stickyErrors |= static_cast<uint16_t>(AS5047U_Error::P2ramWarning);
    if (errfl & (1u << 3))  stickyErrors |= static_cast<uint16_t>(AS5047U_Error::P2ramError);
    if (errfl & (1u << 4))  stickyErrors |= static_cast<uint16_t>(AS5047U_Error::FramingError);
    if (errfl & (1u << 5))  stickyErrors |= static_cast<uint16_t>(AS5047U_Error::CommandError);
    if (errfl & (1u << 6))  stickyErrors |= static_cast<uint16_t>(AS5047U_Error::CrcError);
    if (errfl & (1u << 7))  stickyErrors |= static_cast<uint16_t>(AS5047U_Error::WatchdogError);
    if (errfl & (1u << 9))  stickyErrors |= static_cast<uint16_t>(AS5047U_Error::OffCompError);
    if (errfl & (1u << 10)) stickyErrors |= static_cast<uint16_t>(AS5047U_Error::CordicOverflow);
}
 
AS5047U_Error AS5047U::getStickyErrorFlags() const {
    uint16_t val = stickyErrors.exchange(0);
    return static_cast<AS5047U_Error>(val);
}

// Public API implementations
void AS5047U::setPad(uint8_t pad) noexcept { padByte = pad; }

bool AS5047U::setHysteresis(AS5047U_REG::SETTINGS3::Hysteresis hys, uint8_t retries) {
    auto s3 = readReg<AS5047U_REG::SETTINGS3>();
    s3.bits.HYS = static_cast<uint8_t>(hys);
    return writeReg(s3, retries);
}

AS5047U_REG::SETTINGS3::Hysteresis AS5047U::getHysteresis() const {
    auto s3 = readReg<AS5047U_REG::SETTINGS3>();
    return static_cast<AS5047U_REG::SETTINGS3::Hysteresis>(s3.bits.HYS);
}

bool AS5047U::setAngleOutputSource(AS5047U_REG::SETTINGS2::AngleOutputSource src, uint8_t retries) {
    auto s2 = readReg<AS5047U_REG::SETTINGS2>();
    s2.bits.Data_select = static_cast<uint8_t>(src);
    return writeReg(s2, retries);
}

AS5047U_REG::SETTINGS2::AngleOutputSource AS5047U::getAngleOutputSource() const {
    auto s2 = readReg<AS5047U_REG::SETTINGS2>();
    return static_cast<AS5047U_REG::SETTINGS2::AngleOutputSource>(s2.bits.Data_select);
}

AS5047U_REG::DIA AS5047U::getDiagnostics() const {
    return readReg<AS5047U_REG::DIA>();
}

// ══════════════════════════════════════════════════════════════════════════════════════════
//                           PRIVATE - COMMUNICATION API
// ══════════════════════════════════════════════════════════════════════════════════════════

uint16_t AS5047U::readRegister(uint16_t address) const {
    uint16_t result = 0;
    if (frameFormat == FrameFormat::SPI_16) {
        // ---- 16-bit frame without CRC ----
        // Prepare read command (bit14=1 for read)
        uint16_t cmd = static_cast<uint16_t>(0x4000 | (address & 0x3FFF));
        uint8_t tx[2] = {
            static_cast<uint8_t>(cmd >> 8),
            static_cast<uint8_t>(cmd & 0xFF)
        };
        uint8_t rx[2];
        
        // Send read command
        spi.transfer(tx, rx, 2);
        
        // Send NOP to receive register data
        uint8_t txNOP[2] = {0x00, 0x00};
        uint8_t rxData[2];
        spi.transfer(txNOP, rxData, 2);
        
        // Process response
        uint16_t raw = (static_cast<uint16_t>(rxData[0]) << 8) | rxData[1];
        updateStickyErrors(readReg<AS5047U_REG::ERRFL>().value);
        result = raw & 0x3FFF;  // Mask out status flags
    } 
    else if (frameFormat == FrameFormat::SPI_24) {
        // ---- 24-bit frame with CRC ----
        // Prepare read command with CRC
        uint16_t crcInput = static_cast<uint16_t>((1 << 14) | (address & 0x3FFF));
        uint8_t crc = computeCRC8(crcInput);
        uint8_t txCmd[3] = {
            static_cast<uint8_t>(((address >> 8) & 0x3F) | 0x40),  // bit6=1 for read
            static_cast<uint8_t>(address & 0xFF),
            crc
        };
        uint8_t rxCmd[3];
        spi.transfer(txCmd, rxCmd, 3);
        
        // Send NOP to receive register data
        uint16_t nopAddr = AS5047U_REG::NOP::ADDRESS;
        uint16_t nopCrcInput = static_cast<uint16_t>((1 << 14) | (nopAddr & 0x3FFF));
        uint8_t crcNOP = computeCRC8(nopCrcInput);
        uint8_t txNOP[3] = {
            static_cast<uint8_t>(((nopAddr >> 8) & 0x3F) | 0x40),
            static_cast<uint8_t>(nopAddr & 0xFF),
            crcNOP
        };
        uint8_t rxDataFrame[3];
        spi.transfer(txNOP, rxDataFrame, 3);
        
        // Process response with CRC verification
        uint16_t raw = (static_cast<uint16_t>(rxDataFrame[0]) << 8) | rxDataFrame[1];
        uint8_t crcDevice = rxDataFrame[2];
        uint8_t crcCalc = computeCRC8(raw);
        
        if (crcDevice != crcCalc) {
            updateStickyErrors(readReg<AS5047U_REG::ERRFL>().value);
        }
        
        result = raw & 0x3FFF;
    } 
    else if (frameFormat == FrameFormat::SPI_32) {
        // ---- 32-bit frame with CRC and pad byte ----
        // Prepare read command with CRC and pad
        uint16_t crcInput = static_cast<uint16_t>((1 << 14) | (address & 0x3FFF));
        uint8_t crc = computeCRC8(crcInput);
        uint8_t txCmd[4] = {
            padByte,
            static_cast<uint8_t>(((address >> 8) & 0x3F) | 0x40),  // bit6=1 for read
            static_cast<uint8_t>(address & 0xFF),
            crc
        };
        uint8_t rxCmd[4];
        spi.transfer(txCmd, rxCmd, 4);
        
        // Send NOP to receive register data
        uint16_t nopAddr = AS5047U_REG::NOP::ADDRESS;
        uint16_t nopCrcInput = static_cast<uint16_t>((1 << 14) | (nopAddr & 0x3FFF));
        uint8_t crcNOP = computeCRC8(nopCrcInput);
        uint8_t txNOP[4] = {
            padByte,
            static_cast<uint8_t>(((nopAddr >> 8) & 0x3F) | 0x40),
            static_cast<uint8_t>(nopAddr & 0xFF),
            crcNOP
        };
        uint8_t rxDataFrame[4];
        spi.transfer(txNOP, rxDataFrame, 4);
        
        // Process response with CRC verification
        uint16_t raw = (static_cast<uint16_t>(rxDataFrame[1]) << 8) | rxDataFrame[2];
        uint8_t crcDevice = rxDataFrame[3];
        uint8_t crcCalc = computeCRC8(raw);
        
        if (crcDevice != crcCalc) {
            updateStickyErrors(readReg<AS5047U_REG::ERRFL>().value);
        }
        
        result = raw & 0x3FFF;
    }
    return result;
}

bool AS5047U::writeRegister(uint16_t address, uint16_t value, uint8_t retries) const {
    bool success = false;
    uint16_t errMask = static_cast<uint16_t>(AS5047U_Error::CrcError) | static_cast<uint16_t>(AS5047U_Error::FramingError);
    for (uint8_t attempt = 0; attempt <= retries; ++attempt) {
        if (frameFormat == FrameFormat::SPI_16) {
            // ---- 16-bit write (two frames) ----
            // First frame: send address
            uint16_t cmd = static_cast<uint16_t>(address & 0x3FFF);  // bit14=0 for write
            uint8_t tx[2] = {
                static_cast<uint8_t>(cmd >> 8),
                static_cast<uint8_t>(cmd & 0xFF)
            };
            uint8_t rx_dummy[2];
            spi.transfer(tx, rx_dummy, 2);
            
            // Second frame: send data payload
            uint16_t dataFrame = value & 0x3FFF;
            tx[0] = static_cast<uint8_t>(dataFrame >> 8);
            tx[1] = static_cast<uint8_t>(dataFrame & 0xFF);
            uint8_t rxDataResp[2];
            spi.transfer(tx, rxDataResp, 2);
            
            // Check for errors from previous command
            auto err = readReg<AS5047U_REG::ERRFL>().value;
            if (!(err & errMask)) { success = true; break; }
            updateStickyErrors(err);
        } 
        else if (frameFormat == FrameFormat::SPI_24) {
            // ---- 24-bit write with CRC ----
            // First frame: send address with CRC
            uint16_t cmdPayload = static_cast<uint16_t>((0 << 14) | (address & 0x3FFF));
            uint8_t cmdCrc = computeCRC8(cmdPayload);
            uint8_t txCmd[3] = {
                static_cast<uint8_t>(((address >> 8) & 0x3F) | 0x00),  // bit6=0 for write
                static_cast<uint8_t>(address & 0xFF),
                cmdCrc
            };
            uint8_t rxCmd[3];
            spi.transfer(txCmd, rxCmd, 3);
            
            // Second frame: send data with CRC
            uint16_t dataPayload = value & 0x3FFF;
            uint8_t dataCrc = computeCRC8(dataPayload);
            uint8_t txData[3] = {
                static_cast<uint8_t>((dataPayload >> 8) & 0xFF),
                static_cast<uint8_t>(dataPayload & 0xFF),
                dataCrc
            };
            uint8_t rxData[3];
            spi.transfer(txData, rxData, 3);
            
            // after write, check error flags
            auto err = readReg<AS5047U_REG::ERRFL>().value;
            if (!(err & errMask)) { success = true; break; }
            updateStickyErrors(err);
        } 
        else if (frameFormat == FrameFormat::SPI_32) {
            // ---- 32-bit write with CRC and pad byte ----
            // First frame: send address with CRC and pad
            uint16_t cmdPayload = static_cast<uint16_t>((0 << 14) | (address & 0x3FFF));
            uint8_t cmdCrc = computeCRC8(cmdPayload);
            uint8_t txCmd[4] = {
                padByte,
                static_cast<uint8_t>(((address >> 8) & 0x3F) | 0x00),  // bit6=0 for write
                static_cast<uint8_t>(address & 0xFF),
                cmdCrc
            };
            uint8_t rxCmd[4];
            spi.transfer(txCmd, rxCmd, 4);
            
            // Second frame: send data with CRC and pad
            uint16_t dataPayload = value & 0x3FFF;
            uint8_t dataCrc = computeCRC8(dataPayload);
            uint8_t txData[4] = {
                padByte,
                static_cast<uint8_t>((dataPayload >> 8) & 0xFF),
                static_cast<uint8_t>(dataPayload & 0xFF),
                dataCrc
            };
            uint8_t rxData[4];
            spi.transfer(txData, rxData, 4);
            
            // after write, check error flags
            auto err = readReg<AS5047U_REG::ERRFL>().value;
            if (!(err & errMask)) { success = true; break; }
            updateStickyErrors(err);
        }
    }
    return success;
}



// ════════════════════════════════════════════════════════════════════════════════════════════
//                       Public API: retry-enabled getters and status dump
// ════════════════════════════════════════════════════════════════════════════════════════════

// Complete dumpStatus with full register dump
void AS5047U::dumpStatus() const {
    printf("\n=== AS5047U Comprehensive Status ===\n");
    // Core measurements
    printf("Angle (COM) : %u\n", getAngle());
    printf("Angle (UNC) : %u\n", getRawAngle());
    printf("Velocity    : %d counts (%.3f deg/s, %.3f rad/s, %.3f RPM)\n",
           getVelocity(), getVelocityDegPerSec(), getVelocityRadPerSec(), getVelocityRPM());
    printf("AGC         : %u\n", getAGC());
    printf("Magnitude   : %u\n", getMagnitude());
    // Error flags
    uint16_t err = getErrorFlags();
    printf("ERRFL       : 0x%04X\n", err);
    // Diagnostic register bits
    auto dia = readReg<AS5047U_REG::DIA>();
    printf("DIA (0x3FF5): 0x%04X\n", dia.value);
    printf("  VDD_mode         : %u\n", dia.bits.VDD_mode);
    printf("  LoopsFinished    : %u\n", dia.bits.LoopsFinished);
    printf("  CORDIC_overflow  : %u\n", dia.bits.CORDIC_overflow_flag);
    printf("  Comp_l           : %u\n", dia.bits.Comp_l);
    printf("  Comp_h           : %u\n", dia.bits.Comp_h);
    printf("  MagHalf_flag     : %u\n", dia.bits.MagHalf_flag);
    printf("  CosOff_fin       : %u\n", dia.bits.CosOff_fin);
    printf("  SinOff_fin       : %u\n", dia.bits.SinOff_fin);
    printf("  OffComp_finished : %u\n", dia.bits.OffComp_finished);
    printf("  AGC_finished     : %u\n", dia.bits.AGC_finished);
    printf("  SPI_cnt          : %u\n", dia.bits.SPI_cnt);
    // Configuration registers
    auto dis = readReg<AS5047U_REG::DISABLE>();
    printf("DISABLE (0x0015): 0x%04X UVW_off=%u ABI_off=%u FILTER_disable=%u\n",
           dis.value, dis.bits.UVW_off, dis.bits.ABI_off, dis.bits.FILTER_disable);
    auto s1 = readReg<AS5047U_REG::SETTINGS1>();
    printf("SETTINGS1(0x0016): K_max=%u K_min=%u Dia3_en=%u Dia4_en=%u\n",
           s1.bits.K_max, s1.bits.K_min, s1.bits.Dia3_en, s1.bits.Dia4_en);
    auto s2 = readReg<AS5047U_REG::SETTINGS2>();
    printf("SETTINGS2(0x0019): IWIDTH=%u NOISESET=%u DIR=%u UVW_ABI=%u DAECDIS=%u ABI_DEC=%u Data_select=%u PWMon=%u\n",
           s2.bits.IWIDTH, s2.bits.NOISESET, s2.bits.DIR, s2.bits.UVW_ABI,
           s2.bits.DAECDIS, s2.bits.ABI_DEC, s2.bits.Data_select, s2.bits.PWMon);
    auto s3 = readReg<AS5047U_REG::SETTINGS3>();
    printf("SETTINGS3(0x001A): UVWPP=%u HYS=%u ABIRES=%u\n",
           s3.bits.UVWPP, s3.bits.HYS, s3.bits.ABIRES);
    // Other registers
    auto sind = readReg<AS5047U_REG::SINDATA>();
    printf("SINDATA(0x3FFA): %d\n", sind.bits.SINDATA);
    auto eccsum = readReg<AS5047U_REG::ECC_Checksum>();
    printf("ECC_Checksum(0x3FD0): %u\n", eccsum.bits.ECC_s);
    auto prog = readReg<AS5047U_REG::PROG>();
    printf("PROG(0x0003): PROGEN=%u PROGOTP=%u OTPREF=%u PROGVER=%u\n",
           prog.bits.PROGEN, prog.bits.PROGOTP, prog.bits.OTPREF, prog.bits.PROGVER);
    // Interface settings
    printf("FrameFormat      : %u  PadByte=0x%02X\n", static_cast<uint8_t>(frameFormat), padByte);
    printf("========================================\n\n");
}
