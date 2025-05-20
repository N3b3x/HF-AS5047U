#include "AS5047U.hpp"

// Inline function definitions
inline void AS5047U::setFrameFormat(FrameFormat format) {
    frameFormat = format;
}

// Constructor implementation
AS5047U::AS5047U(spiBus &bus, FrameFormat frameFormat)
    : spi(bus), frameFormat(frameFormat) {
    // No further initialization (use sensor defaults unless configured).
}
uint16_t AS5047U::readRegister(uint16_t address, uint8_t padByte = 0x00) {
    uint16_t result = 0;
    if(frameFormat == FrameFormat::SPI_16) {
        // 16-bit frame without CRC (no PAD byte used)
        uint16_t cmd = static_cast<uint16_t>(0x4000 | (address & 0x3FFF)); // bit14=1 for read
        uint8_t tx[2], rx[2];
        // Send read command (16 bits)
        tx[0] = static_cast<uint8_t>(cmd >> 8);
        tx[1] = static_cast<uint8_t>(cmd & 0xFF);
        spi.transfer(tx, rx, 2);
        // Send NOP (no-op) to receive the register data
        uint8_t txNOP[2] = {0x00, 0x00};
        uint8_t rxData[2];
        spi.transfer(txNOP, rxData, 2);
        uint16_t raw = (static_cast<uint16_t>(rxData[0]) << 8) | rxData[1];
        // Mask out status flags (bits 15,14) and keep 14-bit data
        result = raw & 0x3FFF;
    } else if(frameFormat == FrameFormat::SPI_24) {
        // 24-bit frame with CRC (8-bit CRC for each 16-bit word) (no PAD byte used)
        uint16_t crcInput = static_cast<uint16_t>((1 << 14) | (address & 0x3FFF)); // RW=1 at bit14 for read
        uint8_t crc = computeCRC8(crcInput);
        uint8_t txCmd[3];
        txCmd[0] = static_cast<uint8_t>(((address >> 8) & 0x3F) | 0x40); // bit6=1 (read), bits5-0 = addr[13:8]
        txCmd[1] = static_cast<uint8_t>(address & 0xFF);
        txCmd[2] = crc;
        uint8_t rxCmd[3];
        spi.transfer(txCmd, rxCmd, 3);
        // Second frame (NOP) to receive data
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
        uint16_t raw = (static_cast<uint16_t>(rxDataFrame[0]) << 8) | rxDataFrame[1];
        uint8_t crcDevice = rxDataFrame[2];
        uint8_t crcCalc = computeCRC8(raw);
        if (crcDevice != crcCalc) {
            // CRC error: read and update sticky error flags
            updateStickyErrors(readReg<AS5047U_REG::ERRFL>().value);
        }
        result = raw & 0x3FFF;
    } else if(frameFormat == FrameFormat::SPI_32) {
        // 32-bit frame with CRC and pad - using the provided padByte
        uint16_t crcInput = static_cast<uint16_t>((1 << 14) | (address & 0x3FFF));
        uint8_t crc = computeCRC8(crcInput);
        uint8_t txCmd[4] = {
            padByte, // Use the provided PAD byte
            static_cast<uint8_t>(((address >> 8) & 0x3F) | 0x40),
            static_cast<uint8_t>(address & 0xFF),
            crc
        };
        uint8_t rxCmd[4];
        spi.transfer(txCmd, rxCmd, 4);
        // Second frame (NOP) to get data
        uint16_t nopAddr = AS5047U_REG::NOP::ADDRESS;
        uint16_t nopCrcInput = static_cast<uint16_t>((1 << 14) | (nopAddr & 0x3FFF));
        uint8_t crcNOP = computeCRC8(nopCrcInput);
        uint8_t txNOP[4] = {
            padByte, // Use the provided PAD byte
            static_cast<uint8_t>(((nopAddr >> 8) & 0x3F) | 0x40),
            static_cast<uint8_t>(nopAddr & 0xFF),
            crcNOP
        };
        uint8_t rxDataFrame[4];
        spi.transfer(txNOP, rxDataFrame, 4);
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
void AS5047U::writeRegister(uint16_t address, uint16_t value, uint8_t padByte = 0x00) {
    if(frameFormat == FrameFormat::SPI_16) {
        // 16-bit write (two 16-bit frames)
        uint16_t cmd = static_cast<uint16_t>(address & 0x3FFF); // bit14=0 for write
        uint8_t tx[2] = {
            static_cast<uint8_t>(cmd >> 8),
            static_cast<uint8_t>(cmd & 0xFF)
        };
        uint8_t rx_dummy[2];
        spi.transfer(tx, rx_dummy, 2);
        // Second frame: send 14-bit data payload
        uint16_t dataFrame = value & 0x3FFF;
        tx[0] = static_cast<uint8_t>(dataFrame >> 8);
        tx[1] = static_cast<uint8_t>(dataFrame & 0xFF);
        uint8_t rxDataResp[2];
        spi.transfer(tx, rxDataResp, 2);
        // (MISO returns status of previous command, not explicitly checked here)
    } else if(frameFormat == FrameFormat::SPI_24) {
        // 24-bit write with CRC
        // Per docs: bits 23=0 (don't care), 22=0 (write), 21:8=address, 7:0=CRC
        uint16_t cmdPayload = static_cast<uint16_t>((0 << 14) | (address & 0x3FFF));
        uint8_t cmdCrc = computeCRC8(cmdPayload);
        uint8_t txCmd[3] = {
            static_cast<uint8_t>(((address >> 8) & 0x3F) | 0x00), // bit6=0 (write)
            static_cast<uint8_t>(address & 0xFF),
            cmdCrc
        };
        uint8_t rxCmd[3];
        spi.transfer(txCmd, rxCmd, 3);
        // Second frame: send data + CRC
        uint16_t dataPayload = value & 0x3FFF;
        uint8_t dataCrc = computeCRC8(dataPayload);
        uint8_t txData[3] = {
            static_cast<uint8_t>((dataPayload >> 8) & 0xFF),
            static_cast<uint8_t>(dataPayload & 0xFF),
            dataCrc
        };
        uint8_t rxData[3];
        spi.transfer(txData, rxData, 3);
    } else if(frameFormat == FrameFormat::SPI_32) {
        // 32-bit write with CRC and pad
        // Per docs: bits 31:24=PAD, 23=0 (don't care), 22=0 (write), 21:8=address, 7:0=CRC
        uint16_t cmdPayload = static_cast<uint16_t>((0 << 14) | (address & 0x3FFF));
        uint8_t cmdCrc = computeCRC8(cmdPayload);
        uint8_t txCmd[4] = {
            padByte, // Use the provided PAD byte
            static_cast<uint8_t>(((address >> 8) & 0x3F) | 0x00), // bit6=0 (write)
            static_cast<uint8_t>(address & 0xFF),
            cmdCrc
        };
        uint8_t rxCmd[4];
        spi.transfer(txCmd, rxCmd, 4);
        // Second frame: send data + CRC
        uint16_t dataPayload = value & 0x3FFF;
        uint8_t dataCrc = computeCRC8(dataPayload);
        uint8_t txData[4] = {
            padByte, // Use the provided PAD byte
            static_cast<uint8_t>((dataPayload >> 8) & 0xFF),
            static_cast<uint8_t>(dataPayload & 0xFF),
            dataCrc
        };
        uint8_t rxData[4];
        spi.transfer(txData, rxData, 4);
    }
}

uint8_t AS5047U::computeCRC8(uint16_t data16) const {
    // CRC-8 with polynomial 0x1D, initial value 0xC4, final XOR 0xFF
    // Reference implementation: no reflection, final XOR 0xFF
    uint8_t crc = 0xC4;
    for (int i = 0; i < 16; ++i) {
        bool bit = ((data16 >> (15 - i)) & 1) ^ ((crc >> 7) & 1);
        crc = (crc << 1) ^ (bit ? 0x1D : 0x00);
    }
    crc ^= 0xFF;
    return crc;
}
// CRC-8 self-test (static_asserts, will fail build if regression occurs)
static_assert(AS5047U().computeCRC8(0x3FFF) == 0xF3, "CRC8(0x3FFF) should be 0xF3");
static_assert(AS5047U().computeCRC8(0x0000) == 0x3B, "CRC8(0x0000) should be 0x3B");
static_assert(AS5047U().computeCRC8(0x1234) == 0xB2, "CRC8(0x1234) should be 0xB2");

// ══════════════════════════════════════════════════════════════════════════════════════════
//                                PRIVATE HELPERS
// ══════════════════════════════════════════════════════════════════════════════════════════

// Helper: encode/decode register value

/**
 * @brief Anonymous namespace for SPI encoding and decoding functions.
 * 
 * Contains utility templates for converting between register types and raw uint16_t values
 * for SPI communication with the AS5047U magnetic encoder.
 */
namespace {
/**
 * @brief Encodes a register object to its raw 16-bit representation.
 * 
 * @tparam RegT The register type to encode.
 * @param r The register instance to encode.
 * @return uint16_t The raw 16-bit value representing the register.
 */
template <typename RegT>
static inline uint16_t encode(const RegT& r) { return r.value; }

/**
 * @brief Decodes a raw 16-bit value into a typed register object.
 * 
 * @tparam RegT The register type to decode into.
 * @param raw The raw 16-bit value to decode.
 * @return RegT A register object with the given value.
 */
template <typename RegT>
static inline RegT decode(uint16_t raw) { RegT r{}; r.value = raw; return r; }
}

// ══════════════════════════════════════════════════════════════════════════════════════════
//                                 PUBLIC HIGH-LEVEL API
// ══════════════════════════════════════════════════════════════════════════════════════════

uint16_t AS5047U::getAngle() { return readReg<AS5047U_REG::ANGLECOM>().bits.ANGLECOM_value; }
uint16_t AS5047U::getRawAngle() { return readReg<AS5047U_REG::ANGLEUNC>().bits.ANGLEUNC_value; }
int16_t AS5047U::getVelocity() {
    auto v = readReg<AS5047U_REG::VEL>().bits.VEL_value;
    // Sign-extend 14-bit value to int16_t (datasheet: VEL[13:0], sign at bit 13)
    return static_cast<int16_t>((static_cast<int16_t>(v << 2)) >> 2);
}
uint8_t AS5047U::getAGC() { return readReg<AS5047U_REG::AGC>().bits.AGC_value; }
uint16_t AS5047U::getMagnitude() { return readReg<AS5047U_REG::MAG>().bits.MAG_value; }
uint16_t AS5047U::getErrorFlags() { return readReg<AS5047U_REG::ERRFL>().value; }

void AS5047U::setZeroPosition(uint16_t angleOffset) {
    angleOffset &= 0x3FFF;
    AS5047U_REG::ZPOSM m{}; m.bits.ZPOSM_bits = (angleOffset >> 6) & 0xFF;
    AS5047U_REG::ZPOSL l{}; l.bits.ZPOSL_bits = angleOffset & 0x3F;
    writeReg(m); writeReg(l);
}

uint16_t AS5047U::getZeroPosition() const {
    auto m = const_cast<AS5047U*>(this)->readReg<AS5047U_REG::ZPOSM>().bits.ZPOSM_bits;
    auto l = const_cast<AS5047U*>(this)->readReg<AS5047U_REG::ZPOSL>().bits.ZPOSL_bits;
    return static_cast<uint16_t>((m << 6) | l);
}

void AS5047U::setDirection(bool clockwise) {
    auto s2 = readReg<AS5047U_REG::SETTINGS2>();
    s2.bits.DIR = clockwise ? 0 : 1;
    writeReg(s2);
}

void AS5047U::setABIResolution(uint8_t resolution_bits) {
    resolution_bits = std::clamp<uint8_t>(resolution_bits, 10, 14);
    auto s3 = readReg<AS5047U_REG::SETTINGS3>();
    s3.bits.ABIRES = resolution_bits - 10;
    writeReg(s3);
}

void AS5047U::setUVWPolePairs(uint8_t polePairs) {
    polePairs = std::clamp<uint8_t>(polePairs, 1, 7);
    auto s3 = readReg<AS5047U_REG::SETTINGS3>();
    s3.bits.UVWPP = polePairs;
    writeReg(s3);
}

void AS5047U::setIndexPulseLength(uint8_t pulseLengthLSB) {
    auto s2 = readReg<AS5047U_REG::SETTINGS2>();
    s2.bits.IWIDTH = (pulseLengthLSB == 1) ? 1 : 0;
    writeReg(s2);
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
void AS5047U::configureInterface(bool abi, bool uvw, bool pwm) {
    auto dis = readReg<AS5047U_REG::DISABLE>();
    auto s2  = readReg<AS5047U_REG::SETTINGS2>();
    dis.bits.ABI_off = abi ? 0 : 1;
    dis.bits.UVW_off = uvw ? 0 : 1;
    if (abi && !uvw) {
        s2.bits.UVW_ABI = 0;
        s2.bits.PWMon   = pwm;
    } else if (!abi && uvw) {
        s2.bits.UVW_ABI = 1;
        s2.bits.PWMon   = pwm;
    } else {
        s2.bits.PWMon   = pwm;
        s2.bits.UVW_ABI = 0;
    }
    writeReg(dis); writeReg(s2);
}

void AS5047U::setDynamicAngleCompensation(bool enable) {
    auto s2 = readReg<AS5047U_REG::SETTINGS2>();
    s2.bits.DAECDIS = enable ? 0 : 1;
    writeReg(s2);
}

void AS5047U::setAdaptiveFilter(bool enable) {
    auto dis = readReg<AS5047U_REG::DISABLE>();
    dis.bits.FILTER_disable = enable ? 0 : 1;
    writeReg(dis);
}

void AS5047U::setFilterParameters(uint8_t k_min, uint8_t k_max) {
    k_min = std::min<uint8_t>(k_min, 7); k_max = std::min<uint8_t>(k_max, 7);
    auto s1 = readReg<AS5047U_REG::SETTINGS1>();
    s1.bits.K_min = k_min;
    s1.bits.K_max = k_max;
    writeReg(s1);
}

void AS5047U::set150CTemperatureMode(bool enable) {
    auto s2 = readReg<AS5047U_REG::SETTINGS2>();
    s2.bits.NOISESET = enable ? 1 : 0;
    writeReg(s2);
}

bool AS5047U::programOTP() {
    FrameFormat backup = frameFormat;
    if (frameFormat == FrameFormat::SPI_16) frameFormat = FrameFormat::SPI_24;
    setZeroPosition(getAngle());
    uint16_t volatileShadow[5];
    for (uint16_t a = 0x0016; a <= 0x001A; ++a) volatileShadow[a-0x0016] = readRegister(a);
    auto ecc = readReg<AS5047U_REG::ECC>();
    ecc.bits.ECC_en = 1;
    writeReg(ecc);
    auto key = readReg<AS5047U_REG::ECC_Checksum>().bits.ECC_s;
    ecc.bits.ECC_chsum = key;
    writeReg(ecc);
    for (uint16_t a = 0x0016; a <= 0x001A; ++a)
        if (readRegister(a) != volatileShadow[a-0x0016]) { frameFormat = backup; return false; }
    AS5047U_REG::PROG p{};  p.bits.PROGEN = 1;  writeReg(p);
    p.bits.PROGOTP = 1;     writeReg(p);
    for (uint16_t i=0;i<15000;++i)
        if (readRegister(PROG_ADDR) == 0x0001) { frameFormat = backup; return true; }
    frameFormat = backup;
    return false;
}

void AS5047U::updateStickyErrors(uint16_t errfl) {
    // Map ERRFL bits to sticky error enum
    if (errfl & (1 << 6)) stickyErrors |= static_cast<uint8_t>(AS5047U_Error::CRC);
    if (errfl & (1 << 4)) stickyErrors |= static_cast<uint8_t>(AS5047U_Error::Framing);
    if (errfl & (1 << 5)) stickyErrors |= static_cast<uint8_t>(AS5047U_Error::Command);
    if (errfl & (1 << 7)) stickyErrors |= static_cast<uint8_t>(AS5047U_Error::Watchdog);
    if (errfl & (1 << 0)) stickyErrors |= static_cast<uint8_t>(AS5047U_Error::AGCWarning);
    if (errfl & (1 << 1)) stickyErrors |= static_cast<uint8_t>(AS5047U_Error::MagHalf);
    if (errfl & (1 << 2)) stickyErrors |= static_cast<uint8_t>(AS5047U_Error::P2RAMWarn);
    if (errfl & (1 << 3)) stickyErrors |= static_cast<uint8_t>(AS5047U_Error::P2RAMError);
}

AS5047U_Error AS5047U::getStickyErrorFlags() {
    uint8_t val = stickyErrors.exchange(0);
    return static_cast<AS5047U_Error>(val);
}
