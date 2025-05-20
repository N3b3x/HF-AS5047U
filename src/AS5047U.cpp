
#include "AS5047U.hpp"

// Inline function definitions
inline void AS5047U::setFrameFormat(FrameFormat format) {
    frameFormat = format;
}

// Implementation of constructor
AS5047U::AS5047U(spiBus &bus, FrameFormat frameFormat)
    : spi(bus), frameFormat(frameFormat) {
    // Nothing else to initialize; sensor default registers will be used unless changed by user.
}

uint16_t AS5047U::readRegister(uint16_t address) {
    uint16_t result = 0;
    if(frameFormat == FrameFormat::SPI_16) {
        // 16-bit frame without CRC
        uint16_t cmd = static_cast<uint16_t>(0x4000 | (address & 0x3FFF)); // bit14=1 for read, bit15 ignored
        uint8_t tx[2];
        uint8_t rx[2];
        // Send read command (16 bits)
        tx[0] = static_cast<uint8_t>(cmd >> 8);
        tx[1] = static_cast<uint8_t>(cmd & 0xFF);
        spi.transfer(tx, rx, 2);
        // Send NOP to receive data
        uint16_t nopCmd = 0x0000;
        tx[0] = 0x00;
        tx[1] = 0x00;
        uint8_t rxData[2];
        spi.transfer(tx, rxData, 2);
        uint16_t raw = (static_cast<uint16_t>(rxData[0]) << 8) | rxData[1];
        // Mask out error/warning flags (bits15,14) and keep 14-bit data
        result = raw & 0x3FFF;
    } else if(frameFormat == FrameFormat::SPI_24) {
        // 24-bit frame with CRC
        uint16_t crcInput = static_cast<uint16_t>((1 << 14) | (address & 0x3FFF)); // prepare 16-bit command (RW=1 at bit14)
        uint8_t crc = computeCRC8(crcInput);
        uint8_t txCmd[3];
        txCmd[0] = static_cast<uint8_t>(((address >> 8) & 0x3F) | 0x40); // bit6=1 (read), bits5-0 = addr[13:8]
        txCmd[1] = static_cast<uint8_t>(address & 0xFF);
        txCmd[2] = crc;
        uint8_t rxCmd[3];
        spi.transfer(txCmd, rxCmd, 3);
        // Second 24-bit frame (NOP) to receive the data
        uint16_t nopAddr = REG_NOP;
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
        uint8_t crcCalc = computeCRC8(raw);
        (void)crcCalc; // ignore computed CRC here (device sets CRCERR flag on mismatch)
        result = raw & 0x3FFF;
    } else if(frameFormat == FrameFormat::SPI_32) {
        // 32-bit frame with CRC and pad
        uint16_t crcInput = static_cast<uint16_t>((1 << 14) | (address & 0x3FFF));
        uint8_t crc = computeCRC8(crcInput);
        uint8_t txCmd[4] = {
            0x00, // pad byte
            static_cast<uint8_t>(((address >> 8) & 0x3F) | 0x40),
            static_cast<uint8_t>(address & 0xFF),
            crc
        };
        uint8_t rxCmd[4];
        spi.transfer(txCmd, rxCmd, 4);
        // Second 32-bit frame (NOP) to get data
        uint16_t nopAddr = REG_NOP;
        uint16_t nopCrcInput = static_cast<uint16_t>((1 << 14) | (nopAddr & 0x3FFF));
        uint8_t crcNOP = computeCRC8(nopCrcInput);
        uint8_t txNOP[4] = {
            0x00,
            static_cast<uint8_t>(((nopAddr >> 8) & 0x3F) | 0x40),
            static_cast<uint8_t>(nopAddr & 0xFF),
            crcNOP
        };
        uint8_t rxDataFrame[4];
        spi.transfer(txNOP, rxDataFrame, 4);
        uint16_t raw = (static_cast<uint16_t>(rxDataFrame[1]) << 8) | rxDataFrame[2];
        uint8_t rxCrc = rxDataFrame[3];
        uint8_t crcCalc = computeCRC8(raw);
        (void)rxCrc;
        (void)crcCalc;
        result = raw & 0x3FFF;
    }
    return result;
}

void AS5047U::writeRegister(uint16_t address, uint16_t value) {
    if(frameFormat == FrameFormat::SPI_16) {
        // 16-bit write (two 16-bit frames)
        uint16_t cmd = static_cast<uint16_t>(address & 0x3FFF); // bit14=0 (write)
        uint8_t tx[2] = { static_cast<uint8_t>(cmd >> 8), static_cast<uint8_t>(cmd & 0xFF) };
        uint8_t rx_dummy[2];
        spi.transfer(tx, rx_dummy, 2);
        // Second frame: send 14-bit data
        uint16_t dataFrame = value & 0x3FFF;
        tx[0] = static_cast<uint8_t>(dataFrame >> 8);
        tx[1] = static_cast<uint8_t>(dataFrame & 0xFF);
        uint8_t rxDataResp[2];
        spi.transfer(tx, rxDataResp, 2);
        // (MISO returns status of previous command, if any; not explicitly checked here)
    } else if(frameFormat == FrameFormat::SPI_24) {
        // 24-bit write with CRC
        uint16_t cmdPayload = static_cast<uint16_t>((0 << 14) | (address & 0x3FFF));
        uint8_t cmdCrc = computeCRC8(cmdPayload);
        uint8_t txCmd[3] = {
            static_cast<uint8_t>(((address >> 8) & 0x3F) | 0x00),
            static_cast<uint8_t>(address & 0xFF),
            cmdCrc
        };
        uint8_t rxCmd[3];
        spi.transfer(txCmd, rxCmd, 3);
        // Second 24-bit frame: send data + CRC
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
        uint16_t cmdPayload = static_cast<uint16_t>((0 << 14) | (address & 0x3FFF));
        uint8_t cmdCrc = computeCRC8(cmdPayload);
        uint8_t txCmd[4] = {
            0x00,
            static_cast<uint8_t>(((address >> 8) & 0x3F) | 0x00),
            static_cast<uint8_t>(address & 0xFF),
            cmdCrc
        };
        uint8_t rxCmd[4];
        spi.transfer(txCmd, rxCmd, 4);
        uint16_t dataPayload = value & 0x3FFF;
        uint8_t dataCrc = computeCRC8(dataPayload);
        uint8_t txData[4] = {
            0x00,
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
    uint8_t crc = 0xC4;
    uint16_t data = data16;
    for(int i = 0; i < 16; ++i) {
        uint8_t bit = (data & 0x8000) ? 1 : 0;
        data <<= 1;
        if(crc & 0x80) {
            crc = static_cast<uint8_t>((crc << 1) ^ 0x1D);
        } else {
            crc <<= 1;
        }
        if(bit) {
            crc ^= 0x1D;
        }
    }
    crc ^= 0xFF;
    return crc;
}

// Public method implementations

uint16_t AS5047U::getAngle() {
    return readRegister(REG_ANGLECOM);
}

uint16_t AS5047U::getRawAngle() {
    return readRegister(REG_ANGLEUNC);
}

int16_t AS5047U::getVelocity() {
    uint16_t raw = readRegister(REG_VEL);
    int16_t vel = static_cast<int16_t>(raw);
    if(raw & 0x2000) {
        vel |= 0xC000; // sign-extend if bit13 (sign bit of 14-bit value) is set
    }
    return vel;
}

uint8_t AS5047U::getAGC() {
    uint16_t raw = readRegister(REG_AGC);
    return static_cast<uint8_t>(raw & 0x00FF);
}

uint16_t AS5047U::getMagnitude() {
    uint16_t raw = readRegister(REG_MAG);
    return raw & 0x3FFF;
}

uint16_t AS5047U::getErrorFlags() {
    return readRegister(REG_ERRFL);
}

void AS5047U::setZeroPosition(uint16_t angleOffset) {
    uint16_t angle = angleOffset & 0x3FFF;
    uint16_t zposm = (angle >> 6) & 0x00FF;
    uint16_t zposl = angle & 0x003F;
    writeRegister(REG_ZPOSM, zposm);
    writeRegister(REG_ZPOSL, zposl);
}

uint16_t AS5047U::getZeroPosition() const {
    AS5047U *nonConst = const_cast<AS5047U*>(this);
    uint16_t msb = nonConst->readRegister(REG_ZPOSM) & 0x00FF;
    uint16_t lsb = nonConst->readRegister(REG_ZPOSL) & 0x003F;
    return static_cast<uint16_t>((msb << 6) | lsb);
}

void AS5047U::setDirection(bool clockwise) {
    uint16_t reg = readRegister(REG_SETTINGS2);
    if(clockwise) {
        reg &= ~0x0004;
    } else {
        reg |= 0x0004;
    }
    writeRegister(REG_SETTINGS2, reg);
}

void AS5047U::setABIResolution(uint8_t resolution_bits) {
    if(resolution_bits < 10) resolution_bits = 10;
    if(resolution_bits > 14) resolution_bits = 14;
    uint8_t code = resolution_bits - 10;
    uint16_t reg = readRegister(REG_SETTINGS3);
    reg &= ~0x00E0;
    reg |= static_cast<uint16_t>((code & 0x07) << 5);
    writeRegister(REG_SETTINGS3, reg);
}

void AS5047U::setUVWPolePairs(uint8_t polePairs) {
    if(polePairs < 1) polePairs = 1;
    if(polePairs > 7) polePairs = 7;
    uint16_t reg = readRegister(REG_SETTINGS3);
    reg &= ~0x0007;
    reg |= static_cast<uint16_t>(polePairs & 0x07);
    writeRegister(REG_SETTINGS3, reg);
}

void AS5047U::setIndexPulseLength(uint8_t pulseLengthLSB) {
    uint16_t reg = readRegister(REG_SETTINGS2);
    if(pulseLengthLSB == 1) {
        reg |= 0x0001;
    } else {
        reg &= ~0x0001;
    }
    writeRegister(REG_SETTINGS2, reg);
}

void AS5047U::configureInterface(bool abi, bool uvw, bool pwm) {
    uint16_t disableReg = readRegister(REG_DISABLE);
    uint16_t settings2 = readRegister(REG_SETTINGS2);
    // Configure interface enable/disable bits
    if(abi) disableReg &= ~0x0002; else disableReg |= 0x0002;
    if(uvw) disableReg &= ~0x0001; else disableReg |= 0x0001;
    // Determine PWM routing based on active interfaces
    if(abi && !uvw) {
        settings2 &= ~0x0008; // UVW_ABI = 0 (ABI mode)
        if(pwm) {
            settings2 |= 0x0080;    // enable PWM on W
            disableReg |= 0x0001;   // ensure UVW outputs off
        } else {
            settings2 &= ~0x0080;
        }
    } else if(!abi && uvw) {
        settings2 |= 0x0008; // UVW_ABI = 1 (UVW mode)
        if(pwm) {
            settings2 |= 0x0080;    // enable PWM on I
            disableReg |= 0x0002;   // ensure ABI outputs off
        } else {
            settings2 &= ~0x0080;
        }
    } else if(abi && uvw) {
        // Both interfaces on, PWM not available
        disableReg &= ~0x0001;
        disableReg &= ~0x0002;
        settings2 &= ~0x0080;
        settings2 &= ~0x0008;
    } else {
        // Both off
        disableReg |= 0x0001;
        disableReg |= 0x0002;
        if(pwm) {
            settings2 &= ~0x0008;
            settings2 |= 0x0080;   // PWM on W by default (ABI mode)
        } else {
            settings2 &= ~0x0080;
        }
    }
    writeRegister(REG_DISABLE, disableReg);
    writeRegister(REG_SETTINGS2, settings2);
}

void AS5047U::setDynamicAngleCompensation(bool enabled) {
    uint16_t reg = readRegister(REG_SETTINGS2);
    if(enabled) {
        reg &= ~0x0010;
    } else {
        reg |= 0x0010;
    }
    writeRegister(REG_SETTINGS2, reg);
}

void AS5047U::setAdaptiveFilter(bool enabled) {
    uint16_t reg = readRegister(REG_DISABLE);
    if(enabled) {
        reg &= ~0x0040;
    } else {
        reg |= 0x0040;
    }
    writeRegister(REG_DISABLE, reg);
}

void AS5047U::setFilterParameters(uint8_t k_min, uint8_t k_max) {
    if(k_min > 7) k_min = 7;
    if(k_max > 7) k_max = 7;
    uint16_t reg = readRegister(REG_SETTINGS1);
    reg &= ~0x003F;
    reg |= static_cast<uint16_t>((k_max & 0x07) | ((k_min & 0x07) << 3));
    writeRegister(REG_SETTINGS1, reg);
}

void AS5047U::setNoiseModeHigh(bool highNoise) {
    uint16_t reg = readRegister(REG_SETTINGS2);
    if(highNoise) {
        reg |= 0x0002;
    } else {
        reg &= ~0x0002;
    }
    writeRegister(REG_SETTINGS2, reg);
}

bool AS5047U::programOTP() {
    // For reliability, use 24-bit frames for programming if not already in CRC mode
    FrameFormat prevFormat = frameFormat;
    if(frameFormat == FrameFormat::SPI_16) {
        frameFormat = FrameFormat::SPI_24;
    }
    // 1. (Power-on assumed done externally)
    // 2. (Custom settings should have been written via set* methods)
    // 3. (Magnet placed at desired zero position by user)
    // 4. Read current angle (with compensation)
    uint16_t currentAngle = readRegister(REG_ANGLECOM) & 0x3FFF;
    // 5. Write zero position registers
    uint16_t zposm = (currentAngle >> 6) & 0x00FF;
    uint16_t zposl = currentAngle & 0x003F;
    writeRegister(REG_ZPOSM, zposm);
    writeRegister(REG_ZPOSL, zposl);
    // 6. Read back volatile registers 0x0016-0x001A (ZPOSM, ZPOSL, SETTINGS1-3)
    uint16_t regsVolatile[5];
    for(uint16_t addr = 0x0016; addr <= 0x001A; ++addr) {
        regsVolatile[addr - 0x0016] = readRegister(addr);
    }
    // 7. Enable ECC (ECC_en = 1 in ECC register 0x001B)
    uint16_t eccReg = readRegister(REG_ECC);
    eccReg |= 0x0080;
    writeRegister(REG_ECC, eccReg);
    // 8. Read calculated ECC checksum from ECCCHK register
    uint16_t eccKey = readRegister(REG_ECCCHK) & 0x007F;
    // 9. Write ECC checksum key into ECC register (preserving ECC_en)
    eccReg = 0x0080 | (eccKey & 0x007F);
    writeRegister(REG_ECC, eccReg);
    // 10. Read registers 0x0016-0x001B (including ECC) for verification
    uint16_t regsRead1[6];
    for(uint16_t addr = 0x0016; addr <= 0x001B; ++addr) {
        regsRead1[addr - 0x0016] = readRegister(addr);
    }
    // 11. Compare intended vs read values
    bool match = true;
    for(int i = 0; i < 5; ++i) {
        if(regsRead1[i] != regsVolatile[i]) {
            match = false;
            break;
        }
    }
    if(regsRead1[5] != eccReg) match = false;
    if(!match) {
        frameFormat = prevFormat;
        return false;
    }
    // 12. Enable OTP programming mode (PROGEN=1)
    writeRegister(REG_PROG, PROG_BIT_PROGEN);
    // 13. Start OTP burn (PROGOTP=1 while PROGEN=1)
    writeRegister(REG_PROG, PROG_BIT_PROGEN | PROG_BIT_PROGOTP);
    // 14. Wait until PROG register reads 0x0001 (programming complete)
    bool progDone = false;
    for(int attempts = 0; attempts < 10000; ++attempts) {
        uint16_t progVal = readRegister(REG_PROG);
        if(progVal == 0x0001) {
            progDone = true;
            break;
        }
        // (A short delay could be inserted here on real hardware)
    }
    if(!progDone) {
        frameFormat = prevFormat;
        return false;
    }
    // 15. Clear volatile registers 0x0015-0x001B (write 0x0000 to each)
    for(uint16_t addr = 0x0015; addr <= 0x001B; ++addr) {
        writeRegister(addr, 0x0000);
    }
    // 16. Set PROGVER=1 (enable guard-band test mode)
    writeRegister(REG_PROG, PROG_BIT_PROGEN | PROG_BIT_PROGVER);
    // 17. Refresh registers from OTP (OTPREF=1)
    writeRegister(REG_PROG, PROG_BIT_PROGEN | PROG_BIT_PROGVER | PROG_BIT_OTPREF);
    // 18. Read registers 0x0016-0x001B (post-refresh)
    uint16_t regsRead2[6];
    for(uint16_t addr = 0x0016; addr <= 0x001B; ++addr) {
        regsRead2[addr - 0x0016] = readRegister(addr);
    }
    // 19. Compare with original intended settings
    bool success = true;
    for(int i = 0; i < 6; ++i) {
        if(regsRead2[i] != regsRead1[i]) {
            success = false;
            break;
        }
    }
    // (Steps 20-23: optional power-cycle verification not automated here)
    // Restore previous SPI frame format
    frameFormat = prevFormat;
    return success;
}