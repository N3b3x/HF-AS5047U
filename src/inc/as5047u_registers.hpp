/**
 * @file as5047u_registers.hpp
 * @brief Strong‑typed register/bit‑field definitions for AMS AS5047U.
 *
 * Each struct wraps one 16‑bit register.  Usage pattern:
 *
 * ```cpp
 * AS5047U::Settings2 s2(bus);        // read current value from sensor
 * s2.bits.DIR = 1;                   // change one field
 * s2.bits.PWMon = 1;
 * s2.commit(bus);                    // write it back
 * ```
 *
 * All addresses follow the official datasheet (v1‑01, 2020‑12‑16).
 */

#pragma once
#include <cstdint>
#include "AS5047U_driver.hpp"   // needs spiBus + readRegister/writeRegister

namespace AS5047U_REG
{
/* ---------- common helpers ------------------------------------------------ */

using U16 = uint16_t;
template<U16 ADDR> struct RwReg   // readable / writeable volatile register
{
    static constexpr U16 address = ADDR;

    spiBus &bus;
    union { U16 value; struct Bits; } reg;

    explicit RwReg(spiBus &b) : bus{b} { value = AS5047U::rawRead(bus, address); }
    explicit RwReg(spiBus &b, U16 raw) : bus{b} { value = raw; }

    /** Write modified value back to the device */
    void commit() { AS5047U::rawWrite(bus, address, value); }
};

/* ---------- ERRFL – Error Flags (0x0001) ---------------------------------- */

struct ERRFL : RwReg<0x0001>
{
    using RwReg::RwReg;
    struct Bits
    {
        U16 AGC_warning         : 1;   ///< bit0
        U16 MagHalf             : 1;   ///< bit1
        U16 P2ram_warning       : 1;   ///< bit2 (single‑bit corrected)
        U16 P2ram_error         : 1;   ///< bit3 (double‑bit – uncorrectable)
        U16 Framing_error       : 1;   ///< bit4
        U16 Command_error       : 1;   ///< bit5
        U16 CRC_error           : 1;   ///< bit6
        U16 WDTST               : 1;   ///< bit7 watchdog / osc error
        U16 _reserved8          : 1;
        U16 OffCompNotFinished  : 1;   ///< bit9
        U16 CORDIC_overflow     : 1;   ///< bit10
        U16 _reserved11_15      : 5;
    } bits;
};

/* ---------- DISABLE (0x0015) ---------------------------------------------- */

struct DISABLE : RwReg<0x0015>
{
    using RwReg::RwReg;
    struct Bits
    {
        U16 UVW_off         : 1;  ///< 1 = tri‑state U/V/W
        U16 ABI_off         : 1;  ///< 1 = tri‑state A/B/I
        U16 _reserved2_5    : 4;
        U16 FILTER_disable  : 1;  ///< 1 = disable adaptive filter
        U16 _reserved7_15   : 9;
    } bits;
};

/* ---------- SETTINGS1 (0x0018) -------------------------------------------- */
struct SETTINGS1 : RwReg<0x0018>
{
    using RwReg::RwReg;
    enum class KCoef : uint8_t { K0, K1, K2, K3, K4, K5, K6, K7 };

    struct Bits
    {
        U16 K_max : 3;          ///< [2:0]
        U16 K_min : 3;          ///< [5:3]
        U16 _reserved6_15 :10;
    } bits;

    void setK(KCoef kMin, KCoef kMax)
    {
        bits.K_min = static_cast<U16>(kMin);
        bits.K_max = static_cast<U16>(kMax);
    }
};

/* ---------- SETTINGS2 (0x0019) -------------------------------------------- */
struct SETTINGS2 : RwReg<0x0019>
{
    using RwReg::RwReg;

    struct Bits
    {
        U16 IWIDTH       : 1;   ///< 0 = 3 LSB index, 1 = 1 LSB
        U16 NOISESET     : 1;   ///< 0 = ONL (low‑noise), 1 = ONH
        U16 DIR          : 1;   ///< 0 = CW increases ANGLE, 1 = CCW
        U16 UVW_ABI      : 1;   ///< 0 = ABI+PWM(W)  1 = UVW+PWM(I)
        U16 DAECDIS      : 1;   ///< 1 = disable dynamic angle compensation
        U16 ABI_DEC      : 1;   ///< decimal count (ABI)
        U16 Data_select  : 1;   ///< 0 = ANGLECOM on 0x3FFF, 1 = ANGLEUNC
        U16 PWMon        : 1;   ///< enable PWM output
        U16 _reserved8_15: 8;
    } bits;
};

/* ---------- SETTINGS3 (0x001A) -------------------------------------------- */
struct SETTINGS3 : RwReg<0x001A>
{
    using RwReg::RwReg;

    enum class PolePairs : uint8_t { P1, P2, P3, P4, P5, P6, P7 };
    enum class Hysteresis : uint8_t { LSB1=0, LSB2, LSB3, OFF };
    enum class ABIRes : uint8_t   { RES4096=0, RES2048, RES1024, RES512,
                                    RES256,  RES1000=8, RES500,  RES400,
                                    RES300,  RES200,    RES100,  RES50, RES25 };

    struct Bits
    {
        U16 UVWPP : 3;      ///< pole‑pair setting
        U16 HYS   : 2;      ///< hysteresis (datasheet fig.55)
        U16 ABIRES: 3;      ///< ABI resolution code
        U16 _reserved8_15 :8;
    } bits;
};

/* ---------- ZPOSM/L (0x0016 / 0x0017) ------------------------------------- */
struct ZPOSM : RwReg<0x0016> { using RwReg::RwReg; struct Bits{ U16 MSB8 :8; U16 _r:8;} bits; };
struct ZPOSL : RwReg<0x0017>
{
    using RwReg::RwReg;
    struct Bits
    {
        U16 LSB6      : 6;
        U16 DIA1_en   : 1;   ///< automotive versions only
        U16 DIA2_en   : 1;   ///< automotive versions only
        U16 _reserved : 8;
    } bits;
};

/* ---------- ECC (0x001B) -------------------------------------------------- */
struct ECC : RwReg<0x001B>
{
    using RwReg::RwReg;
    struct Bits
    {
        U16 ECC_chsum :7;
        U16 ECC_en    :1;
        U16 _res      :8;
    } bits;
};

/* ---------- PROG (0x0003) ------------------------------------------------- */
struct PROG : RwReg<0x0003>
{
    using RwReg::RwReg;
    struct Bits
    {
        U16 PROGEN   :1;
        U16 _r1      :1;
        U16 OTPREF   :1;
        U16 PROGOTP  :1;
        U16 _r4_5    :2;
        U16 PROGVER  :1;
        U16 _r7_15   :9;
    } bits;
};

} // namespace AS5047U_REG
  