/**
 * @file AS5047U_REGISTERS.hpp
 * @brief Register definitions for the AS5047U Rotary Position Sensor.
 *
 * @details This file provides structured register definitions for the AS5047U magnetic position
 * sensor, including both volatile (runtime) registers and non-volatile (OTP) registers. Each
 * register is defined as a struct with its address, full register value access, and bit-field
 * access through a union.
 *
 * The AS5047U is a 14-bit on-axis magnetic rotary position sensor with incremental (ABI) and
 * commutation (UVW) interfaces. It features field-programmable OTP memory for custom configuration
 * and dynamic angle error compensation (DAEC).
 *
 * Key features accessible through these registers:
 * - 14-bit absolute position measurement (0-16383 counts per revolution)
 * - Configurable incremental encoder output (ABI interface)
 * - Configurable commutation signals output (UVW interface)
 * - Dynamic angle error compensation
 * - Configurable PWM output
 * - Programmable zero position
 * - Error flags and diagnostics
 *
 * The register definitions include both the raw register addresses and structured bit fields
 * for easier programmatic access.
 *
 * @author Nebiyu Tadesse
 * @copyright Copyright (c) 2025 Nebiyu Tadesse
 * @license GNU GPL v3 or later
 */

#pragma once
#include <cstdint>

namespace AS5047U_REG {
///////////////////////////////////////////////////////////////////////////////
//                                                                           //
//   ██╗   ██╗ ██████╗ ██╗      █████╗ ████████╗██╗██╗     ███████╗          //
//   ██║   ██║██╔═══██╗██║     ██╔══██╗╚══██╔══╝██║██║     ██╔════╝          //
//   ██║   ██║██║   ██║██║     ███████║   ██║   ██║██║     █████╗            //
//   ╚██╗ ██╔╝██║   ██║██║     ██╔══██║   ██║   ██║██║     ██╔══╝            //
//    ╚████╔╝ ╚██████╔╝███████╗██║  ██║   ██║   ██║███████╗███████╗          //
//     ╚═══╝   ╚═════╝ ╚══════╝╚═╝  ╚═╝   ╚═╝   ╚═╝╚══════╝╚══════╝          //
//                                                                           //
//   ██████╗ ███████╗ ██████╗ ██╗███████╗████████╗███████╗██████╗ ███████╗   //
//   ██╔══██╗██╔════╝██╔════╝ ██║██╔════╝╚══██╔══╝██╔════╝██╔══██╗██╔════╝   //
//   ██████╔╝█████╗  ██║  ███╗██║███████╗   ██║   █████╗  ██████╔╝███████╗   //
//   ██╔══██╗██╔══╝  ██║   ██║██║╚════██║   ██║   ██╔══╝  ██╔══██╗╚════██║   //
//   ██║  ██║███████╗╚██████╔╝██║███████║   ██║   ███████╗██║  ██║███████║   //
//   ╚═╝  ╚═╝╚══════╝ ╚═════╝ ╚═╝╚══════╝   ╚═╝   ╚══════╝╚═╝  ╚═╝╚══════╝   //
//                                                                           //
//===========================================================================//
//                          VOLATILE REGISTERS                               //
//===========================================================================//
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief NOP – No Operation register (0x0000, default 0x0000)
 *
 * Address | Name | Default | Description
 * ------- | ---- | ------- | --------------
 * 0x0000  | NOP  | 0x0000  | No operation
 *
 * @details The NOP register is used to perform a no-operation command. Reading or writing to this
 * register has no effect.
 */
struct NOP {
    static constexpr uint16_t ADDRESS = 0x0000;
    union {
        uint16_t value;
        struct {
            uint16_t reserved : 16; // No specific bits defined for NOP
        } bits;
    };
};

/**
 * @brief ERRFL – Error Flag register (0x0001, default 0x0000)
 *
 * Bits | Name                | R  | Description
 * -----|---------------------|----|----------------------------------------------
 * 0    | AGC_warning         | R  | AGC at limit flag (1 = AGC value reached 0 or 255,
 *      |                     |    | indicating extreme magnetic field)
 * 1    | MagHalf             | R  | Magnetic field half error (1 = AGC=255 and measured
 *      |                     |    | magnitude ~ half of target)
 * 2    | P2RAM_warning       | R  | OTP shadow (P2RAM) single-bit correction flag (ECC
 *      |                     |    | corrected one bit)
 * 3    | P2RAM_error         | R  | OTP shadow (P2RAM) double-bit error flag (uncorrectable
 *      |                     |    | ECC error)
 * 4    | Framing_error       | R  | SPI framing error (bad SPI frame alignment)
 * 5    | Command_error       | R  | SPI command error (invalid SPI command received)
 * 6    | CRC_error           | R  | SPI CRC communication error (CRC checksum mismatch)
 * 7    | WDTST               | R  | Watchdog failure (1 = internal oscillator or watchdog
 *      |                     |    | test failed)
 * 8    | (unused)            | -  | Unused bit (always 0)
 * 9    | OffCompNotFinished  | R  | Offset compensation not finished (1 = internal offset
 *      |                     |    | calibration still in progress)
 * 10   | CORDIC_Overflow     | R  | CORDIC overflow error flag (numerical overflow in
 *      |                     |    | CORDIC cord. calc)
 * 11-15| (reserved)          | -  | Reserved (reads 0)
 *
 * @note Reading the ERRFL register clears all the error flags (self-clearing on read). In case any
 * error flag is set, it is recommended to read the DIA (Diagnostic) register for more detailed
 * status.
 */
struct ERRFL {
    static constexpr uint16_t ADDRESS = 0x0001;
    union {
        uint16_t value;
        struct {
            uint16_t AGC_warning : 1;
            uint16_t MagHalf : 1;
            uint16_t P2RAM_warning : 1;
            uint16_t P2RAM_error : 1;
            uint16_t Framing_error : 1;
            uint16_t Command_error : 1;
            uint16_t CRC_error : 1;
            uint16_t WDTST : 1;
            uint16_t unused_8 : 1;
            uint16_t OffCompNotFinished : 1;
            uint16_t CORDIC_Overflow : 1;
            uint16_t reserved_11_15 : 5;
        } bits;
    };
};

/**
 * @brief PROG – OTP programming control register (0x0003, default 0x0000)
 *
 * Bits | Name      | R/W | Description
 * -----|-----------|-----|----------------------------------------------
 * 0    | PROGEN    | R/W | OTP program enable (0 = Normal operation, 1 = Enable OTP
 *      |           |     | read/write access)
 * 1    | (reserved)| -   | Reserved (must be 0)
 * 2    | OTPREF    | R/W | OTP refresh trigger (1 = refresh shadow registers with OTP
 *      |           |     | content)
 * 3    | PROGOTP   | R/W | Start OTP programming cycle (set 1 to begin burning OTP fuses)
 * 4    | (reserved)| -   | Reserved (must be 0)
 * 5    | (reserved)| -   | Reserved (must be 0)
 * 6    | PROGVER   | R/W | Program verify mode enable (1 = enable guard-band verification mode)
 * 7-15 | (reserved)| -   | Reserved (must be 0)
 *
 * @details Writing this register controls the OTP programming sequence:
 * - Set **PROGEN=1** to enable OTP programming mode (allow OTP registers 0x0015–0x001B to be
 *   written).
 * - Then set **PROGOTP=1** to initiate the OTP burn procedure. The PROGOTP bit will self-clear when
 *   the programming cycle completes.
 * - The **PROGVER** bit can be set to 1 to enter verification mode (guard band test) after
 *   programming.
 * - Toggling **OTPREF** (e.g., set to 1) reloads the non-volatile data from OTP into the device's
 *   shadow registers (useful after programming or to revert soft writes).
 *
 * After starting a programming cycle, the host should poll the PROG register until it reads 0x0001,
 * indicating the procedure is complete (PROGEN remains set, other bits cleared). It is recommended
 * to perform verification and a power cycle (with PROGVER and OTPREF as described in the datasheet)
 * to ensure OTP programming success.
 */
struct PROG {
    static constexpr uint16_t ADDRESS = 0x0003;
    union {
        uint16_t value;
        struct {
            uint16_t PROGEN : 1;
            uint16_t reserved_1 : 1;
            uint16_t OTPREF : 1;
            uint16_t PROGOTP : 1;
            uint16_t reserved_4 : 1;
            uint16_t reserved_5 : 1;
            uint16_t PROGVER : 1;
            uint16_t reserved_7 : 1;
            uint16_t reserved_8_15 : 8;
        } bits;
    };
};

/**
 * @brief DIA – Diagnostic register (0x3FF5, read-only)
 *
 * Bits | Name               | R | Description
 * -----|--------------------|---|----------------------------------------------
 * 0    | VDD_mode           | R | Supply voltage mode (0 = 3.3V operation mode,
 *      |                    |   | 1 = 5V operation mode)
 * 1    | LoopsFinished      | R | Magneto-mechanical loops finished (1 = all internal
 *      |                    |   | startup loops complete)
 * 2    | CORDIC_overflow_flag| R| CORDIC overflow error flag (1 = CORDIC arithmetic
 *      |                    |   | overflow occurred)
 * 3    | Comp_l             | R | AGC low *warning* flag (1 = magnetic field *strong* –
 *      |                    |   | AGC near 0)
 * 4    | Comp_h             | R | AGC high *warning* flag (1 = magnetic field *weak* –
 *      |                    |   | AGC near 255)
 * 5    | MagHalf_flag       | R | Magnitude half-range error flag (1 = measured MAG below
 *      |                    |   | half of target)
 * 6    | CosOff_fin         | R | Cosine offset compensation finished (1 = cosine offset
 *      |                    |   | calib complete)
 * 7    | SinOff_fin         | R | Sine offset compensation finished (1 = sine offset
 *      |                    |   | calib complete)
 * 8    | OffComp_finished   | R | Offset compensation finished flag (0 = still running,
 *      |                    |   | 1 = completed)
 * 9    | AGC_finished       | R | AGC initialization finished (1 = initial gain calibration
 *      |                    |   | complete)
 * 10   | (unused)           | - | Unused bit (always 0)
 * 11-12| SPI_cnt            | R | SPI frame counter (2-bit rolling count of consecutive
 *      |                    |   | SPI frames)
 * 13-15| (reserved)         | - | Reserved (reads 0)
 */
struct DIA {
    static constexpr uint16_t ADDRESS = 0x3FF5;
    union {
        uint16_t value;
        struct {
            uint16_t VDD_mode : 1;
            uint16_t LoopsFinished : 1;
            uint16_t CORDIC_overflow_flag : 1;
            uint16_t Comp_l : 1;
            uint16_t Comp_h : 1;
            uint16_t MagHalf_flag : 1;
            uint16_t CosOff_fin : 1;
            uint16_t SinOff_fin : 1;
            uint16_t OffComp_finished : 1;
            uint16_t AGC_finished : 1;
            uint16_t unused_10 : 1;
            uint16_t SPI_cnt : 2;
            uint16_t reserved_13_15 : 3;
        } bits;
    };
};

/**
 * @brief AGC – Automatic Gain Control register (0x3FF9, read-only)
 *
 * Bits | Name       | R | Description
 * ---- | ---------- | - | -----------------------------------------------
 * 0-7  | AGC_value  | R | 8-bit AGC value (dynamic gain level, 0x00=minimum gain, 0xFF=maximum
 * gain) 8-15 | (reserved) | - | Reserved (reads 0)
 *
 * @details The AGC_value represents the current automatic gain control setting of the sensor’s
 * analog front-end. A low value (near 0) means the magnetic field is strong (gain reduced), whereas
 * a high value (near 255) means the field is weak (gain increased). This value is continuously
 * adjusted to keep the CORDIC magnitude stable.
 */
struct AGC {
    static constexpr uint16_t ADDRESS = 0x3FF9;
    union {
        uint16_t value;
        struct {
            uint16_t AGC_value : 8;
            uint16_t reserved_8_15 : 8;
        } bits;
    };
};

/**
 * @brief SINDATA – Raw digital sine channel data (0x3FFA, default 0x0000)
 *
 * Bits | Name     | R | Description
 * ---- | -------- | - | ---------------------------------------------
 * 0-15 | SINDATA  | R | Raw digital sine channel data (16-bit signed)
 *
 * @details The SINDATA register provides the raw digital output of the sine channel from the
 * sensor’s CORDIC engine. The value is a signed 16-bit integer.
 */
struct SINDATA {
    static constexpr uint16_t ADDRESS = 0x3FFA;
    union {
        uint16_t value;
        struct {
            int16_t SINDATA : 16; // Raw signed sine data
        } bits;
    };
};

/**
 * @brief COSDATA – Raw digital cosine channel data (0x3FFB, default 0x0000)
 *
 * Bits | Name     | R | Description
 * ---- | -------- | - | ---------------------------------------------
 * 0-15 | COSDATA  | R | Raw digital cosine channel data (16-bit signed)
 *
 * @details The COSDATA register provides the raw digital output of the cosine channel from the
 * sensor’s CORDIC engine. The value is a signed 16-bit integer.
 */
struct COSDATA {
    static constexpr uint16_t ADDRESS = 0x3FFB;
    union {
        uint16_t value;
        struct {
            int16_t COSDATA : 16; // Raw signed cosine data
        } bits;
    };
};

/**
 * @brief VEL – Velocity register (0x3FFC, read-only)
 *
 * Bits | Name       | R | Description
 * ---- | ---------- | - | --------------------------------------------------------
 * 0-13 | VEL_value  | R | Measured velocity (14-bit two’s complement value)
 * 14-15| (reserved) | - | Reserved bits (reads 0)
 *
 * @details VEL_value is a signed 14-bit integer representing the rotational velocity. The value is
 * given in two’s complement format. A positive value indicates rotation in the default direction
 * (DIR=0 defined direction), while a negative value indicates rotation in the opposite direction.
 * The scale (LSB unit) is proportional to speed, but the exact units depend on the sensor
 * configuration and sampling rate (e.g., LSB per measurement interval).
 */
struct VEL {
    static constexpr uint16_t ADDRESS = 0x3FFC;
    union {
        uint16_t value;
        struct {
            uint16_t VEL_value : 14;
            uint16_t reserved_14_15 : 2;
        } bits;
    };
};

/**
 * @brief MAG – CORDIC Magnitude register (0x3FFD, read-only)
 *
 * Bits | Name       | R | Description
 * ---- | ---------- | - | -----------------------------------------------
 * 0-13 | MAG_value  | R | CORDIC magnitude (14-bit value of magnetic field strength)
 * 14-15| (reserved) | - | Reserved bits (reads 0)
 *
 * @details The MAG_value represents the magnitude of the vector computed by the CORDIC (Coordinate
 * Rotation Digital Computer), corresponding to the strength of the magnetic field as seen by the
 * sensor. Under normal conditions with a properly placed magnet, this value stays near a target
 * level (e.g., ~0x0FFF). A significantly lower value (together with the MagHalf_flag in DIA or
 * MagHalf in ERRFL) indicates the field is about half of the expected strength.
 */
struct MAG {
    static constexpr uint16_t ADDRESS = 0x3FFD;
    union {
        uint16_t value;
        struct {
            uint16_t MAG_value : 14;
            uint16_t reserved_14_15 : 2;
        } bits;
    };
};

/**
 * @brief ANGLEUNC – Uncompensated Angle register (0x3FFE, read-only)
 *
 * Bits | Name          | R | Description
 * ---- | ------------- | - | ----------------------------------------------
 * 0-13 | ANGLEUNC_value| R | Absolute angle without dynamic compensation (14-bit)
 * 14-15| (reserved)    | - | Reserved bits (reads 0)
 *
 * @details ANGLEUNC_value is the raw angle measurement from the Hall sensor array and CORDIC, not
 * including the Dynamic Angle Error Compensation (DAEC) filtering. The value is 14-bit (0–16383)
 * representing 0–360° (with 0x0000 corresponding to 0° and 0x3FFF (~16383) corresponding to just
 * shy of 360°). Use this register if the dynamically compensated angle (ANGLECOM) is not desired.
 */
struct ANGLEUNC {
    static constexpr uint16_t ADDRESS = 0x3FFE;
    union {
        uint16_t value;
        struct {
            uint16_t ANGLEUNC_value : 14;
            uint16_t reserved_14_15 : 2;
        } bits;
    };
};

/**
 * @brief ECC_Checksum – ECC checksum register (0x3FD0, default 0x0000)
 *
 * Bits | Name   | R/W | Description
 * ---- | ------ | --- | -------------------------------------------------------------------------
 * 6:0  | ECC_s  | R   | Calculated ECC checksum based on actual register setting
 *
 * @details The ECC_Checksum register provides a 7-bit ECC checksum value calculated based on the
 * current register settings.
 */
struct ECC_Checksum {
    static constexpr uint16_t ADDRESS = 0x3FD0;
    union {
        uint16_t value;
        struct {
            uint16_t ECC_s : 7;         // Calculated ECC checksum
            uint16_t reserved_7_15 : 9; // Reserved bits
        } bits;
    };
};

/**
 * @brief ANGLECOM – Compensated Angle register (0x3FFF, read-only)
 *
 * Bits | Name          | R | Description
 * ---- | ------------- | - | ----------------------------------------------
 * 0-13 | ANGLECOM_value| R | Absolute angle with dynamic compensation (14-bit)
 * 14-15| (reserved)    | - | Reserved bits (reads 0)
 *
 * @details ANGLECOM_value is the fully compensated 14-bit absolute angle reading, incorporating the
 * sensor’s dynamic angle error compensation (DAEC) algorithm. This value is updated in real-time
 * with minimal latency at high speeds, thanks to DAEC. It is the primary angle output for most
 * applications. If the Data_select bit in SETTINGS2 is 0 (default), reading from address 0x3FFF
 * returns this ANGLECOM value. The angle is represented as a number from 0 to 16383 corresponding
 * to 0° to 360° (with 14-bit resolution).
 */
struct ANGLECOM {
    static constexpr uint16_t ADDRESS = 0x3FFF;
    union {
        uint16_t value;
        struct {
            uint16_t ANGLECOM_value : 14;
            uint16_t reserved_14_15 : 2;
        } bits;
    };
};

/////////////////////////////////////////////////////////////////////////////////                                                                            //
//   ███╗   ██╗ ██████╗ ███╗   ██╗                                             //
//   ████╗  ██║██╔═══██╗████╗  ██║                                             //
//   ██╔██╗ ██║██║   ██║██╔██╗ ██║                                             //
//   ██║╚██╗██║██║   ██║██║╚██╗██║                                             //
//   ██║ ╚████║╚██████╔╝██║ ╚████║                                             //
//   ╚═╝  ╚═══╝ ╚═════╝ ╚═╝  ╚═══╝                                             //
//                                                                             //
//   ██╗   ██╗ ██████╗ ██╗      █████╗ ████████╗██╗██╗     ███████╗            //
//   ██║   ██║██╔═══██╗██║     ██╔══██╗╚══██╔══╝██║██║     ██╔════╝            //
//   ██║   ██║██║   ██║██║     ███████║   ██║   ██║██║     █████╗              //
//   ╚██╗ ██╔╝██║   ██║██║     ██╔══██║   ██║   ██║██║     ██╔══╝              //
//    ╚████╔╝ ╚██████╔╝███████╗██║  ██║   ██║   ██║███████╗███████╗            //
//     ╚═══╝   ╚═════╝ ╚══════╝╚═╝  ╚═╝   ╚═╝   ╚═╝╚══════╝╚══════╝            //
//                                                                             //
//   ██████╗ ███████╗ ██████╗ ██╗███████╗████████╗███████╗██████╗ ███████╗     //
//   ██╔══██╗██╔════╝██╔════╝ ██║██╔════╝╚══██╔══╝██╔════╝██╔══██╗██╔════╝     //
//   ██████╔╝█████╗  ██║  ███╗██║███████╗   ██║   █████╗  ██████╔╝███████╗     //
//   ██╔══██╗██╔══╝  ██║   ██║██║╚════██║   ██║   ██╔══╝  ██╔══██╗╚════██║     //
//   ██║  ██║███████╗╚██████╔╝██║███████║   ██║   ███████╗██║  ██║███████║     //
//   ╚═╝  ╚═╝╚══════╝ ╚═════╝ ╚═╝╚══════╝   ╚═╝   ╚══════╝╚═╝  ╚═╝╚══════╝     //
//                                                                             //
//   ██╗ ██████╗ ████████╗██████╗ ██╗                                          //
//   ██║██╔═══██╗╚══██╔══╝██╔══██╗██║                                          //
//   ██║██║   ██║   ██║   ██████╔╝██║                                          //
//   ╚═╝██║   ██║   ██║   ██╔═══╝ ╚═╝                                          //
//   ██╗╚██████╔╝   ██║   ██║     ██╗                                          //
//   ╚═╝ ╚═════╝    ╚═╝   ╚═╝     ╚═╝                                          //
//=============================================================================//
//                      NON-VOLATILE REGISTERS (OTP)                           //
//=============================================================================//
/////////////////////////////////////////////////////////////////////////////////

/**
 * @brief DISABLE – Outputs and filter disable register (0x0015, default 0x0000)
 *
 * Bits | Name          | R/W/P | Description
 * ---- | ------------- | ----- | ---------------------------------------------------------
 * 0    | UVW_off       | R/W/P | UVW outputs off (0 = Normal mode, 1 = UVW pins tristate)
 * 1    | ABI_off       | R/W/P | ABI outputs off (0 = Normal mode, 1 = ABI pins tristate)
 * 2-5  | (reserved)    | R/W/P | Reserved bits (must be 0)
 * 6    | FILTER_disable| R/W/P | Disable output filter (0 = Filter enabled (default), 1 = Filter
 *      |               |       | disabled)
 * 7    | (reserved)    | R/W/P | Reserved bit (must be 0)
 * 8-15 | (unused)      | -     | Unused upper bits (reads 0)
 */
struct DISABLE {
    static constexpr uint16_t ADDRESS = 0x0015;
    union {
        uint16_t value;
        struct {
            uint16_t UVW_off : 1;
            uint16_t ABI_off : 1;
            uint16_t reserved_2_5 : 4;
            uint16_t FILTER_disable : 1;
            uint16_t reserved_7 : 1;
            uint16_t reserved_8_15 : 8;
        } bits;
    };

    /// @brief UVW output mode (UVW_off bit)
    enum class UVWMode : uint8_t {
        Normal = 0,  /**< Normal operation mode (UVW outputs active) */
        Tristate = 1 /**< High-impedance mode (UVW pins tristated) */
    };

    /// @brief ABI output mode (ABI_off bit)
    enum class ABIMode : uint8_t {
        Normal = 0,  /**< Normal operation mode (ABI outputs active) */
        Tristate = 1 /**< High-impedance mode (ABI pins tristated) */
    };

    /// @brief Output filter control (FILTER_disable bit)
    enum class FilterMode : uint8_t {
        Enabled = 0, /**< Digital filter enabled (default) */
        Disabled = 1 /**< Digital filter disabled (faster response, more noise) */
    };
};

/**
 * @brief ZPOSM – Zero Position MSB register (0x0016, default 0x0000)
 *
 * Bits  | Name    | R/W/P | Description
 * ------|---------|-------|-----------------------------------------------
 * 0-7   | ZPOSM   | R/W/P | Most significant 8 bits of the zero position
 * 8-15  | (unused)| -     | Unused upper bits (reads 0)
 *
 * @note Together with ZPOSL, forms a 14-bit zero position value.
 */
struct ZPOSM {
    static constexpr uint16_t ADDRESS = 0x0016;
    union {
        uint16_t value;
        struct {
            uint16_t ZPOSM_bits : 8;
            uint16_t reserved_8_15 : 8;
        } bits;
    };
};

/**
 * @brief ZPOSL – Zero Position LSB register (0x0017, default 0x0000)
 *
 * Bits  | Name    | R/W/P | Description
 * ------|---------|-------|-----------------------------------------------
 * 0-5   | ZPOSL   | R/W/P | Least significant 6 bits of the zero position
 * 6     | Dia1_en | R/W/P | Diagnostic feature 1 (Default 0; for AS5147U automotive variant only)
 * 7     | Dia2_en | R/W/P | Diagnostic feature 2 (Default 0; for AS5147U automotive variant only)
 * 8-15  | (unused)| -     | Unused upper bits (reads 0)
 *
 * @note Bits 6 and 7 are not applicable on the AS5047U (remain 0 unless using AS5147U).
 */
struct ZPOSL {
    static constexpr uint16_t ADDRESS = 0x0017;
    union {
        uint16_t value;
        struct {
            uint16_t ZPOSL_bits : 6;
            uint16_t Dia1_en : 1;
            uint16_t Dia2_en : 1;
            uint16_t reserved_8_15 : 8;
        } bits;
    };
};

/**
 * @brief SETTINGS1 – Custom setting register 1 (0x0018, default 0x0000)
 *
 * Bits  | Name    | R/W/P | Description
 * ------|---------|-------|--------------------------------------------------------------
 * 0-2   | K_max   | R/W/P | K_max coefficient for adaptive filter (default 0x0)
 * 3-5   | K_min   | R/W/P | K_min coefficient for adaptive filter (default 0x0)
 * 6     | Dia3_en | R/W/P | Diagnostic feature 3 (Default 0; not applicable for AS5047U)
 * 7     | Dia4_en | R/W/P | Diagnostic feature 4 (Default 0; not applicable for AS5047U)
 * 8-15  | (unused)| -     | Unused upper bits (reads 0)
 *
 * @details K_max and K_min define the adaptive filter's dynamic range. Higher K values increase
 * filter bandwidth (more noise, less filtering). Both default to 0 (minimum filter setting).
 * Dia3_en and Dia4_en are reserved (used only in the AS5147U variant).
 */
struct SETTINGS1 {
    static constexpr uint16_t ADDRESS = 0x0018;
    union {
        uint16_t value;
        struct {
            uint16_t K_max : 3;
            uint16_t K_min : 3;
            uint16_t Dia3_en : 1;
            uint16_t Dia4_en : 1;
            uint16_t reserved_8_15 : 8;
        } bits;
    };
    /// @brief Enumerated options for K_max field (adaptive filter max coefficient)
    enum class AdaptiveFilterKmax : uint8_t {
        Code0 = 0b000, /**< K_max actual value = 6 (default) */
        Code1 = 0b001, /**< K_max actual value = 5 */
        Code2 = 0b010, /**< K_max actual value = 4 */
        Code3 = 0b011, /**< K_max actual value = 3 */
        Code4 = 0b100, /**< K_max actual value = 5 (same as Code1) */
        Code5 = 0b101, /**< K_max actual value = 1 */
        Code6 = 0b110, /**< K_max actual value = 0 */
        Code7 = 0b111  /**< K_max actual value = 0 (same as Code6) */
    };
    /// @brief Enumerated options for K_min field (adaptive filter min coefficient)
    enum class AdaptiveFilterKmin : uint8_t {
        Code0 = 0b000, /**< K_min actual value = 2 (default) */
        Code1 = 0b001, /**< K_min actual value = 3 */
        Code2 = 0b010, /**< K_min actual value = 4 */
        Code3 = 0b011, /**< K_min actual value = 5 */
        Code4 = 0b100, /**< K_min actual value = 6 */
        Code5 = 0b101, /**< K_min actual value = 0 */
        Code6 = 0b110, /**< K_min actual value = 1 */
        Code7 = 0b111  /**< K_min actual value = 1 (same as Code6) */
    };
};

/**
 * @brief SETTINGS2 – Custom setting register 2 (0x0019, default 0x0000)
 *
 * Bits  | Name        | R/W/P | Description
 * ------|-------------|-------|--------------------------------------------------------------
 * 0     | IWIDTH      | R/W/P | Index pulse width (0: 3 LSB pulses (default), 1: 1 LSB pulse)
 * 1     | NOISESET    | R/W/P | Noise setting for 3.3V operation at 150°C (0: Normal noise,
 *       |             |       | 1: Reduced noise)
 * 2     | DIR         | R/W/P | Rotation direction setting (0: CW default, 1: Invert direction)
 * 3     | UVW_ABI     | R/W/P | Select ABI or UVW interface for outputs and PWM (0: ABI active,
 *       |             |       | PWM on W; 1: UVW active, PWM on I)
 * 4     | DAECDIS     | R/W/P | Dynamic angle error compensation disable (0: DAEC on (default),
 *       |             |       | 1: DAEC off)
 * 5     | ABI_DEC     | R/W/P | ABI decimal count enable (0: Binary resolution, 1: Decimal pulses
 *       |             |       | per revolution)
 * 6     | Data_select | R/W/P | Selects angle data at 0x3FFF (0: ANGLECOM, 1: ANGLEUNC readout)
 * 7     | PWMon       | R/W/P | PWM output enable (0: PWM disabled, 1: PWM enabled)
 * 8-15  | (unused)    | -     | Unused upper bits (reads 0)
 *
 * @details This register configures various interface options. IWIDTH sets the index pulse width in
 * the ABI interface. DIR inverts the direction of the incremental signals (defines rotation
 * direction as clockwise when DIR=0). UVW_ABI selects which incremental interface is active and
 * which pin outputs the PWM signal. Setting DAECDIS=1 disables dynamic angle error compensation
 * (the angle outputs will be raw). ABI_DEC=1 selects a decimal pulse count per revolution (e.g.,
 * 1000 ppr) instead of binary powers of two. Data_select controls whether the 0x3FFF address
 * returns the compensated or uncompensated angle. PWM output must be enabled (PWMon=1) and mapped
 * via UVW_ABI.
 */
struct SETTINGS2 {
    static constexpr uint16_t ADDRESS = 0x0019;
    union {
        uint16_t value;
        struct {
            uint16_t IWIDTH : 1;
            uint16_t NOISESET : 1;
            uint16_t DIR : 1;
            uint16_t UVW_ABI : 1;
            uint16_t DAECDIS : 1;
            uint16_t ABI_DEC : 1;
            uint16_t Data_select : 1;
            uint16_t PWMon : 1;
            uint16_t reserved_8_15 : 8;
        } bits;
    };
    /// @brief Index pulse width options (IWIDTH bit)
    enum class IndexWidth : uint8_t {
        Width3LSB = 0, /**< 3 LSB long index pulse (default) */
        Width1LSB = 1  /**< 1 LSB long index pulse */
    };
    /// @brief High-temperature noise reduction setting (NOISESET bit)
    enum class NoiseSetting : uint8_t {
        Normal = 0,  /**< Normal noise setting (for typical conditions) */
        HighTemp = 1 /**< Reduced noise for 3.3V operation at ~150°C (use at high temp) */
    };
    /// @brief Rotation direction setting (DIR bit) – defines incremental output phase sense
    enum class RotationDirection : uint8_t {
        Normal = 0,  /**< Default rotation direction (CW defined as DIR=0) */
        Inverted = 1 /**< Inverted rotation direction (swap A/B phasing) */
    };
    /// @brief Output interface selection for ABI vs UVW and PWM mapping (UVW_ABI bit)
    enum class OutputInterfaceMode : uint8_t {
        ABI_PWM_on_W = 0, /**< ABI signals active (A,B,I); PWM output on W pin */
        UVW_PWM_on_I = 1  /**< UVW signals active; PWM output on I (index) pin */
    };
    /// @brief Dynamic angle error compensation (DAEC) mode (DAECDIS bit)
    enum class DynamicCompensation : uint8_t {
        Enabled = 0, /**< DAEC enabled (dynamic compensation ON, default) */
        Disabled = 1 /**< DAEC disabled (output raw angle without comp.) */
    };
    /// @brief ABI resolution counting mode (ABI_DEC bit) – binary vs decimal pulses per revolution
    enum class ABICountMode : uint8_t {
        Binary = 0, /**< Binary resolution (2^N steps per revolution) */
        Decimal = 1 /**< Decimal resolution (e.g., 1000 pulses per revolution if configured) */
    };
    /// @brief Angle data source for 0x3FFF output (Data_select bit)
    enum class AngleOutputSource : uint8_t {
        UseANGLECOM = 0, /**< 0x3FFF reads ANGLECOM (compensated angle) */
        UseANGLEUNC = 1  /**< 0x3FFF reads ANGLEUNC (uncompensated angle) */
    };
    /// @brief PWM output enable (PWMon bit)
    enum class PWMMode : uint8_t {
        Disabled = 0, /**< PWM output disabled */
        Enabled = 1   /**< PWM output enabled */
    };
};

static_assert(sizeof(SETTINGS2) == 2, "SETTINGS2 must be 2 bytes");

/**
 * @brief SETTINGS3 – Custom setting register 3 (0x001A, default 0x0000)
 *
 * Bits  | Name    | R/W/P | Description
 * ------|---------|-------|--------------------------------------------------------------
 * 0-2   | UVWPP   | R/W/P | UVW commutation pole pair count (0=1pp up to 6=7pp, 7=7pp)
 * 3-4   | HYS     | R/W/P | Incremental output hysteresis setting (see table below)
 * 5-7   | ABIRES  | R/W/P | Resolution of ABI interface (incremental resolution code)
 * 8-15  | (unused)| -     | Unused upper bits (reads 0)
 *
 * Hysteresis (HYS) options (for incremental outputs):
 *   - 00: 1 LSB hysteresis (0.17°) – **default**
 *   - 01: 2 LSB hysteresis (0.35°)
 *   - 10: 3 LSB hysteresis (0.52°)
 *   - 11: 0 LSB (no hysteresis, outputs may toggle on slightest motion)
 *
 * The ABIRES field selects the incremental encoder resolution. Its interpretation depends on
 * ABI_DEC (binary or decimal mode). For binary mode (ABI_DEC=0):
 *   - code 0b000 = 12-bit (4096 steps/1024 pulses per rev, default)
 *   - 0b001 = 11-bit
 *   - 0b010 = 10-bit
 *   - 0b011 = 13-bit
 *   - 0b100 = 14-bit (16384 steps/4096 ppr, max)
 *   - Codes 0b101–0b111 are reserved in binary mode (treated as 14-bit max).
 * For decimal mode (ABI_DEC=1): codes map to decimal pulses:
 *   - 0b000 = 1000 ppr (4000 steps)
 *   - 0b001 = 500 ppr
 *   - 0b010 = 400 ppr
 *   - 0b011 = 300 ppr
 *   - 0b100 = 200 ppr
 *   - 0b101 = 100 ppr
 *   - 0b110 = 50 ppr
 *   - 0b111 = 25 ppr.
 */
struct SETTINGS3 {
    static constexpr uint16_t ADDRESS = 0x001A;
    union {
        uint16_t value;
        struct {
            uint16_t UVWPP : 3;
            uint16_t HYS : 2;
            uint16_t ABIRES : 3;
            uint16_t reserved_8_15 : 8;
        } bits;
    };
    /// @brief UVW pole pair count (UVWPP) options for BLDC commutation
    enum class UVWPolePairs : uint8_t {
        PP1 = 0b000,    /**< 1 pole pair (default) */
        PP2 = 0b001,    /**< 2 pole pairs */
        PP3 = 0b010,    /**< 3 pole pairs */
        PP4 = 0b011,    /**< 4 pole pairs */
        PP5 = 0b100,    /**< 5 pole pairs */
        PP6 = 0b101,    /**< 6 pole pairs */
        PP7 = 0b110,    /**< 7 pole pairs (also for code 0b111) */
        PP7_ALT = 0b111 /**< 7 pole pairs (alternate code, same as 0b110) */
    };
    /// @brief Incremental output hysteresis (HYS) settings:
    enum class Hysteresis : uint8_t {
        LSB_1 = 0b00, /**< 1 LSB hysteresis (default, ~0.17°) */
        LSB_2 = 0b01, /**< 2 LSB hysteresis (~0.35°) */
        LSB_3 = 0b10, /**< 3 LSB hysteresis (~0.52°) */
        NONE = 0b11   /**< 0 LSB hysteresis (no hysteresis) */
    };
    /// @brief ABI interface resolution (ABIRES) codes – see description for binary vs decimal mode
    /// interpretation
    enum class ABIResolution : uint8_t {
        Code0 =
            0b000, /**< Binary: 4096 steps (1024 ppr, 12-bit) ; Decimal: 4000 steps (1000 ppr) */
        Code1 = 0b001, /**< Binary: 2048 steps (512 ppr, 11-bit) ; Decimal: 2000 steps (500 ppr) */
        Code2 = 0b010, /**< Binary: 1024 steps (256 ppr, 10-bit) ; Decimal: 1600 steps (400 ppr) */
        Code3 = 0b011, /**< Binary: 8192 steps (2048 ppr, 13-bit) ; Decimal: 1200 steps (300 ppr) */
        Code4 =
            0b100, /**< Binary: 16384 steps (4096 ppr, 14-bit) ; Decimal: 800 steps  (200 ppr) */
        Code5 = 0b101, /**< (Reserved in binary mode)            ; Decimal: 400 steps (100 ppr) */
        Code6 = 0b110, /**< (Reserved in binary mode)            ; Decimal: 200 steps (50 ppr) */
        Code7 = 0b111  /**< (Reserved in binary mode)            ; Decimal: 100 steps (25 ppr) */
    };
};

static_assert(sizeof(SETTINGS3) == 2, "SETTINGS3 must be 2 bytes");

/**
 * @brief ECC – ECC Settings register (0x001B, default 0x0000)
 *
 * Bits  | Name      | R/W/P | Description
 * ------|-----------|-------|---------------------------------------------
 * 0-6   | ECC_chsum | R/W/P | ECC checksum bits (7-bit ECC for OTP content)
 * 7     | ECC_en    | R/W/P | Enable ECC protection (0 = disable, 1 = enable ECC)
 * 8-15  | (unused)  | -     | Unused upper bits (reads 0)
 *
 * @details This register holds the Error-Correcting Code (ECC) for the OTP registers. To protect
 * the custom settings, set ECC_en=1 and then program the ECC_chsum with the correct checksum before
 * burning OTP. The 7-bit ECC checksum must be computed (read from address 0x00D1/ECC_s mirror after
 * enabling ECC) and written here prior to final OTP programming.
 */
struct ECC {
    static constexpr uint16_t ADDRESS = 0x001B;
    union {
        uint16_t value;
        struct {
            uint16_t ECC_chsum : 7;
            uint16_t ECC_en : 1;
            uint16_t reserved_8_15 : 8;
        } bits;
    };

    /// @brief ECC enable/disable (ECC_en bit)
    enum class ECCMode : uint8_t {
        Disabled = 0, /**< ECC disabled (default) */
        Enabled = 1   /**< ECC enabled (protects OTP content) */
    };
};

} // namespace AS5047U_REG