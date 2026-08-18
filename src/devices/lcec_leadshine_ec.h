//
//    Copyright (C) 2026 linuxcnc-ethercat contributors
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

/// @file
/// @brief Driver for the Leadshine R2EC / R3EC EtherCAT bus couplers and
/// their modular I/O sub-devices (issue #434).
///
/// The R2EC/R3EC is a single EtherCAT slave that hosts up to 32 (R2EC) or
/// 64 (R3EC) modules on a passive backplane.  Each populated slot is
/// described in the LinuxCNC XML config as a `<subModule>` (see the generic
/// submodule support in lcec_conf.c / lcec_main.c):
///
///   <slave idx="0" type="R2EC" name="io">
///     <subModule id="0" ident="0x61100025" name="in0">
///       <modParam name="inputFilter0" value="20"/>
///     </subModule>
///     <subModule id="1" ident="0x61100205" name="out0">
///       <modParam name="safeState" value="0"/>
///     </subModule>
///   </slave>
///
/// The coupler reports the modules it actually detects in 0xF050; the master
/// mirrors the configured list into 0xF030 so the coupler accepts the PDO
/// mapping at the SafeOp transition.
///
/// Addressing (per ESI / MDP conventions), for slot N (= `<subModule id>`):
///   CoE objects : inputs 0x6000 + N*SLOT_INCR, outputs 0x7000 + N*SLOT_INCR
///   PDOs        : TxPdo (in)  0x1A00 + N*PdoIncr
///                 RxPdo (out) 0x1600 + N*PdoIncr
///   SLOT_INCR is 0x10 for both couplers.  PdoIncr is 0x10 for R2EC and
///   0x08 for R3EC (packed into the type's `flags`, see LEADSHINE_EC_FLAG).

#ifndef _LCEC_LEADSHINE_EC_H_
#define _LCEC_LEADSHINE_EC_H_

#include <stdint.h>

#include "../lcec.h"
#include "lcec_class_ain.h"
#include "lcec_class_aout.h"
#include "lcec_class_din.h"
#include "lcec_class_dout.h"
#include "lcec_class_enc.h"

// CoE objects the coupler exposes.
#define LEADSHINE_EC_READMODULES 0xF050  // detected module ident list (read)
#define LEADSHINE_EC_CONFMODULES 0xF030  // configured module ident list (written by us)
#define LEADSHINE_EC_INOBJ       0x6000  // per-slot input object base
#define LEADSHINE_EC_OUTOBJ      0x7000  // per-slot output object base
#define LEADSHINE_EC_TXPDO       0x1A00  // per-slot input (Tx) PDO base
#define LEADSHINE_EC_RXPDO       0x1600  // per-slot output (Rx) PDO base
#define LEADSHINE_EC_SLOT_INCR   0x10    // CoE object index increment per slot

// The coupler is a CoE device with mailbox sync managers, so process-data
// PDOs live on SM2 (outputs) / SM3 (inputs).
#define LEADSHINE_EC_SM_MBOX_OUT 0  // SM0: mailbox out (empty)
#define LEADSHINE_EC_SM_MBOX_IN  1  // SM1: mailbox in (empty)
#define LEADSHINE_EC_SM_OUT      2  // SM2: RxPDO / outputs
#define LEADSHINE_EC_SM_IN       3  // SM3: TxPDO / inputs

// SM PDO-assignment objects: 0x1C10 + SM index (0x1C12 = SM2, 0x1C13 = SM3).
// Subindex 0 is the USINT count; subindices 1..N are UINT PDO indices.
#define LEADSHINE_EC_SM_PDO_ASSIGN(sm) (0x1C10 + (sm))

// Per-slot config object base (0x8000 + slot*SLOT_INCR) and the analog/temperature
// diagnostic object base (0xA000).  All addresses below are taken from the R3EC
// ESI (documentation/R3EC-v2.4.xml) and are DependOnSlot, i.e. base + slot*incr.
#define LEADSHINE_EC_CFGOBJ  0x8000  // per-slot config object base
#define LEADSHINE_EC_DIAGOBJ 0xA000  // analog/temp diagnostic (input) object base

// Config-object subindices (per the ESI DT8000 layouts).
#define LEADSHINE_EC_SUB_SAFESTATE_LO 1  // DO/relay: output state on link loss, bits 0-15  (UINT16)
#define LEADSHINE_EC_SUB_SAFESTATE_HI 2  // DO 32-ch: output state on link loss, bits 16-31 (UINT16)
#define LEADSHINE_EC_SUB_FILTER_BASE  3  // DI: input filter, subindices 3..6 per 8-channel group (UINT16)
// Analog full scale.  The class default is 0x7fff, but these modules are
// specified +/-32000 (manual 6.6 / 6.7.1: the DA link-loss preset takes
// -32000..32000).  Leaving the default would make a HAL value of 1.0 command
// ~2.4% past the module's rated full scale -- on the +/-10V range, 10.24V
// instead of 10.00V, which is a real scaling error for an analog velocity
// reference.
#define LEADSHINE_EC_ANALOG_FULLSCALE 32000

#define LEADSHINE_EC_SUB_ANALOG_BASE  1  // AIN/AOUT: per-channel range/mode config, sub 1..4 (USINT8)

// DA-only objects, offsets within the slot's 0x10-wide config window:
// <cfgbase>+2 = per-channel behaviour when the EtherCAT link is lost,
// <cfgbase>+3 = the value output in "preset" mode.  Both are sub 1..4.
#define LEADSHINE_EC_AOUT_LINKLOST_OBJ 2
#define LEADSHINE_EC_AOUT_LINKVAL_OBJ  3
#define LEADSHINE_EC_SUB_ENC_MODE     1  // encoder: operation mode (USINT8)
#define LEADSHINE_EC_SUB_ENC_ABPHASE  2  // encoder: AB phase (USINT8)
#define LEADSHINE_EC_SUB_ENC_SETVALUE 3  // encoder: preset counter value (DINT32)
#define LEADSHINE_EC_SUB_ENC_MIN      4  // encoder: minimum value (DINT32)
#define LEADSHINE_EC_SUB_ENC_MAX      5  // encoder: maximum value (DINT32)
#define LEADSHINE_EC_SUB_ENC_ZCLEAR   7  // encoder: clear counter on Z phase (USINT8)
#define LEADSHINE_EC_SUB_ENC_COUNTMODE 8  // encoder: count mode (USINT8)
#define LEADSHINE_EC_SUB_ENC_FILTER    9  // encoder: input filter (UINT16)

// The encoder module also carries 4 high-speed digital inputs (IN0..IN3) and
// 4 digital outputs (OUT0..OUT3) on its own terminals.  The inputs arrive in
// the "IO status INPUT" byte of the module's last TxPDO, whose object sits at
// offset 6 in the slot's CoE window; the outputs are the encoder RxPDO's
// single "general output" byte at <outbase>:01.
#define LEADSHINE_EC_ENC_IOOBJ    6  // offset of the IO-status object within the slot
#define LEADSHINE_EC_SUB_ENC_IO_OUT 1  // IO status OUTPUT (readback)
#define LEADSHINE_EC_SUB_ENC_IO_IN  2  // IO status INPUT
#define LEADSHINE_EC_ENC_DIN      4  // IN0..IN3
#define LEADSHINE_EC_ENC_DOUT     4  // OUT0..OUT3

/// @brief Pack per-type geometry into the typelist `flags` field.
/// @param max_slots Number of backplane slots the coupler supports.
/// @param pdo_incr  PDO index increment per slot (0x10 R2EC, 0x08 R3EC).
#define LEADSHINE_EC_FLAG(max_slots, pdo_incr) (((uint64_t)(max_slots) & 0xff) | (((uint64_t)(pdo_incr) & 0xff) << 8))
#define LEADSHINE_EC_MAX_SLOTS(f)              ((int)((f) & 0xff))
#define LEADSHINE_EC_PDO_INCR(f)               ((int)(((f) >> 8) & 0xff))

/// @brief Kind of module in a slot; selects the PDO shape and HAL class.
typedef enum {
  MODULE_UNKNOWN = 0,
  MODULE_DIN,      // digital input
  MODULE_DOUT,     // digital output (incl. relay)
  MODULE_DIO,      // mixed digital input + output
  MODULE_AIN,      // analog input
  MODULE_AOUT,     // analog output
  MODULE_ENCODER,  // encoder
} leadshine_ec_modkind_t;

/// @brief Static description of a supported module type.
///
/// `in` / `out` are the channel counts (digital: bits; analog/encoder:
/// channels).  This single table is the source of truth: the driver builds
/// the `lcec_submodule_desc_t[]` (attached to `types[].modules`) from it at
/// load time, so the XML parser can validate `<subModule ident=...>` and its
/// child `<modParam>`s.
typedef struct {
  uint32_t ident;                         // module ident (matches <subModule ident=...>)
  const char *name;                       // module name (per ESI)
  leadshine_ec_modkind_t type;            // module kind
  uint8_t in;                             // input channel count
  uint8_t out;                            // output channel count
  const lcec_modparam_desc_t *modparams;  // modparams supported by this module type
} leadshine_ec_module_def_t;

/// @brief Runtime state for one populated backplane slot.
///
/// Only the class container(s) relevant to the module kind are allocated; the
/// rest stay NULL.  Encoders don't use a class container (class_enc owns its
/// own struct), so we keep an array plus per-channel position PDO offsets.
typedef struct {
  uint8_t id;                        // backplane slot (= <subModule id>)
  uint32_t ident;                    // module ident
  leadshine_ec_modkind_t type;       // module kind
  lcec_class_din_channels_t *din;    // DIN / DIO inputs, or NULL
  lcec_class_dout_channels_t *dout;  // DOUT / DIO outputs, or NULL
  lcec_class_ain_channels_t *ain;    // AIN inputs, or NULL
  lcec_class_aout_channels_t *aout;  // AOUT outputs, or NULL
  int enc_count;                     // number of encoder channels
  lcec_class_enc_data_t *enc;        // encoder channel array, or NULL
  unsigned int *enc_pos_os;          // per-encoder position PDO offset, or NULL
  unsigned int *enc_err_os;          // per-encoder error-byte PDO offset, or NULL
  hal_u32_t **enc_error;             // per-encoder error-byte pin, or NULL
} leadshine_ec_slot_t;

/// @brief Per-slave HAL data: one entry per configured `<subModule>`.
typedef struct {
  int slot_count;
  leadshine_ec_slot_t *slots;
} lcec_leadshine_ec_data_t;

// Modparam ids.  These are the keys passed to lcec_submodule_modparam_get();
// they are per-module-type, so the same numeric id may recur across the
// digital/analog/encoder tables without conflict.
#define LEADSHINE_EC_MP_SAFESTATE      1  // digital output safe value on link loss
#define LEADSHINE_EC_MP_INPUTFILTER(g) (3 + (g))  // digital input filter, group g (0..3)
#define LEADSHINE_EC_MP_ANALOG_CFG(ch) (2 + (ch))  // analog channel config, ch (0..3)
#define LEADSHINE_EC_MP_AOUT_LINKLOST     6        // DA: output behaviour on link loss
#define LEADSHINE_EC_MP_AOUT_LINKLOSTVAL  7        // DA: output value on link loss
#define LEADSHINE_EC_MP_ENC_MODE       2
#define LEADSHINE_EC_MP_ENC_ABPHASE    3
#define LEADSHINE_EC_MP_ENC_MINVALUE   4
#define LEADSHINE_EC_MP_ENC_MAXVALUE   5
#define LEADSHINE_EC_MP_ENC_COUNTMODE  6
#define LEADSHINE_EC_MP_ENC_FILTER     7
#define LEADSHINE_EC_MP_ENC_SETVALUE   8
#define LEADSHINE_EC_MP_ENC_ZCLEAR     9

static const lcec_modparam_desc_t leadshine_ec_digital_params[] = {
    {"safeState", LEADSHINE_EC_MP_SAFESTATE, MODPARAM_TYPE_U32, "0", "Output value when link is lost (0 = all off)"},
    {"inputFilter0", LEADSHINE_EC_MP_INPUTFILTER(0), MODPARAM_TYPE_U32, "10", "Input filter time group 0 (channels 0-7), 0-255 ms"},
    {"inputFilter1", LEADSHINE_EC_MP_INPUTFILTER(1), MODPARAM_TYPE_U32, "10", "Input filter time group 1 (channels 8-15), 0-255 ms"},
    {"inputFilter2", LEADSHINE_EC_MP_INPUTFILTER(2), MODPARAM_TYPE_U32, "10", "Input filter time group 2 (channels 16-23), 0-255 ms"},
    {"inputFilter3", LEADSHINE_EC_MP_INPUTFILTER(3), MODPARAM_TYPE_U32, "10", "Input filter time group 3 (channels 24-31), 0-255 ms"},
    {NULL},
};

// Per-channel range/mode select, 0x8000+slot*0x10 sub 1..4.  AD and DA do not
// share an encoding, so they get separate tables -- otherwise the XML would
// accept a DA-only setting on an AD module and silently drop it.  Both ship
// correctly configured, so an omitted modparam writes nothing and leaves the
// module as-is.
//
// AD (R3-A0400-IV), per manual 6.6.  Note the codes are NOT the same as the
// DA module's below -- e.g. 2 is +/-10V here but +/-5V there.
#define LS_AD_RANGES "0=+/-5V 1=1-5V 2=+/-10V(default) 3=0-10V 4=0-20mA 5=4-20mA 6=0-5V 7=+/-20mA"
static const lcec_modparam_desc_t leadshine_ec_ain_params[] = {
    {"ch0Config", LEADSHINE_EC_MP_ANALOG_CFG(0), MODPARAM_TYPE_U32, NULL, "Channel 0 input range: " LS_AD_RANGES},
    {"ch1Config", LEADSHINE_EC_MP_ANALOG_CFG(1), MODPARAM_TYPE_U32, NULL, "Channel 1 input range: " LS_AD_RANGES},
    {"ch2Config", LEADSHINE_EC_MP_ANALOG_CFG(2), MODPARAM_TYPE_U32, NULL, "Channel 2 input range: " LS_AD_RANGES},
    {"ch3Config", LEADSHINE_EC_MP_ANALOG_CFG(3), MODPARAM_TYPE_U32, NULL, "Channel 3 input range: " LS_AD_RANGES},
    {NULL},
};

// DA (R3-A0004-IV), per manual 6.7.1.  Full scale is +/-32000 counts.
//
// linkLostMode matters for safety: the module's own default is 0 = Hold, so a
// dropped EtherCAT link leaves the last commanded value on the terminals.
// When the output is a velocity reference to a drive, that means the drive
// keeps running at the last commanded speed.  Set 1 (Clear) unless something
// else in the machine already handles that case.
static const lcec_modparam_desc_t leadshine_ec_aout_params[] = {
    {"ch0Config", LEADSHINE_EC_MP_ANALOG_CFG(0), MODPARAM_TYPE_U32, NULL,
        "Channel 0 output range: 0=0-5V 1=1-5V 2=+/-5V 3=0-10V 4=+/-10V 5=0-20mA 6=4-20mA"},
    {"ch1Config", LEADSHINE_EC_MP_ANALOG_CFG(1), MODPARAM_TYPE_U32, NULL,
        "Channel 1 output range: 0=0-5V 1=1-5V 2=+/-5V 3=0-10V 4=+/-10V 5=0-20mA 6=4-20mA"},
    {"ch2Config", LEADSHINE_EC_MP_ANALOG_CFG(2), MODPARAM_TYPE_U32, NULL,
        "Channel 2 output range: 0=0-5V 1=1-5V 2=+/-5V 3=0-10V 4=+/-10V 5=0-20mA 6=4-20mA"},
    {"ch3Config", LEADSHINE_EC_MP_ANALOG_CFG(3), MODPARAM_TYPE_U32, NULL,
        "Channel 3 output range: 0=0-5V 1=1-5V 2=+/-5V 3=0-10V 4=+/-10V 5=0-20mA 6=4-20mA"},
    {"linkLostMode", LEADSHINE_EC_MP_AOUT_LINKLOST, MODPARAM_TYPE_U32, NULL,
        "All channels, on EtherCAT link loss: 0=hold last value (module default), 1=clear to zero, 2=output linkLostValue"},
    {"linkLostValue", LEADSHINE_EC_MP_AOUT_LINKLOSTVAL, MODPARAM_TYPE_S32, NULL,
        "All channels, value output when linkLostMode=2 (-32000..32000, full scale)"},
    {NULL},
};

// Defaults below are the module's own power-on values, read back from an
// R3-E0200-S-V20 (0x8000:xx).  Keep them accurate: the driver only writes a
// setting the user actually specified, so these strings are documentation, and
// a wrong "default" here invites someone to copy it into their XML.  In
// particular min/max really are the full int32 range -- the previous
// +/-100000 would have silently wrapped the count after ~10 turns of a
// 2500 PPR encoder in 4x mode.
// Values below are from the R3 Series Extension Module User Manual, section
// 6.11.2 (R3-E0200-S).  Note encoderMode defaults to 0 = 1x, so a quadrature
// encoder counts one edge per cycle unless you ask for 2 (4x); on a 2500 PPR
// encoder that is 2500 counts/rev rather than 10000.
static const lcec_modparam_desc_t leadshine_ec_encoder_params[] = {
    {"encoderMode", LEADSHINE_EC_MP_ENC_MODE, MODPARAM_TYPE_U32, "0",
        "0=AB 1x, 1=AB 2x, 2=AB 4x, 3=pulse+direction, 4=CW/CCW"},
    {"abPhase", LEADSHINE_EC_MP_ENC_ABPHASE, MODPARAM_TYPE_U32, "0", "0=positive phase, 1=negative (reverses count direction)"},
    {"presetValue", LEADSHINE_EC_MP_ENC_SETVALUE, MODPARAM_TYPE_S32, "0", "Value loaded into the counter"},
    {"minValue", LEADSHINE_EC_MP_ENC_MINVALUE, MODPARAM_TYPE_S32, "-2147483647", "Counter minimum; only meaningful in ring mode"},
    {"maxValue", LEADSHINE_EC_MP_ENC_MAXVALUE, MODPARAM_TYPE_S32, "2147483647", "Counter maximum; only meaningful in ring mode"},
    {"zPhaseClear", LEADSHINE_EC_MP_ENC_ZCLEAR, MODPARAM_TYPE_U32, "0", "Z-phase pulse reset: 0=disabled, 1=enabled"},
    {"countMode", LEADSHINE_EC_MP_ENC_COUNTMODE, MODPARAM_TYPE_U32, "0", "0=ring (wraps at min/max), 1=linear"},
    {"encoderFilter", LEADSHINE_EC_MP_ENC_FILTER, MODPARAM_TYPE_U32, "2", "Input filter time, unit 100 ns (0-65535)"},
    {NULL},
};

/// @brief The one supported-module table (source of truth for `types[].modules`).
static const leadshine_ec_module_def_t leadshine_ec_module_table[] = {
    // ---- Digital input ----
    {0x61100025, "PM-1600", MODULE_DIN, 16, 0, leadshine_ec_digital_params},
    {0x61100026, "R3-1600-V20", MODULE_DIN, 16, 0, leadshine_ec_digital_params},
    {0x81100025, "R3-1600", MODULE_DIN, 16, 0, leadshine_ec_digital_params},
    {0x61100045, "PM-3200", MODULE_DIN, 32, 0, leadshine_ec_digital_params},
    {0x61100046, "R3-3200-V20", MODULE_DIN, 32, 0, leadshine_ec_digital_params},
    {0x61101045, "PM-3200-1", MODULE_DIN, 32, 0, leadshine_ec_digital_params},
    {0x61101046, "R3-3200-1-V20", MODULE_DIN, 32, 0, leadshine_ec_digital_params},
    {0x61102045, "PM-3200-2", MODULE_DIN, 32, 0, leadshine_ec_digital_params},
    {0x61102046, "R3-3200-2-V20", MODULE_DIN, 32, 0, leadshine_ec_digital_params},
    {0x81100045, "R3-3200", MODULE_DIN, 32, 0, leadshine_ec_digital_params},
    {0x81101045, "R3-3200-1", MODULE_DIN, 32, 0, leadshine_ec_digital_params},
    {0x81102045, "R3-3200-2", MODULE_DIN, 32, 0, leadshine_ec_digital_params},

    // ---- Digital output (incl. relay) ----
    {0x61100205, "PM-0016-N", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},
    {0x61100206, "R3-0016-N-V20", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},
    {0x81100205, "R3-0016-N", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},
    {0x61110205, "PM-0016-P", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},
    {0x61110206, "R3-0016-P-V20", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},
    {0x81110205, "R3-0016-P", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},
    {0x61100405, "PM-0032-N", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x61100406, "R3-0032-N-V20", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x61101405, "PM-0032-N-1", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x61101406, "R3-0032-N-1-V20", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x61102405, "PM-0032-N-2", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x61102406, "R3-0032-N-2-V20", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x81100405, "R3-0032-N", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x81101405, "R3-0032-N-1", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x81102405, "R3-0032-N-2", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x61110406, "R3-0032-P-V20", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x81110405, "R3-0032-P", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x61900106, "R3-0008-R-V20", MODULE_DOUT, 0, 8, leadshine_ec_digital_params},
    {0x81900105, "R3-0008-R", MODULE_DOUT, 0, 8, leadshine_ec_digital_params},
    {0x61900205, "PM-0016-R", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},
    {0x61900206, "R3-0016-R-V20", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},
    {0x81900205, "R3-0016-R", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},

    // ---- Mixed digital input + output ----
    {0x61100116, "R3-0808-N-V20", MODULE_DIO, 8, 8, leadshine_ec_digital_params},
    {0x81100115, "R3-0808-N", MODULE_DIO, 8, 8, leadshine_ec_digital_params},
    {0x61100225, "PM-1616-N", MODULE_DIO, 16, 16, leadshine_ec_digital_params},
    {0x61100226, "R3-1616-N-V20", MODULE_DIO, 16, 16, leadshine_ec_digital_params},
    {0x81100225, "R3-1616-N", MODULE_DIO, 16, 16, leadshine_ec_digital_params},
    {0x61110226, "R3-1616-P-V20", MODULE_DIO, 16, 16, leadshine_ec_digital_params},
    {0x81110225, "R3-1616-P", MODULE_DIO, 16, 16, leadshine_ec_digital_params},
    {0x61101446, "R3-3232-N-1-V20", MODULE_DIO, 32, 32, leadshine_ec_digital_params},
    {0x81101445, "R3-3232-N-1", MODULE_DIO, 32, 32, leadshine_ec_digital_params},

    // ---- Analog input ----
    {0x61000025, "PM-A0400-IV", MODULE_AIN, 4, 0, leadshine_ec_ain_params},
    {0x61000026, "R3-A0400-IV-V20", MODULE_AIN, 4, 0, leadshine_ec_ain_params},
    {0x81000025, "R3-A0400-IV", MODULE_AIN, 4, 0, leadshine_ec_ain_params},

    // ---- Analog output ----
    {0x61000205, "PM-A0004-IV", MODULE_AOUT, 0, 4, leadshine_ec_aout_params},
    {0x61000206, "R3-A0004-IV-V20", MODULE_AOUT, 0, 4, leadshine_ec_aout_params},
    {0x81000205, "R3-A0004-IV", MODULE_AOUT, 0, 4, leadshine_ec_aout_params},

    // ---- Encoder ----
    {0x61300025, "PM-E0200-S", MODULE_ENCODER, 2, 0, leadshine_ec_encoder_params},
    {0x61300026, "R3-E0200-S-V20", MODULE_ENCODER, 2, 0, leadshine_ec_encoder_params},
    {0x61300125, "PM-E0200-D", MODULE_ENCODER, 2, 0, leadshine_ec_encoder_params},
    {0x61300126, "R3-E0200-D-V20", MODULE_ENCODER, 2, 0, leadshine_ec_encoder_params},
    {0x81300025, "R3-E0200-S", MODULE_ENCODER, 2, 0, leadshine_ec_encoder_params},
    {0x81300125, "R3-E0200-D", MODULE_ENCODER, 2, 0, leadshine_ec_encoder_params},

    {0, NULL, MODULE_UNKNOWN, 0, 0, NULL},
};

#endif
