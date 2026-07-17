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
/// @brief Driver for the Leadshine R2EC / R3EC modular EtherCAT bus couplers.
///
/// See lcec_leadshine_ec.h for the device overview.  This is the first driver
/// to consume the generic `<subModule>` support: the coupler declares the
/// module types it accepts via `types[].modules` (built from
/// `leadshine_ec_module_table`), and in `_init` it walks the configured
/// `slave->submodules` list, builds a dynamic PDO/sync-manager layout, exports
/// the per-slot HAL pins through the `lcec_class_*` helpers, applies each
/// module's `<modParam>`s, and writes the configured module ident list (0xF030).
///
/// All CoE object / PDO / subindex addresses are taken from the R3EC ESI in
/// documentation/R3EC-v2.4.xml.  Digital modules default to the bit-wise PDO
/// mapping (object 0x6000/0x7000, one BOOL per subindex); packed-word mappings
/// (0x6001/0x6002) are not used.
///
/// ## R3-RS02-485 serial module (MODULE_SERIAL) -- UNTESTED against hardware
///
/// The R3-RS02-485 (ident 0x82100005, ESI class "485 Serials") is a 2-channel
/// RS-485 gateway.  It carries no LinuxCNC-native semantics; instead it maps a
/// small mailbox into the cyclic process image so userspace can push arbitrary
/// byte frames -- here specifically MOVILINK telegrams to a SEW MOVIDRIVE.
/// Determinism does NOT matter: this is a diagnostic / configuration channel,
/// so the HAL pins are plain u32 registers copied verbatim each cycle.
///
/// Per port p (0/1), the CoE objects live at OUTOBJ/INOBJ + slot*SLOT_INCR +
/// p*4.  The output (RxPDO) side carries a 5-word control header
/// (CtrlWord, OutputLength, TransmitEn, TransmitSID, ReadSID) plus 11 data
/// words (DataOut0..10, i.e. 22 bytes); the input (TxPDO) side mirrors it with
/// a status header (StateWord, InputSID, InputLength, TxFifoExist, RxFifoExist)
/// plus 11 data words (DataIn0..10).  All entries are 16-bit.
///
/// Software handshake (per the ESI, driven entirely from userspace):
///   TX: fill DataOut0..N, set OutputLength = N bytes, then increment
///       TransmitSID (send ID).  The module transmits the frame once and
///       echoes the SID in InputSID / advances StateWord when done.
///   RX: increment ReadSID to request the next received frame; the module
///       publishes it in DataIn0..N with InputLength set.  TxFifoExist /
///       RxFifoExist report FIFO occupancy.
/// The driver does not interpret any of this -- it only shuttles the words
/// between HAL and the process image.  There is no hardware to test against,
/// so the object/subindex map below is trusted from the ESI as documented.

#include "lcec_leadshine_ec.h"

#include <stdio.h>
#include <string.h>

#include "../lcec.h"

// HAL component id, created in lcec_main.c; used for the raw hal_pin_*_newf
// registrations in the serial module (the class helpers use it internally).
extern int lcec_comp_id;

static int lcec_leadshine_ec_init(int comp_id, lcec_slave_t *slave);
static void lcec_leadshine_ec_read(lcec_slave_t *slave, long period);
static void lcec_leadshine_ec_write(lcec_slave_t *slave, long period);

static lcec_typelist_t types[] = {
    {"R2EC", LCEC_LEADSHINE_VID, 0x61400005, 0, NULL, lcec_leadshine_ec_init, NULL, LEADSHINE_EC_FLAG(32, 0x10)},
    {"R3EC", LCEC_LEADSHINE_VID, 0x61400025, 0, NULL, lcec_leadshine_ec_init, NULL, LEADSHINE_EC_FLAG(64, 0x08)},
    {NULL},
};

/// @brief Register the coupler types, building the submodule descriptor table
/// from `leadshine_ec_module_table` (single source of truth) and attaching it
/// to every type's `.modules`.  Modeled on ADD_TYPES_WITH_CIA402_MODPARAMS;
/// runs at load time, so it uses the malloc-based LCEC_ALLOCATE_ARRAY.
static void AddTypesLeadshineEc(void) __attribute__((constructor));
static void AddTypesLeadshineEc(void) {
  int n = 0;
  while (leadshine_ec_module_table[n].name != NULL) n++;

  lcec_submodule_desc_t *subs = LCEC_ALLOCATE_ARRAY(lcec_submodule_desc_t, (n + 1));
  for (int i = 0; i < n; i++) {
    subs[i].name = leadshine_ec_module_table[i].name;
    subs[i].ident = leadshine_ec_module_table[i].ident;
    subs[i].modparams = leadshine_ec_module_table[i].modparams;
  }
  subs[n].name = NULL;  // NULL-terminate

  for (int i = 0; types[i].name != NULL; i++) {
    types[i].modules = subs;
  }
  lcec_addtypes(types, __FILE__);
}

/// @brief Look up a module definition by its ident.
static const leadshine_ec_module_def_t *leadshine_ec_find_module(uint32_t ident) {
  for (const leadshine_ec_module_def_t *m = leadshine_ec_module_table; m->name != NULL; m++) {
    if (m->ident == ident) {
      return m;
    }
  }
  return NULL;
}

/// @brief Count the configured submodules on a slave.
static int leadshine_ec_count_slots(lcec_slave_t *slave) {
  int n = 0;
  for (lcec_slave_submodule_t *s = slave->submodules; s != NULL; s = s->next) {
    n++;
  }
  return n;
}

/// @brief Allocate a persistent "<base>-<kind>-<ch>" HAL pin name.
static char *leadshine_ec_name(const char *base, const char *kind, int ch) {
  char buf[HAL_NAME_LEN];
  snprintf(buf, sizeof(buf), "%s-%s-%d", base, kind, ch);
  char *s = LCEC_HAL_ALLOCATE_ARRAY(char, (strlen(buf) + 1));
  strcpy(s, buf);
  return s;
}

/// @brief Allocate a persistent "<base>-<kind>" HAL pin-name prefix.
static char *leadshine_ec_prefix(const char *base, const char *kind) {
  char buf[HAL_NAME_LEN];
  snprintf(buf, sizeof(buf), "%s-%s", base, kind);
  char *s = LCEC_HAL_ALLOCATE_ARRAY(char, (strlen(buf) + 1));
  strcpy(s, buf);
  return s;
}

// ------------------------------------------------------------------
// PDO/sync-manager layout
//
// Each PDO/entry index is DependOnSlot: real index = base + slot*increment
// (SLOT_INCR for CoE objects, pdo_incr for PDO indices).  The bit-wise digital
// mapping (default) and the analog/encoder value+diagnostic mappings come
// straight from the ESI.  Mandatory input PDOs that we don't expose as HAL
// pins (analog/DA diagnostics, encoder error words) are still mapped so the
// module reaches OP.
// ------------------------------------------------------------------

/// @brief Append a slot's output (RxPDO, SM2) mapping.
static void leadshine_ec_append_output_pdos(lcec_syncs_t *syncs, lcec_slave_submodule_t *s, const leadshine_ec_module_def_t *def,
    int pdo_incr) {
  uint16_t obj = LEADSHINE_EC_OUTOBJ + s->id * LEADSHINE_EC_SLOT_INCR;
  uint16_t pdo = LEADSHINE_EC_RXPDO + s->id * pdo_incr;

  switch (def->type) {
    case MODULE_DOUT:
    case MODULE_DIO:
      if (def->out == 0) break;
      lcec_syncs_add_pdo_info(syncs, pdo);
      for (int ch = 0; ch < def->out; ch++) {
        lcec_syncs_add_pdo_entry(syncs, obj, ch + 1, 1);  // BOOL
      }
      break;
    case MODULE_AOUT:
      lcec_syncs_add_pdo_info(syncs, pdo);
      for (int ch = 0; ch < def->out; ch++) {
        lcec_syncs_add_pdo_entry(syncs, obj, ch + 1, 16);  // INT
      }
      break;
    case MODULE_ENCODER:
      // mandatory "general output" byte
      lcec_syncs_add_pdo_info(syncs, pdo);
      lcec_syncs_add_pdo_entry(syncs, obj, 1, 8);  // USINT
      break;
    case MODULE_SERIAL:
      // Two ports; port p uses object window obj+p*4 and PDOs pdo+p*4+{0..3}.
      // Per port: a 5-word control PDO then three data PDOs of 4/4/3 words.
      for (int p = 0; p < LEADSHINE_EC_SER_PORTS; p++) {
        uint16_t octrl = obj + p * LEADSHINE_EC_SER_PORT_INCR;
        uint16_t opdo = pdo + p * LEADSHINE_EC_SER_PORT_INCR;
        lcec_syncs_add_pdo_info(syncs, opdo);  // control: CtrlWord..ReadSID
        for (int sub = 1; sub <= 5; sub++) {
          lcec_syncs_add_pdo_entry(syncs, octrl, sub, 16);  // UINT
        }
        lcec_syncs_add_pdo_info(syncs, opdo + 1);  // DataOut0..3
        for (int sub = 1; sub <= 4; sub++) {
          lcec_syncs_add_pdo_entry(syncs, octrl + 1, sub, 16);
        }
        lcec_syncs_add_pdo_info(syncs, opdo + 2);  // DataOut4..7
        for (int sub = 1; sub <= 4; sub++) {
          lcec_syncs_add_pdo_entry(syncs, octrl + 2, sub, 16);
        }
        lcec_syncs_add_pdo_info(syncs, opdo + 3);  // DataOut8..10
        for (int sub = 1; sub <= 3; sub++) {
          lcec_syncs_add_pdo_entry(syncs, octrl + 3, sub, 16);
        }
      }
      break;
    default: break;
  }
}

/// @brief Append a slot's input (TxPDO, SM3) mapping.
static void leadshine_ec_append_input_pdos(lcec_syncs_t *syncs, lcec_slave_submodule_t *s, const leadshine_ec_module_def_t *def,
    int pdo_incr) {
  uint16_t obj = LEADSHINE_EC_INOBJ + s->id * LEADSHINE_EC_SLOT_INCR;
  uint16_t diag = LEADSHINE_EC_DIAGOBJ + s->id * LEADSHINE_EC_SLOT_INCR;
  uint16_t pdo = LEADSHINE_EC_TXPDO + s->id * pdo_incr;

  switch (def->type) {
    case MODULE_DIN:
    case MODULE_DIO:
      if (def->in == 0) break;
      lcec_syncs_add_pdo_info(syncs, pdo);
      for (int ch = 0; ch < def->in; ch++) {
        lcec_syncs_add_pdo_entry(syncs, obj, ch + 1, 1);  // BOOL
      }
      break;
    case MODULE_AIN:
      lcec_syncs_add_pdo_info(syncs, pdo);  // 0x1A00 value
      for (int ch = 0; ch < def->in; ch++) {
        lcec_syncs_add_pdo_entry(syncs, obj, ch + 1, 16);  // INT
      }
      lcec_syncs_add_pdo_info(syncs, pdo + 1);  // 0x1A01 diagnostic (mandatory)
      for (int ch = 0; ch < def->in; ch++) {
        lcec_syncs_add_pdo_entry(syncs, diag, ch + 1, 8);  // USINT
      }
      break;
    case MODULE_AOUT:
      // analog output modules also carry a mandatory input diagnostic PDO
      lcec_syncs_add_pdo_info(syncs, pdo);  // 0x1A00 diagnostic
      for (int ch = 0; ch < def->out; ch++) {
        lcec_syncs_add_pdo_entry(syncs, diag, ch + 1, 8);  // USINT
      }
      break;
    case MODULE_ENCODER:
      lcec_syncs_add_pdo_info(syncs, pdo);  // 0x1A00 position + error
      for (int ch = 0; ch < def->in; ch++) {
        lcec_syncs_add_pdo_entry(syncs, obj, ch + 1, 32);  // DINT position
      }
      for (int ch = 0; ch < def->in; ch++) {
        lcec_syncs_add_pdo_entry(syncs, obj, def->in + ch + 1, 8);  // USINT error
      }
      // TODO(hw): the encoder also declares further mandatory TxPDOs (0x1A01+,
      // latch/compare).  They are not mapped here; if a module fails to reach
      // OP without them, add them following the ESI.
      break;
    case MODULE_SERIAL:
      // Mirror of the output side: per port a 5-word status PDO then three data
      // PDOs of 4/4/3 words (INOBJ / TXPDO).
      for (int p = 0; p < LEADSHINE_EC_SER_PORTS; p++) {
        uint16_t istat = obj + p * LEADSHINE_EC_SER_PORT_INCR;
        uint16_t ipdo = pdo + p * LEADSHINE_EC_SER_PORT_INCR;
        lcec_syncs_add_pdo_info(syncs, ipdo);  // status: StateWord..RxFifoExist
        for (int sub = 1; sub <= 5; sub++) {
          lcec_syncs_add_pdo_entry(syncs, istat, sub, 16);  // UINT
        }
        lcec_syncs_add_pdo_info(syncs, ipdo + 1);  // DataIn0..3
        for (int sub = 1; sub <= 4; sub++) {
          lcec_syncs_add_pdo_entry(syncs, istat + 1, sub, 16);
        }
        lcec_syncs_add_pdo_info(syncs, ipdo + 2);  // DataIn4..7
        for (int sub = 1; sub <= 4; sub++) {
          lcec_syncs_add_pdo_entry(syncs, istat + 2, sub, 16);
        }
        lcec_syncs_add_pdo_info(syncs, ipdo + 3);  // DataIn8..10
        for (int sub = 1; sub <= 3; sub++) {
          lcec_syncs_add_pdo_entry(syncs, istat + 3, sub, 16);
        }
      }
      break;
    default: break;
  }
}

/// @brief Build the dynamic sync-manager / PDO layout for the configured slots.
///
/// SM0/SM1 are (empty) mailbox syncs; process data lives on SM2 (outputs) and
/// SM3 (inputs).  All outputs are grouped under SM2 and all inputs under SM3,
/// because the builder appends PDOs to the most recently added sync.
static void leadshine_ec_build_syncs(lcec_slave_t *slave, lcec_syncs_t *syncs, int pdo_incr) {
  lcec_syncs_init(slave, syncs);

  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_DEFAULT);  // SM0 mailbox out
  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DEFAULT);   // SM1 mailbox in

  // SM2: outputs (RxPDO)
  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_DEFAULT);
  for (lcec_slave_submodule_t *s = slave->submodules; s != NULL; s = s->next) {
    const leadshine_ec_module_def_t *def = leadshine_ec_find_module(s->ident);
    if (def != NULL) {
      leadshine_ec_append_output_pdos(syncs, s, def, pdo_incr);
    }
  }

  // SM3: inputs (TxPDO)
  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DEFAULT);
  for (lcec_slave_submodule_t *s = slave->submodules; s != NULL; s = s->next) {
    const leadshine_ec_module_def_t *def = leadshine_ec_find_module(s->ident);
    if (def != NULL) {
      leadshine_ec_append_input_pdos(syncs, s, def, pdo_incr);
    }
  }

  slave->sync_info = &syncs->syncs[0];
}

// ------------------------------------------------------------------
// Per-slot HAL registration
// ------------------------------------------------------------------

static void leadshine_ec_register_din(lcec_slave_t *slave, leadshine_ec_slot_t *slot, const char *base, int count) {
  uint16_t obj = LEADSHINE_EC_INOBJ + slot->id * LEADSHINE_EC_SLOT_INCR;
  slot->din = lcec_din_allocate_channels(count);
  for (int ch = 0; ch < count; ch++) {
    slot->din->channels[ch] = lcec_din_register_channel_named(slave, obj, ch + 1, leadshine_ec_name(base, "din", ch));
  }
}

static void leadshine_ec_register_dout(lcec_slave_t *slave, leadshine_ec_slot_t *slot, const char *base, int count) {
  uint16_t obj = LEADSHINE_EC_OUTOBJ + slot->id * LEADSHINE_EC_SLOT_INCR;
  slot->dout = lcec_dout_allocate_channels(count);
  for (int ch = 0; ch < count; ch++) {
    slot->dout->channels[ch] = lcec_dout_register_channel_named(slave, obj, ch + 1, leadshine_ec_name(base, "dout", ch));
  }
}

static void leadshine_ec_register_ain(lcec_slave_t *slave, leadshine_ec_slot_t *slot, const char *base, int count) {
  uint16_t obj = LEADSHINE_EC_INOBJ + slot->id * LEADSHINE_EC_SLOT_INCR;
  char *prefix = leadshine_ec_prefix(base, "ain");
  slot->ain = lcec_ain_allocate_channels(count);
  for (int ch = 0; ch < count; ch++) {
    lcec_class_ain_options_t *opt = lcec_ain_options();
    opt->name_prefix = prefix;
    opt->valueonly = 1;        // diagnostics are mapped but not exposed as pins
    opt->value_sidx = ch + 1;  // 0x6000:1..N, INT16 (ESI)
    slot->ain->channels[ch] = lcec_ain_register_channel(slave, ch, obj, opt);
  }
}

static void leadshine_ec_register_aout(lcec_slave_t *slave, leadshine_ec_slot_t *slot, const char *base, int count) {
  uint16_t obj = LEADSHINE_EC_OUTOBJ + slot->id * LEADSHINE_EC_SLOT_INCR;
  char *prefix = leadshine_ec_prefix(base, "aout");
  slot->aout = lcec_aout_allocate_channels(count);
  for (int ch = 0; ch < count; ch++) {
    lcec_class_aout_options_t *opt = lcec_aout_options();
    opt->name_prefix = prefix;
    opt->value_sidx = ch + 1;  // 0x7000:1..N, INT16 (ESI)
    slot->aout->channels[ch] = lcec_aout_register_channel(slave, ch, obj, opt);
  }
}

static void leadshine_ec_register_enc(lcec_slave_t *slave, leadshine_ec_slot_t *slot, const char *base, int count) {
  uint16_t obj = LEADSHINE_EC_INOBJ + slot->id * LEADSHINE_EC_SLOT_INCR;
  slot->enc_count = count;
  slot->enc = LCEC_HAL_ALLOCATE_ARRAY(lcec_class_enc_data_t, count);
  slot->enc_pos_os = LCEC_HAL_ALLOCATE_ARRAY(unsigned int, count);
  for (int ch = 0; ch < count; ch++) {
    class_enc_init(slave, &slot->enc[ch], 32, leadshine_ec_name(base, "enc", ch));
    // class_enc registers no PDO itself; map the 32-bit position value here
    // (0x6000:1..N, DINT).
    lcec_pdo_init(slave, obj, ch + 1, &slot->enc_pos_os[ch], NULL);
  }
}

/// @brief Register one serial-port HAL pin and capture its process-data offset.
///
/// Name shape: "<module>.<master>.<slave>.<base>-p<port>-<leaf>", e.g.
/// "lcec.0.r3ec01.ser-p0-out0".  `dir` is HAL_IN for control/output words
/// (user -> module) and HAL_OUT for status/input words (module -> user).
static void leadshine_ec_register_ser_pin(lcec_slave_t *slave, const char *base, int port, const char *leaf, hal_pin_dir_t dir,
    hal_u32_t **pin, uint16_t obj, uint16_t subidx, unsigned int *os) {
  lcec_master_t *master = slave->master;
  int err = hal_pin_u32_newf(dir, pin, lcec_comp_id, "%s.%s.%s.%s-p%d-%s", LCEC_MODULE_NAME, master->name, slave->name, base, port,
      leaf);
  if (err != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "%s.%s: failed registering serial pin %s-p%d-%s\n", master->name, slave->name, base,
        port, leaf);
    return;
  }
  // Capture the cyclic-image offset for the same 16-bit object entry.
  lcec_pdo_init(slave, obj, subidx, os, NULL);
}

static void leadshine_ec_register_serial(lcec_slave_t *slave, leadshine_ec_slot_t *slot, const char *base, int ports) {
  uint16_t octrl_base = LEADSHINE_EC_OUTOBJ + slot->id * LEADSHINE_EC_SLOT_INCR;
  uint16_t istat_base = LEADSHINE_EC_INOBJ + slot->id * LEADSHINE_EC_SLOT_INCR;
  char leaf[16];

  slot->ser_ports = ports;
  for (int p = 0; p < ports; p++) {
    leadshine_ec_serial_port_t *port = &slot->ser[p];
    uint16_t octrl = octrl_base + p * LEADSHINE_EC_SER_PORT_INCR;
    uint16_t istat = istat_base + p * LEADSHINE_EC_SER_PORT_INCR;

    // Output / control words (HAL_IN, user -> module): 0x7000+ ctrl object subs
    // 1..5, then DataOut0..10 spread over data objects octrl+1..+3.
    leadshine_ec_register_ser_pin(slave, base, p, "ctrl-word", HAL_IN, &port->ctrl_word, octrl, 1, &port->ctrl_word_os);
    leadshine_ec_register_ser_pin(slave, base, p, "out-length", HAL_IN, &port->out_length, octrl, 2, &port->out_length_os);
    leadshine_ec_register_ser_pin(slave, base, p, "transmit-en", HAL_IN, &port->transmit_en, octrl, 3, &port->transmit_en_os);
    leadshine_ec_register_ser_pin(slave, base, p, "transmit-sid", HAL_IN, &port->transmit_sid, octrl, 4, &port->transmit_sid_os);
    leadshine_ec_register_ser_pin(slave, base, p, "read-sid", HAL_IN, &port->read_sid, octrl, 5, &port->read_sid_os);
    for (int w = 0; w < LEADSHINE_EC_SER_DATA_WORDS; w++) {
      uint16_t dobj = octrl + 1 + w / 4;  // data objects octrl+1, +2, +3
      uint16_t dsub = (w % 4) + 1;        // subindices 1..4 (last object 1..3)
      snprintf(leaf, sizeof(leaf), "out%d", w);
      leadshine_ec_register_ser_pin(slave, base, p, leaf, HAL_IN, &port->out[w], dobj, dsub, &port->out_os[w]);
    }

    // Input / status words (HAL_OUT, module -> user): 0x6000+ status object subs
    // 1..5, then DataIn0..10 spread over data objects istat+1..+3.
    leadshine_ec_register_ser_pin(slave, base, p, "state-word", HAL_OUT, &port->state_word, istat, 1, &port->state_word_os);
    leadshine_ec_register_ser_pin(slave, base, p, "in-sid", HAL_OUT, &port->in_sid, istat, 2, &port->in_sid_os);
    leadshine_ec_register_ser_pin(slave, base, p, "in-length", HAL_OUT, &port->in_length, istat, 3, &port->in_length_os);
    leadshine_ec_register_ser_pin(slave, base, p, "tx-fifo", HAL_OUT, &port->tx_fifo, istat, 4, &port->tx_fifo_os);
    leadshine_ec_register_ser_pin(slave, base, p, "rx-fifo", HAL_OUT, &port->rx_fifo, istat, 5, &port->rx_fifo_os);
    for (int w = 0; w < LEADSHINE_EC_SER_DATA_WORDS; w++) {
      uint16_t dobj = istat + 1 + w / 4;  // data objects istat+1, +2, +3
      uint16_t dsub = (w % 4) + 1;        // subindices 1..4 (last object 1..3)
      snprintf(leaf, sizeof(leaf), "in%d", w);
      leadshine_ec_register_ser_pin(slave, base, p, leaf, HAL_OUT, &port->in[w], dobj, dsub, &port->in_os[w]);
    }
  }
}

// ------------------------------------------------------------------
// Modparam application (per slot).  Addresses per the R3EC ESI DT8000 layouts.
// Writes are only issued for modparams the user actually set.
// ------------------------------------------------------------------

static int leadshine_ec_apply_modparams(lcec_slave_t *slave, lcec_slave_submodule_t *sub, const leadshine_ec_module_def_t *def) {
  LCEC_CONF_MODPARAM_VAL_T *v;
  uint16_t cfg = LEADSHINE_EC_CFGOBJ + sub->id * LEADSHINE_EC_SLOT_INCR;
  int err;

  // digital output safe state: 16-bit low word (bits 0-15) at sub 1, high word
  // (bits 16-31) at sub 2 for 32-channel modules.
  if (def->type == MODULE_DOUT || def->type == MODULE_DIO) {
    v = lcec_submodule_modparam_get(sub, LEADSHINE_EC_MP_SAFESTATE);
    if (v != NULL) {
      if ((err = lcec_write_sdo16(slave, cfg, LEADSHINE_EC_SUB_SAFESTATE_LO, v->u32 & 0xffff)) != 0) {
        return err;
      }
      if (def->out > 16 && (err = lcec_write_sdo16(slave, cfg, LEADSHINE_EC_SUB_SAFESTATE_HI, (v->u32 >> 16) & 0xffff)) != 0) {
        return err;
      }
    }
  }

  // digital input filters: one 16-bit value per group of 8 channels, sub 3..6.
  if (def->type == MODULE_DIN || def->type == MODULE_DIO) {
    int groups = (def->in + 7) / 8;
    for (int g = 0; g < groups && g < 4; g++) {
      v = lcec_submodule_modparam_get(sub, LEADSHINE_EC_MP_INPUTFILTER(g));
      if (v != NULL && (err = lcec_write_sdo16(slave, cfg, LEADSHINE_EC_SUB_FILTER_BASE + g, v->u32 & 0xffff)) != 0) {
        return err;
      }
    }
  }

  // analog per-channel range/mode config: 8-bit value at sub 1..4.
  if (def->type == MODULE_AIN || def->type == MODULE_AOUT) {
    int count = (def->type == MODULE_AIN) ? def->in : def->out;
    for (int ch = 0; ch < count && ch < 4; ch++) {
      v = lcec_submodule_modparam_get(sub, LEADSHINE_EC_MP_ANALOG_CFG(ch));
      if (v != NULL && (err = lcec_write_sdo8(slave, cfg, LEADSHINE_EC_SUB_ANALOG_BASE + ch, v->u32 & 0xff)) != 0) {
        return err;
      }
    }
  }

  // encoder config: one object per encoder channel (0x8000 = ch0, 0x8001 = ch1).
  // A single set of modparams is applied to every channel on the module.
  if (def->type == MODULE_ENCODER) {
    for (int ch = 0; ch < def->in; ch++) {
      uint16_t eobj = cfg + ch;
      if ((v = lcec_submodule_modparam_get(sub, LEADSHINE_EC_MP_ENC_MODE)) != NULL &&
          (err = lcec_write_sdo8(slave, eobj, LEADSHINE_EC_SUB_ENC_MODE, v->u32 & 0xff)) != 0) {
        return err;
      }
      if ((v = lcec_submodule_modparam_get(sub, LEADSHINE_EC_MP_ENC_ABPHASE)) != NULL &&
          (err = lcec_write_sdo8(slave, eobj, LEADSHINE_EC_SUB_ENC_ABPHASE, v->u32 & 0xff)) != 0) {
        return err;
      }
      if ((v = lcec_submodule_modparam_get(sub, LEADSHINE_EC_MP_ENC_MINVALUE)) != NULL &&
          (err = lcec_write_sdo32(slave, eobj, LEADSHINE_EC_SUB_ENC_MIN, (uint32_t)v->s32)) != 0) {
        return err;
      }
      if ((v = lcec_submodule_modparam_get(sub, LEADSHINE_EC_MP_ENC_MAXVALUE)) != NULL &&
          (err = lcec_write_sdo32(slave, eobj, LEADSHINE_EC_SUB_ENC_MAX, (uint32_t)v->s32)) != 0) {
        return err;
      }
      if ((v = lcec_submodule_modparam_get(sub, LEADSHINE_EC_MP_ENC_COUNTMODE)) != NULL &&
          (err = lcec_write_sdo8(slave, eobj, LEADSHINE_EC_SUB_ENC_COUNTMODE, v->u32 & 0xff)) != 0) {
        return err;
      }
      if ((v = lcec_submodule_modparam_get(sub, LEADSHINE_EC_MP_ENC_FILTER)) != NULL &&
          (err = lcec_write_sdo16(slave, eobj, LEADSHINE_EC_SUB_ENC_FILTER, v->u32 & 0xffff)) != 0) {
        return err;
      }
    }
  }

  return 0;
}

/// @brief Write the configured module ident list (0xF030) so the coupler
/// accepts the PDO mapping.  Per the ESI, sub 0 is a USINT count and subs 1..N
/// are one UDINT (32-bit) module ident each (slot id + 1).
static void leadshine_ec_write_module_list(lcec_slave_t *slave) {
  int count = 0;

  for (lcec_slave_submodule_t *s = slave->submodules; s != NULL; s = s->next) {
    if (lcec_write_sdo32(slave, LEADSHINE_EC_CONFMODULES, s->id + 1, s->ident) != 0) {
      rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX "%s.%s: failed writing module ident 0x%08x to 0x%04x:%02x\n", slave->master->name,
          slave->name, s->ident, LEADSHINE_EC_CONFMODULES, s->id + 1);
    }
    if (s->id + 1 > count) {
      count = s->id + 1;
    }
  }

  if (lcec_write_sdo8(slave, LEADSHINE_EC_CONFMODULES, 0x00, count) != 0) {
    rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX "%s.%s: failed writing module count to 0x%04x:00\n", slave->master->name, slave->name,
        LEADSHINE_EC_CONFMODULES);
  }
}

/// @brief Explicitly write the SM2/SM3 PDO assignment objects (0x1C12/0x1C13).
///
/// The master's automatic assignment (from `sync_info`) does not "take" for
/// this coupler's input sync manager — the default input PDO (0x1A00) is marked
/// `Fixed` in the ESI, so the master skips writing 0x1C13 and the module's
/// inputs never appear.  We therefore force the assignment by hand, exactly as
/// the CoE PDO-assignment protocol requires: clear the count (sub 0 = 0), write
/// each active PDO index (sub 1..N), then write the count (sub 0 = N).
///
/// `sync_info` is still provided so the master knows the PDO/entry layout for
/// process-image offset resolution; the values written here match it, taken
/// straight from the built sync structure so the two cannot drift.
static int leadshine_ec_assign_pdos(lcec_slave_t *slave, lcec_syncs_t *syncs) {
  ec_sync_info_t *sm_out = &syncs->syncs[LEADSHINE_EC_SM_OUT];
  ec_sync_info_t *sm_in = &syncs->syncs[LEADSHINE_EC_SM_IN];
  uint16_t out_obj = LEADSHINE_EC_SM_PDO_ASSIGN(LEADSHINE_EC_SM_OUT);  // 0x1C12
  uint16_t in_obj = LEADSHINE_EC_SM_PDO_ASSIGN(LEADSHINE_EC_SM_IN);    // 0x1C13
  int err;

  // clear both assignments first
  if ((err = lcec_write_sdo8(slave, out_obj, 0, 0)) != 0) return err;
  if ((err = lcec_write_sdo8(slave, in_obj, 0, 0)) != 0) return err;

  // assign the output PDOs (SM2)
  for (unsigned int i = 0; i < sm_out->n_pdos; i++) {
    if ((err = lcec_write_sdo16(slave, out_obj, i + 1, sm_out->pdos[i].index)) != 0) return err;
  }
  // assign the input PDOs (SM3)
  for (unsigned int i = 0; i < sm_in->n_pdos; i++) {
    if ((err = lcec_write_sdo16(slave, in_obj, i + 1, sm_in->pdos[i].index)) != 0) return err;
  }

  // write the counts last to activate each assignment
  if ((err = lcec_write_sdo8(slave, out_obj, 0, sm_out->n_pdos)) != 0) return err;
  if ((err = lcec_write_sdo8(slave, in_obj, 0, sm_in->n_pdos)) != 0) return err;

  return 0;
}

// ------------------------------------------------------------------
// init / read / write
// ------------------------------------------------------------------

static int lcec_leadshine_ec_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  int pdo_incr = LEADSHINE_EC_PDO_INCR(slave->flags);
  int max_slots = LEADSHINE_EC_MAX_SLOTS(slave->flags);
  int err;

  lcec_leadshine_ec_data_t *hal_data = LCEC_HAL_ALLOCATE(lcec_leadshine_ec_data_t);
  slave->hal_data = hal_data;
  hal_data->slot_count = leadshine_ec_count_slots(slave);
  hal_data->slots = hal_data->slot_count > 0 ? LCEC_HAL_ALLOCATE_ARRAY(leadshine_ec_slot_t, hal_data->slot_count) : NULL;

  // Build the dynamic PDO / sync-manager layout (consumed after _init returns).
  lcec_syncs_t *syncs = LCEC_HAL_ALLOCATE(lcec_syncs_t);
  leadshine_ec_build_syncs(slave, syncs, pdo_incr);

  // Register HAL pins + PDO entries and apply modparams, one slot at a time.
  int i = 0;
  for (lcec_slave_submodule_t *s = slave->submodules; s != NULL; s = s->next, i++) {
    leadshine_ec_slot_t *slot = &hal_data->slots[i];
    const leadshine_ec_module_def_t *def = leadshine_ec_find_module(s->ident);

    // The parser validates idents against this same table, so this should not
    // happen; guard defensively rather than crash.
    if (def == NULL) {
      rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX "%s.%s: unknown submodule ident 0x%08x in slot %d, skipping\n", master->name,
          slave->name, s->ident, s->id);
      continue;
    }
    if (s->id >= max_slots) {
      rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX "%s.%s: submodule slot %d exceeds max %d for this coupler\n", master->name,
          slave->name, s->id, max_slots);
    }

    slot->id = s->id;
    slot->ident = s->ident;
    slot->type = def->type;

    switch (def->type) {
      case MODULE_DIN: leadshine_ec_register_din(slave, slot, s->name, def->in); break;
      case MODULE_DOUT: leadshine_ec_register_dout(slave, slot, s->name, def->out); break;
      case MODULE_DIO:
        leadshine_ec_register_din(slave, slot, s->name, def->in);
        leadshine_ec_register_dout(slave, slot, s->name, def->out);
        break;
      case MODULE_AIN: leadshine_ec_register_ain(slave, slot, s->name, def->in); break;
      case MODULE_AOUT: leadshine_ec_register_aout(slave, slot, s->name, def->out); break;
      case MODULE_ENCODER: leadshine_ec_register_enc(slave, slot, s->name, def->in); break;
      case MODULE_SERIAL: leadshine_ec_register_serial(slave, slot, s->name, LEADSHINE_EC_SER_PORTS); break;
      default: break;
    }

    if ((err = leadshine_ec_apply_modparams(slave, s, def)) != 0) {
      return err;
    }
  }

  // Tell the coupler which modules the config expects, then force the SM PDO
  // assignment (the master does not reliably assign the input SM by itself).
  leadshine_ec_write_module_list(slave);
  if ((err = leadshine_ec_assign_pdos(slave, syncs)) != 0) {
    return err;
  }

  slave->proc_read = lcec_leadshine_ec_read;
  slave->proc_write = lcec_leadshine_ec_write;
  return 0;
}

static void lcec_leadshine_ec_read(lcec_slave_t *slave, long period) {
  lcec_leadshine_ec_data_t *hal_data = (lcec_leadshine_ec_data_t *)slave->hal_data;
  uint8_t *pd = slave->master->process_data;

  if (!slave->state.operational) {
    return;
  }

  for (int i = 0; i < hal_data->slot_count; i++) {
    leadshine_ec_slot_t *slot = &hal_data->slots[i];
    if (slot->din != NULL) {
      lcec_din_read_all(slave, slot->din);
    }
    if (slot->ain != NULL) {
      lcec_ain_read_all(slave, slot->ain);
    }
    for (int ch = 0; ch < slot->enc_count; ch++) {
      class_enc_update(&slot->enc[ch], 0, 1.0, EC_READ_U32(&pd[slot->enc_pos_os[ch]]), 0, 0);
    }
    for (int p = 0; p < slot->ser_ports; p++) {
      leadshine_ec_serial_port_t *port = &slot->ser[p];
      *(port->state_word) = EC_READ_U16(&pd[port->state_word_os]);
      *(port->in_sid) = EC_READ_U16(&pd[port->in_sid_os]);
      *(port->in_length) = EC_READ_U16(&pd[port->in_length_os]);
      *(port->tx_fifo) = EC_READ_U16(&pd[port->tx_fifo_os]);
      *(port->rx_fifo) = EC_READ_U16(&pd[port->rx_fifo_os]);
      for (int w = 0; w < LEADSHINE_EC_SER_DATA_WORDS; w++) {
        *(port->in[w]) = EC_READ_U16(&pd[port->in_os[w]]);
      }
    }
  }
}

static void lcec_leadshine_ec_write(lcec_slave_t *slave, long period) {
  lcec_leadshine_ec_data_t *hal_data = (lcec_leadshine_ec_data_t *)slave->hal_data;
  uint8_t *pd = slave->master->process_data;

  if (!slave->state.operational) {
    return;
  }

  for (int i = 0; i < hal_data->slot_count; i++) {
    leadshine_ec_slot_t *slot = &hal_data->slots[i];
    if (slot->dout != NULL) {
      lcec_dout_write_all(slave, slot->dout);
    }
    if (slot->aout != NULL) {
      lcec_aout_write_all(slave, slot->aout);
    }
    for (int p = 0; p < slot->ser_ports; p++) {
      leadshine_ec_serial_port_t *port = &slot->ser[p];
      EC_WRITE_U16(&pd[port->ctrl_word_os], (uint16_t)*(port->ctrl_word));
      EC_WRITE_U16(&pd[port->out_length_os], (uint16_t)*(port->out_length));
      EC_WRITE_U16(&pd[port->transmit_en_os], (uint16_t)*(port->transmit_en));
      EC_WRITE_U16(&pd[port->transmit_sid_os], (uint16_t)*(port->transmit_sid));
      EC_WRITE_U16(&pd[port->read_sid_os], (uint16_t)*(port->read_sid));
      for (int w = 0; w < LEADSHINE_EC_SER_DATA_WORDS; w++) {
        EC_WRITE_U16(&pd[port->out_os[w]], (uint16_t)*(port->out[w]));
      }
    }
  }
}
