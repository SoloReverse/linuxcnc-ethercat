# Leadshine R2EC / R3EC bus couplers

The R2EC and R3EC are single EtherCAT slaves that host up to 32 (R2EC) or 64
(R3EC) I/O modules on a passive backplane.  Each populated slot is described in
the LinuxCNC XML config as a `<subModule>`:

```xml
<slave idx="0" type="R3EC" name="io">
  <subModule id="0" ident="0x61300026" name="enc">
    <modParam name="encoderMode"     value="2"/>   <!-- AB 4x -->
    <modParam name="ch0CountsPerRev" value="8000"/><!-- 2000 PPR x 4 -->
    <modParam name="in0Mode"         value="0"/>   <!-- plain digital input -->
    <modParam name="out0Mode"        value="0"/>   <!-- output follows HAL pin -->
  </subModule>
</slave>
```

`id` is the physical slot, counted from 0 at the module nearest the coupler.
`ident` must match what the coupler reports in `0xF050`; read it with
`ethercat upload -p<n> -t uint32 0xF050 1` (subindex 0 is the module count).

Per-slot CoE addressing is `base + slot * 0x10`: inputs `0x6000`, outputs
`0x7000`, config `0x8000`, identity `0x9000`.  PDO indices are
`0x1A00 + slot * PdoIncr` (TxPDO) and `0x1600 + slot * PdoIncr`, where PdoIncr
is `0x10` for R2EC and `0x08` for R3EC.

## Encoder modules and the port-function defaults

The `*-E0200-S` (single-ended, 24 V) and `*-E0200-D` (differential, 5 V)
modules provide 2 encoder channels plus 4 high-speed digital inputs and 4
outputs.  Those 8 terminals do **not** default to plain digital I/O, so the
`din`/`dout` pins are inert until you say otherwise:

* `out<n>Mode` defaults to comparator-driven, so an output pin can be set,
  appear correctly in the process image, and drive nothing.  Set it to `0`.
* `in0Mode`/`in2Mode` default to `2` (high-speed latch), so those pins never
  change state.  Set them to `0`.  Note mode `3` is *clear the encoder
  counter* -- setting it on a wired input pins the count at zero.

`ch<n>CountsPerRev` is consumed by the encoder class, never written to the
module, and is needed for `index-enable` to snap to the Z boundary instead of
degrading to a plain position reset.  It is **counts**, not PPR: a 2000 PPR
encoder in AB 4x mode is 8000.

## Troubleshooting: no input data from a module

If a module reaches OP but its input bytes never change -- typically frozen
garbage, or all zeros with the domain stuck at `WorkingCounter 2/3` -- check
the coupler's EEPROM before suspecting the driver or the wiring:

```
ethercat slaves -v | grep -A6 'CoE details'
```

Both of these must say `yes`:

```
    Enable PDO Assign: yes
    Enable PDO Configuration: yes
```

The IgH master decides whether it may configure a slave's PDOs from the
**slave's own SII (EEPROM)**, not from the ESI file.  If the EEPROM says no,
the master logs

```
Slave does not support assigning PDOs!
Slave does not support changing the PDO mapping!
```

(visible in `dmesg`, which may need `sysctl kernel.dmesg_restrict=0`) and
leaves the slave running whatever default mapping its EEPROM contains.

Couplers have been seen in the field shipping with unflashed sample EEPROM
content -- sync manager 3 defaulting to 7 bytes, PDOs named "Switch 1-8",
"LED 1-8", "Motor Outputs", and `0x1A03` mapped to a nonexistent `0x6030`.
Under that mapping `0x1A00` is eight 1-bit entries, which happens to be exactly
right for a digital input module, so digital I/O works and masks the problem.
An encoder needs `0x6000:01` as a 32-bit value and can never be served by a
1-bit mapping, so it produces nothing.  No driver change can work around this;
the master will not configure PDOs on a slave whose SII forbids it.

Vendor tooling (TwinCAT, Leadsys Studio) configures from the ESI file, which
declares `PdoAssign="true" PdoConfig="true"`, so the same hardware behaves
there.  That difference is the tell.

### Fixing the EEPROM

This cannot be done from the driver: the `ecrt` application API that
linuxcnc-ethercat uses exposes no SII access at all.  Writing the EEPROM is
only reachable through the `ethercat` command-line tool.

Back up first, always:

```
ethercat sii_read -p0 > sii_backup.bin
```

The flags live in the SII General category (type 30) at its byte 5, the
`CoEDetails` bitfield: bit0 EnableSDO, bit1 EnableSDOInfo, bit2
EnablePDOAssign, bit3 EnablePDOConfiguration, bit4 EnableUploadAtStartup, bit5
EnableSDOCompleteAccess.  A coupler shipped with `0x23` has assign and config
clear; `0x2F` sets them.  Locate the byte by walking the category list rather
than assuming an offset -- it is not at a fixed address across revisions.

```
ethercat states -p0 INIT
ethercat sii_write -p0 sii_patched.bin
ethercat rescan -p0
ethercat slaves -v | grep -A6 'CoE details'
```

Replacing the whole SII with one generated from the vendor ESI is the cleaner
long-term fix, since it also corrects the bogus PDO and sync-manager
categories; patching the one byte is the minimal change that restores
function.
