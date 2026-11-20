/**
 * hub_config.h - Fixed settings for the SCR Hub firmware (constants only).
 *
 * Everything that describes THIS board (ports, chip addresses, shunt value,
 * timing) lives here, so the driver files never contain "magic numbers".
 * Decisions: D-019 (file structure), D-022 (this module).
 */
#ifndef HUB_CONFIG_H
#define HUB_CONFIG_H

#include <stdint.h>   /* uint8_t, used by the build-time checks below */

/* ---- Firmware version (printed at boot and by the "status" command) ---- */
#define HUB_FW_VERSION        "0.1.0"

/* ---- Power ports ---------------------------------------------------------
 * 7 switched 12 V output ports, J4..J10. Index 0..6 is used everywhere:
 * index i in HUB_PORT_NAMES and index i in HUB_INA220_ADDRS are the same port.
 * Use them to fill arrays in a .c file, for example:
 *     static const uint8_t addr[HUB_NUM_PORTS] = HUB_INA220_ADDRS;
 */
#define HUB_NUM_PORTS         7

#define HUB_PORT_NAMES        { "J4", "J5", "J6", "J7", "J8", "J9", "J10" }

/* INA220 I2C addresses, 7-bit form (as in the datasheet).
 * Read from Current Sensor.SchDoc address straps (A1, A0) on 2026-11-19:
 *   J4  U6  GND,GND -> 0x40     J8  U10 SDA,SDA -> 0x4A
 *   J5  U7  GND,SDA -> 0x42     J9  U11 SDA,SCL -> 0x4B
 *   J6  U8  GND,SCL -> 0x43     J10 U12 SCL,GND -> 0x4C
 *   J7  U9  SDA,GND -> 0x48
 * Note: ST HAL I2C functions want the address shifted left by 1 (addr << 1). */
#define HUB_INA220_ADDRS      { 0x40, 0x42, 0x43, 0x48, 0x4A, 0x4B, 0x4C }

/* Which ports are fitted on this board: bit 0 = J4 ... bit 6 = J10.
 * 0x7F = all 7 in use. Example: J10 not populated -> 0x3F.
 * A disabled port is skipped (never read) and reported UNUSED, not FAULT.
 * The lists above always keep 7 entries; do NOT shrink HUB_NUM_PORTS,
 * or the index -> J4..J10 mapping would shift (D-023). */
#define HUB_PORT_ENABLED_MASK 0x7Fu

/* Build-time checks (D-023): the build stops with a clear message if a list
 * does not have exactly HUB_NUM_PORTS entries, or the mask names a port that
 * does not exist. They cost no flash or RAM. */
_Static_assert(sizeof((const uint8_t[])HUB_INA220_ADDRS) == HUB_NUM_PORTS,
               "HUB_INA220_ADDRS must have HUB_NUM_PORTS entries");
_Static_assert(sizeof((const char *[])HUB_PORT_NAMES) / sizeof(const char *) == HUB_NUM_PORTS,
               "HUB_PORT_NAMES must have HUB_NUM_PORTS entries");
_Static_assert((HUB_PORT_ENABLED_MASK >> HUB_NUM_PORTS) == 0u,
               "HUB_PORT_ENABLED_MASK has a bit above the last port");

/* Current-sense resistor on every port (R15..R21, 20 milliohm, high side). */
#define HUB_SHUNT_MILLIOHM    20

/* ---- Timing ---- */
/* Read all 7 INA220s once every 100 ms (one sweep). About 1.3 ms per chip
 * at 100 kHz, so the I2C bus is busy about 9% of the time (D-022). */
#define HUB_SWEEP_PERIOD_MS   100

/* Longest wait for any single I2C transfer before giving up (D-007). */
#define HUB_I2C_TIMEOUT_MS    10

/* ---- Test switches (must be 0 in normal firmware) ---- */
/* 1 = pretend the 8 MHz crystal did not start, to test the
 * "crystal FAIL - CAN disabled" path on a board whose crystal works (D-023). */
#define HUB_TEST_FORCE_HSE_FAIL  0

/* ---- CAN (values come from the team; never invent them) ----------------
 * These stay commented out on purpose: any code that uses them will not
 * compile until the team gives the real numbers. A placeholder 0 would be
 * dangerous because 0x000 is the E-Stop message ID.
 */
/* #define HUB_CAN_ID_PORT_DATA     TODO(team) */
/* #define HUB_CAN_SEND_PERIOD_MS   TODO(team) */

#endif /* HUB_CONFIG_H */
