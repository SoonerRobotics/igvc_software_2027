/**
 * console.h - UART debug console: printf output and a typed command menu.
 *
 * Works on any STM32 UART: the Hub passes &huart1, the NUCLEO-C092RC test
 * project passes &huart2 (ST-LINK virtual COM port).
 *
 * Needs the UART's "global interrupt" enabled in CubeMX (NVIC Settings),
 * because received AND sent characters are moved by the interrupt
 * (D-022, D-023). printf only copies into a 512-byte buffer and returns.
 *
 * RULE: never call printf (or any console output) from an interrupt or a
 * HAL callback, or with interrupts switched off, because:
 *  - the console drops that text on purpose (counted as "tx dropped" in
 *    status): writing the buffer from two places at once could corrupt it;
 *  - newlib printf is not re-entrant (unsafe if it interrupts a printf);
 *  - formatting text keeps the interrupt busy, which delays CAN and UART
 *    receive at the same priority (FDCAN1 and USART1 are both priority 0).
 * Set a flag or counter instead and print it from the main loop.
 */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>
#include "main.h"   /* brings in the HAL and UART_HandleTypeDef */

/* Longest command line (characters typed before Enter). */
#define CONSOLE_LINE_MAX   64
/* How many commands can be registered in total. */
#define CONSOLE_MAX_CMDS   16
/* Most words on one line, including the command name ("ina 3" = 2 words). */
#define CONSOLE_MAX_ARGS   4

/* A command handler gets the typed words, like main() in a PC program:
 * argv[0] is the command name, argv[1].. are the extra words. */
typedef void (*console_handler_t)(int argc, char *argv[]);

/* One menu entry. Keep these as "static const" in the module that owns them:
 * the console stores only a pointer, not a copy. */
typedef struct
{
  const char        *name;     /* what you type, e.g. "scan" */
  console_handler_t  handler;  /* function that runs */
  const char        *help;     /* one line shown by "help" */
} console_cmd_t;

/* Start the console on this UART: starts interrupt reception, registers
 * "help" and "status", prints the boot banner and the "> " prompt. */
void console_init(UART_HandleTypeDef *huart);

/* Call every main-loop pass. Never waits: handles the characters already
 * received, echoes them, and runs the command when Enter is pressed. */
void console_poll(void);

/* Wait (main loop only) until all buffered printf text has left the UART,
 * or timeout_ms has passed. printf itself never waits for the wire: it only
 * copies into a 512-byte buffer. Call this before a deliberate reset or
 * hang, or the last lines would never be sent. */
void console_flush(uint32_t timeout_ms);

/* Add a command to the menu. Returns 0 on success, -1 if the table is full
 * or the name is already used. */
int console_register(const console_cmd_t *cmd);

#endif /* CONSOLE_H */
