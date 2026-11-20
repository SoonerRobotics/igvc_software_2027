/**
 * console.c - UART debug console: printf output and a typed command menu.
 *
 * Output: printf() -> syscalls.c _write() -> __io_putchar() below -> TX ring
 *         buffer -> UART TX interrupt sends it in the background.
 * Input:  UART interrupt puts each received byte into an RX ring buffer;
 *         console_poll() (main loop) takes bytes out, echoes them, builds a
 *         line, and runs the matching command when Enter arrives.
 *
 * Each ring buffer has exactly one writer and one reader (main loop and
 * interrupt), so it needs no locks: each side only moves its own index.
 * Decisions: D-019, D-022, D-023.
 */
#include "app/console.h"
#include "app/hub_config.h"
#include "app/hub_clock.h"   /* crystal state for "status" (hub_clock.c must be built too) */

#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * Receive ring buffer (filled by the interrupt, emptied by console_poll)
 * ------------------------------------------------------------------------- */
#define RX_BUF_SIZE  128u                 /* must be a power of 2 */
#define RX_BUF_MASK  (RX_BUF_SIZE - 1u)   /* index & MASK = index % SIZE, but faster */

/* head and tail count up forever (they wrap at 2^32, which is fine).
 * Bytes waiting = head - tail. Slot to use = index & RX_BUF_MASK.
 * head is written only by the interrupt, tail only by console_poll().
 * "volatile" tells the compiler the other side may change them at any time,
 * so it must re-read them from RAM every time. */
static volatile uint8_t  s_rx_buf[RX_BUF_SIZE];
static volatile uint32_t s_rx_head;       /* next slot the interrupt writes */
static volatile uint32_t s_rx_tail;       /* next slot console_poll reads   */
static volatile uint32_t s_rx_dropped;    /* bytes lost because the buffer was full */
static volatile uint32_t s_rx_errors;     /* UART errors seen (overrun, framing, noise) */
static volatile uint32_t s_rx_restarts;   /* times console_poll had to restart reception */

static uint8_t s_rx_byte;                 /* HAL drops each received byte here */

/* ---------------------------------------------------------------------------
 * Console state (main-loop only)
 * ------------------------------------------------------------------------- */
static UART_HandleTypeDef  *s_huart;                    /* NULL until console_init */
static char                 s_line[CONSOLE_LINE_MAX + 1]; /* +1 for the '\0' */
static uint32_t             s_len;                      /* characters in s_line */
static uint8_t              s_line_too_long;            /* user typed past the limit */
static uint8_t              s_prev_cr;                  /* last char was '\r' */
static const console_cmd_t *s_cmds[CONSOLE_MAX_CMDS];
static uint32_t             s_ncmds;

static void cmd_help(int argc, char *argv[]);
static void cmd_status(int argc, char *argv[]);

static const console_cmd_t s_cmd_help   = { "help",   cmd_help,   "list all commands" };
static const console_cmd_t s_cmd_status = { "status", cmd_status, "uptime, firmware version, UART counters" };

/* ---------------------------------------------------------------------------
 * Transmit ring buffer (filled by printf in the main loop, emptied by the
 * UART TX interrupt). D-023.
 *
 * head is written only by the main loop (__io_putchar), tail only by the
 * interrupt (HAL_UART_TxCpltCallback). One "chunk" = the bytes from tail up
 * to head, the end of the array, or TX_CHUNK_MAX bytes, whichever comes
 * first, handed to HAL_UART_Transmit_IT in one go.
 * ------------------------------------------------------------------------- */
#define TX_BUF_SIZE     512u              /* must be a power of 2 */
#define TX_BUF_MASK     (TX_BUF_SIZE - 1u)
#define TX_FULL_WAIT_MS 20u               /* give up after this long with NO progress */
#define TX_CHUNK_MAX    64u               /* bytes per transfer: 64 x 87 us = ~5.6 ms, well under 20 ms */

static volatile uint8_t  s_tx_buf[TX_BUF_SIZE];
static volatile uint32_t s_tx_head;       /* next slot printf writes            */
static volatile uint32_t s_tx_tail;       /* first byte not yet sent            */
static volatile uint32_t s_tx_chunk;      /* bytes in the transfer now running  */
static volatile uint8_t  s_tx_busy;       /* 1 while a transfer is running      */
static volatile uint32_t s_tx_dropped;    /* characters lost (UART stuck, or printf from an interrupt) */
static uint8_t           s_tx_stuck;      /* last wait timed out...             */
static uint32_t          s_tx_stuck_tail; /* ...with tail here                  */

/* 1 if running inside an interrupt, or with interrupts switched off.
 * Waiting there would never end: the TX interrupt cannot run and
 * HAL_GetTick() does not advance. */
static int in_irq_or_masked(void)
{
  return (__get_IPSR() != 0u) || (__get_PRIMASK() != 0u);
}

/* Start sending the next chunk if nothing is being sent.
 * Must run with interrupts off (main loop) or inside the UART interrupt,
 * so the main loop and the interrupt can never both start a transfer. */
static void tx_start_locked(void)
{
  if ((s_huart == NULL) || s_tx_busy)
  {
    return;
  }
  uint32_t tail = s_tx_tail;
  uint32_t len  = s_tx_head - tail;
  if (len == 0u)
  {
    return;                              /* nothing to send */
  }
  uint32_t idx = tail & TX_BUF_MASK;
  if (len > (TX_BUF_SIZE - idx))
  {
    len = TX_BUF_SIZE - idx;             /* stop at the end of the array; the rest is the next chunk */
  }
  if (len > TX_CHUNK_MAX)
  {
    len = TX_CHUNK_MAX;                  /* short chunks: tail moves at least every ~5.6 ms */
  }
  if (HAL_UART_Transmit_IT(s_huart, (const uint8_t *)&s_tx_buf[idx], (uint16_t)len) == HAL_OK)
  {
    s_tx_chunk = len;
    s_tx_busy  = 1u;
  }
  /* If the HAL said busy, the next tx_kick() tries again. */
}

/* Main-loop version of tx_start_locked(): interrupts off for a few
 * microseconds (far less than one received byte, 87 us). */
static void tx_kick(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  tx_start_locked();
  __set_PRIMASK(primask);
}

/* Count a lost character. Done with interrupts off, so a count from the
 * main loop and one from an interrupt cannot overwrite each other. */
static void tx_count_drop(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_tx_dropped++;
  __set_PRIMASK(primask);
}

/* Put one byte into the TX buffer (main loop only). */
static void tx_put(uint8_t c)
{
  if (in_irq_or_masked())
  {
    /* printf from an interrupt (or with interrupts off) is not allowed, see
     * console.h. Writing here would make a second writer of head and could
     * corrupt the buffer, so the character is dropped and counted instead. */
    tx_count_drop();
    return;
  }

  if ((s_tx_head - s_tx_tail) >= TX_BUF_SIZE)
  {
    /* Buffer full. Before console_init nothing drains it, so drop. If the
     * last wait already timed out and nothing has been sent since, the UART
     * is stuck: drop at once instead of waiting 20 ms per character. */
    if ((s_huart == NULL) || (s_tx_stuck && (s_tx_tail == s_tx_stuck_tail)))
    {
      tx_count_drop();
      return;
    }
    /* Wait for space. The 20 ms limit means "nothing sent for 20 ms": the
     * timer restarts every time the interrupt frees bytes (tail moves). */
    uint32_t start     = HAL_GetTick();
    uint32_t last_tail = s_tx_tail;
    while ((s_tx_head - s_tx_tail) >= TX_BUF_SIZE)
    {
      tx_kick();                         /* in case no transfer is running */
      if (s_tx_tail != last_tail)
      {
        last_tail = s_tx_tail;           /* progress: restart the timeout */
        start     = HAL_GetTick();
      }
      if ((HAL_GetTick() - start) >= TX_FULL_WAIT_MS)
      {
        s_tx_stuck      = 1u;
        s_tx_stuck_tail = s_tx_tail;
        tx_count_drop();
        return;
      }
    }
  }
  s_tx_stuck = 0u;

  uint32_t head = s_tx_head;
  s_tx_buf[head & TX_BUF_MASK] = c;
  s_tx_head = head + 1u;                 /* publish the byte AFTER storing it */
}

/* ---------------------------------------------------------------------------
 * printf output: syscalls.c calls this once per character.
 * It replaces the empty "weak" version in syscalls.c. It only copies into
 * the TX buffer and returns; the UART interrupt does the sending.
 * Text printed before console_init() is kept and sent once it starts.
 * ------------------------------------------------------------------------- */
int __io_putchar(int ch)
{
  if (ch == '\n')
  {
    tx_put('\r');                        /* terminals need "\r\n" for a new line */
  }
  tx_put((uint8_t)ch);
  if (!in_irq_or_masked())
  {
    tx_kick();
  }
  return ch;
}

/* Called by the HAL inside the UART interrupt when a chunk has been sent.
 * Free those bytes and start the next chunk, if any. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((s_huart == NULL) || (huart != s_huart))
  {
    return;                              /* another UART: not ours */
  }
  s_tx_tail  = s_tx_tail + s_tx_chunk;
  s_tx_chunk = 0u;
  s_tx_busy  = 0u;
  tx_start_locked();                     /* already inside the interrupt */
}

void console_flush(uint32_t timeout_ms)
{
  if ((s_huart == NULL) || in_irq_or_masked())
  {
    return;                              /* cannot wait here, see in_irq_or_masked() */
  }
  uint32_t start = HAL_GetTick();
  while ((s_tx_head != s_tx_tail) || s_tx_busy)
  {
    tx_kick();
    if ((HAL_GetTick() - start) >= timeout_ms)
    {
      return;
    }
  }
}

/* ---------------------------------------------------------------------------
 * Interrupt side
 * ------------------------------------------------------------------------- */

/* Ask the HAL for the next single byte. When it arrives, the HAL calls
 * HAL_UART_RxCpltCallback(). Returns HAL_BUSY if reception is already running. */
static HAL_StatusTypeDef rx_arm(void)
{
  return HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

/* Called by the HAL inside the UART interrupt when one byte has arrived.
 * Keep it short: copy the byte into the ring buffer and re-arm. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* This function is global: every UART on the chip ends up here.
   * Only handle the console UART; a future UART would get its own branch. */
  if ((s_huart == NULL) || (huart != s_huart))
  {
    return;
  }

  uint32_t head = s_rx_head;
  if ((head - s_rx_tail) < RX_BUF_SIZE)
  {
    s_rx_buf[head & RX_BUF_MASK] = s_rx_byte;
    s_rx_head = head + 1u;               /* publish the byte AFTER storing it */
  }
  else
  {
    s_rx_dropped++;                      /* buffer full: drop the byte, count it */
  }

  (void)rx_arm();                        /* wait for the next byte */
}

/* Called by the HAL inside the UART interrupt on a receive error
 * (overrun = a byte arrived before the last one was read; framing; noise).
 * In this HAL the waiting byte is delivered first: HAL_UART_RxCpltCallback
 * stores it and re-arms, so reception is already running again when this
 * runs, and rx_arm() just returns HAL_BUSY, which is harmless.
 * Rare case: an overrun with no byte waiting makes the HAL stop reception;
 * then rx_arm() here restarts it. Either way the console keeps listening. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((s_huart == NULL) || (huart != s_huart))
  {
    return;
  }
  s_rx_errors++;
  (void)rx_arm();
}

/* ---------------------------------------------------------------------------
 * Main-loop side
 * ------------------------------------------------------------------------- */

/* Take one byte out of the ring buffer. Returns 1 if a byte was there. */
static int rx_pop(uint8_t *c)
{
  uint32_t tail = s_rx_tail;
  if (s_rx_head == tail)
  {
    return 0;                            /* empty */
  }
  *c = s_rx_buf[tail & RX_BUF_MASK];
  s_rx_tail = tail + 1u;                 /* free the slot AFTER reading it */
  return 1;
}

static void print_prompt(void)
{
  printf("> ");
}

/* Split the line into words, find the command, run it. */
static void run_line(void)
{
  char *argv[CONSOLE_MAX_ARGS];
  int   argc = 0;
  char *p    = s_line;

  /* Cut the line at spaces: each word becomes its own '\0'-ended string. */
  while (*p != '\0')
  {
    while ((*p == ' ') || (*p == '\t'))  /* space or tab separates words */
    {
      *p++ = '\0';
    }
    if (*p == '\0')
    {
      break;
    }
    if (argc == CONSOLE_MAX_ARGS)
    {
      printf("note: only the first %d words are used\n", CONSOLE_MAX_ARGS);
      break;
    }
    argv[argc++] = p;
    while ((*p != ' ') && (*p != '\t') && (*p != '\0'))
    {
      p++;
    }
  }

  if (argc == 0)
  {
    return;                              /* empty line: just a new prompt */
  }

  for (uint32_t i = 0; i < s_ncmds; i++)
  {
    if (strcmp(argv[0], s_cmds[i]->name) == 0)
    {
      s_cmds[i]->handler(argc, argv);
      return;
    }
  }
  printf("unknown command '%s', type help\n", argv[0]);
}

/* Handle one received character: edit the line or run it. */
static void handle_char(uint8_t c)
{
  /* Enter can arrive as "\r", "\n" or "\r\n". Treat each as ONE Enter:
   * a '\n' right after a '\r' is the second half of the same Enter. */
  if ((c == '\n') && s_prev_cr)
  {
    s_prev_cr = 0;
    return;
  }
  s_prev_cr = (c == '\r');

  if ((c == '\r') || (c == '\n'))
  {
    printf("\n");
    s_line[s_len] = '\0';
    if (s_line_too_long)
    {
      /* Characters were dropped, so the line is not what was typed:
       * never run a cut-off command. */
      printf("line too long (max %d), ignored\n", CONSOLE_LINE_MAX);
    }
    else
    {
      run_line();
    }
    s_len = 0;
    s_line_too_long = 0;
    print_prompt();
    return;
  }

  if ((c == '\b') || (c == 0x7F))        /* Backspace (terminals send either) */
  {
    if (s_len > 0u)
    {
      s_len--;
      printf("\b \b");                   /* back, overwrite with space, back */
    }
    return;
  }

  if (c == '\t')
  {
    c = ' ';                             /* Tab counts as a space between words */
  }

  if ((c < 0x20u) || (c > 0x7Eu))        /* ignore other control/non-text bytes */
  {
    return;
  }

  if (s_len < CONSOLE_LINE_MAX)
  {
    s_line[s_len++] = (char)c;
    putchar(c);                          /* echo, so you see what you type */
  }
  else
  {
    /* Extra characters are dropped and the whole line is rejected at Enter.
     * Backspacing does not undo this on purpose: the dropped characters are
     * gone, so the line can no longer be what the user meant. */
    s_line_too_long = 1;
  }
}

void console_poll(void)
{
  if (s_huart == NULL)
  {
    return;
  }

  /* Safety net: if reception ever stopped (re-arm failed inside the
   * interrupt), start it again so the console never goes deaf. */
  if (s_huart->RxState == HAL_UART_STATE_READY)
  {
    if (rx_arm() == HAL_OK)
    {
      s_rx_restarts++;
    }
  }

  /* Handle everything received so far; returns as soon as the buffer is empty. */
  uint8_t c;
  while (rx_pop(&c))
  {
    handle_char(c);
  }
}

int console_register(const console_cmd_t *cmd)
{
  if ((cmd == NULL) || (cmd->name == NULL) || (cmd->handler == NULL) ||
      (cmd->help == NULL) || (s_ncmds >= CONSOLE_MAX_CMDS))
  {
    return -1;
  }
  for (uint32_t i = 0; i < s_ncmds; i++)
  {
    if (strcmp(cmd->name, s_cmds[i]->name) == 0)
    {
      return -1;                         /* name already taken */
    }
  }
  s_cmds[s_ncmds++] = cmd;
  return 0;
}

void console_init(UART_HandleTypeDef *huart)
{
  /* newlib-nano holds printf text until a '\n'. Turn that off so the
   * "> " prompt and echoed characters show up immediately. */
  setvbuf(stdout, NULL, _IONBF, 0);

  s_huart = huart;
  s_len = 0;
  s_line_too_long = 0;
  s_prev_cr = 0;

  (void)console_register(&s_cmd_help);
  (void)console_register(&s_cmd_status);

  (void)rx_arm();                        /* start interrupt reception */
  tx_kick();                             /* send any text printed before init */

  printf("\nSCR Hub FW %s - type help\n", HUB_FW_VERSION);
  print_prompt();
}

/* ---------------------------------------------------------------------------
 * Built-in commands
 * ------------------------------------------------------------------------- */
static void cmd_help(int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  for (uint32_t i = 0; i < s_ncmds; i++)
  {
    printf("  %-8s %s\n", s_cmds[i]->name, s_cmds[i]->help);
  }
}

static void cmd_status(int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  printf("FW %s, uptime %lu ms\n", HUB_FW_VERSION, (unsigned long)HAL_GetTick());
  printf("UART rx: dropped %lu, errors %lu, restarts %lu\n",
         (unsigned long)s_rx_dropped, (unsigned long)s_rx_errors,
         (unsigned long)s_rx_restarts);
  printf("UART tx: dropped %lu\n", (unsigned long)s_tx_dropped);
  printf("%s\n", hub_clock_hse_ok() ? "crystal OK" : "crystal FAIL (CAN off)");
}
