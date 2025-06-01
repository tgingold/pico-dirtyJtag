#include "pico/stdlib.h"
#include <stdlib.h>
#include "gdb.h"
#include "tusb.h"

/* State for rx packet decoder */
enum {
    S_RX_INIT,
    S_RX_DATA, /* Packet body */
    S_RX_ESC,  /* Escape character */
    S_RX_CHK1, /* First checksum character */
    S_RX_CHK2, /* Second checksum character */
};


#define GDB_BUFFER_SIZE 2048

/* Registers for cortex M0 */
#define NREGS 17
#define REG_R0 0
#define REG_SP 13
#define REG_LR 14
#define REG_PC 15
#define REG_XPSR 16

struct gdb_state {
    unsigned char itf;
    unsigned char rx_state;
    unsigned char buf[GDB_BUFFER_SIZE];
    unsigned char checksum;
    unsigned char rx_checksum;
    unsigned char connected;
    unsigned int rxlen;
    unsigned int txlen;
    uint32_t regs[NREGS];
};

static const char hexa[] = "0123456789abcdef";

/* Decode an hexa digit, return -1 on error */
static int
hex_digit(char ch)
{
    if (ch >= '0' && ch <= '9')
	return ch - '0';
    if (ch >= 'a' && ch <= 'f')
	return ch - 'a'+ 10;
    if (ch >= 'A' && ch <= 'F')
	return ch - 'A' + 10;
    return -1;
}

/* Decode 2 hex digits (one byte).  Return -1 on error */
static int
hex_byte(const char *p)
{
    int res;
    int v;

    v = hex_digit(p[0]);
    if (v == -1)
        return -1;
    res = v << 4;
    v = hex_digit(p[1]);
    if (v == -1)
        return -1;
    res |= v;
    return res;
}

static char *
get_two_hex_numbers(char *p, char sepch, uint32_t *one, uint32_t *two)
{
    char *sep;
    char *end;

    *one = strtoul(p, &sep, 16);
    if (*sep != sepch)
        return NULL;
    *two = strtoul(sep + 1, &end, 16);
    return end;
}

/* Receive a packet.
   < 0: error
   0: need more characters
   1: got a packet
*/
static int gdb_rx(struct gdb_state *s, char ch)
{
    int digit;

    switch (s->rx_state) {
    case S_RX_INIT:
	s->rxlen = 0;
	s->checksum = 0;
	if (ch == '$') {
	    s->rx_state = S_RX_DATA;
	    break;
	}
	return -1;

    case S_RX_DATA:
	if (ch == '#') {
	    s->rx_state = S_RX_CHK1;
	    break;
	}
	s->checksum += ch;
	if (ch == '}') {
	    s->rx_state = S_RX_ESC;
	    break;
	}
	s->buf[s->rxlen++] = ch;
	break;

    case S_RX_ESC:
	s->checksum += ch;
	s->buf[s->rxlen++] = ch ^ 0x20;
	s->rx_state = S_RX_DATA;
	break;

    case S_RX_CHK1:
	/* End the buffer with a NUL */
	s->buf[s->rxlen++] = 0;
	digit = hex_digit(ch);
	if (digit == -1) {
	    s->rx_state = S_RX_INIT;
	    return -1;
	}
	s->rx_checksum = (digit << 4);
	s->rx_state = S_RX_CHK2;
	break;

    case S_RX_CHK2:
	digit = hex_digit(ch);
	if (digit == -1) {
	    s->rx_state = S_RX_INIT;
	    return -1;
	}
	s->rx_checksum |= digit;
	if (s->rx_checksum != s->checksum) {
	    s->rx_state = S_RX_INIT;
	    return -1;
	}
	s->rx_state = S_RX_INIT;
	return 1;
    }

    if (s->rxlen == GDB_BUFFER_SIZE) {
	s->rx_state = S_RX_INIT;
	return -1;
    }
    else
	return 0;
}

/* Build a reply: init + text/hex2 + finish */
static void gdb_tx_flush(struct gdb_state *s)
{
    tud_cdc_n_write(s->itf, s->buf, s->txlen);
    tud_cdc_n_write_flush(s->itf);
    s->txlen = 0;
}

static void reply_init(struct gdb_state *s)
{
    s->buf[s->txlen++] = '$';
    s->checksum = 0;
}

static void reply_text(struct gdb_state *s, const char *text)
{
    while (*text) {
	s->checksum += *text;
	s->buf[s->txlen++] = *text++;
    }
}

static void reply_hex2(struct gdb_state *s, uint8_t v)
{
    char msg[3];
    msg[0] = hexa[(v >> 4) & 0x0f];
    msg[1] = hexa[v & 0x0f];
    msg[2] = 0;
    reply_text(s, msg);
}

static void reply_finish(struct gdb_state *s)
{
    s->buf[s->txlen++] = '#';
    s->buf[s->txlen++] = hexa[s->checksum >> 4];
    s->buf[s->txlen++] = hexa[s->checksum & 0x0f];

    gdb_tx_flush(s);
}

static void reply(struct gdb_state *s, const char *text)
{
    reply_init(s);
    reply_text(s, text);
    reply_finish(s);
}

static void reply_null(struct gdb_state *s)
{
    reply(s, "");
}

static void reply_ok(struct gdb_state *s)
{
    reply(s, "OK");
}

/* Read registers */
static void cmd_g(struct gdb_state *s)
{
    reply_init(s);

    for (int i = 0; i < NREGS; i++) {
        uint32_t rval;
        int rc;

	rval = s->regs[i];

	/* Put reg (LE) */
	reply_hex2(s, rval);
	reply_hex2(s, rval >> 8);
	reply_hex2(s, rval >> 16);
	reply_hex2(s, rval >> 24);
    }
    reply_finish(s);
}

/* Write registers */
static void cmd_G(struct gdb_state *s)
{
    uint32_t addr, length;
    int rc;
    char *p = s->buf + 1;

    for (unsigned i = 0; i < NREGS; i++) {
	uint32_t v;

	v = 0;
	for (unsigned j = 0; j < 4; j++) {
	    int d = hex_byte(p);
	    if (d < 0) {
		reply_null(s);
		break;
	    }
	    v = (v >> 8) | (d << 24);
	    p += 2;
	}
	s->regs[i] = v;
    }

    if (*p)
	reply_null(s);
    else
	reply_ok(s);
}

/* Continue -> execute */
static void cmd_c(struct gdb_state *s)
{
    unsigned (*fn)(void);
    unsigned res;

    gdb_tx_flush(s);

    stdio_puts("continue");
    stdio_printf("run PC=%08x\n", s->regs[REG_PC]);

    /* Use thumb mode */
    fn = (unsigned (*)(void))(s->regs[REG_PC] | 1);

    /* Overkill as cortex-M0 has no cache */
    __dsb();
    __isb();

    res = fn();
    stdio_printf("result: 0x%08x\n", res);

    reply(s, "S00");
}

/* Read memory */
static void cmd_m(struct gdb_state *s)
{
    uint8_t small_buf[10];
    uint32_t addr, len;
    volatile unsigned char *p;
    int rc;

    if (!get_two_hex_numbers(s->buf + 1, ',', &addr, &len)) {
        reply_null(s);
        return;
    }

    reply_init(s);

    p = (volatile unsigned char *)addr;
    while (len > 0) {
	reply_hex2(s, *p++);
	len--;
    }

    reply_finish(s);
}

/* Write memory */
static void cmd_M(struct gdb_state *s)
{
    uint32_t addr, length;
    int rc;
    char *p = get_two_hex_numbers(s->buf + 1, ',', &addr, &length);

    if (!p || *p != ':' || !length) {
        reply_null(s);
        return;
    }

    p++;

    volatile unsigned char *ptr = (volatile unsigned char *)addr;

    for (; length > 0; length--) {
	*ptr++ = hex_byte(p);
	p += 2;
    }
    reply_ok(s);
}

static void process_packet(struct gdb_state *s)
{
    char cmd = s->buf[0];

    s->txlen = 0;

    // Ack
    s->buf[s->txlen++] = '+';

    switch(cmd) {
    case 'g':
	cmd_g(s);
	return;
    case 'G':
	cmd_G(s);
	return;
    case 'c':
	cmd_c(s);
	return;
    case 'm':
	cmd_m(s);
	return;
    case 'M':
	cmd_M(s);
	return;

    case '?':
	reply(s, "S00");
	return;

    default:
	reply_null(s);
	return;
    }
}

static void gdb_poll(struct gdb_state *s)
{
    int c;

    if (tud_cdc_n_connected(s->itf)) {
	if (!s->connected) {
	    stdio_puts("gdb connection");
	    s->connected = 1;
	    s->rx_state = S_RX_INIT;
	    /* TODO: init */
	}

	while (1) {
	    char c;

	    if (tud_cdc_n_read(s->itf, &c, 1) == 0)
		break;

	    if (gdb_rx(s, c) > 0)
		process_packet(s);
	}
    }
    else {
	if (s->connected) {
	    stdio_puts("gdb disconnect");
	    s->connected = 0;
	}
    }
}

static struct gdb_state state;

void gdb_task(void)
{
    gdb_poll(&state);
}

void gdb_init(unsigned itf)
{
    state.itf = itf;
    state.connected = 0;
    state.rx_state = S_RX_INIT;
}
