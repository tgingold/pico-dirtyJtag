/*
 * This file is part of the Black Magic Debug project.
 *
 * Based on work that is Copyright (C) 2017 Black Sphere Technologies Ltd.
 * Copyright (C) 2017 Dave Marples <dave@marples.net>
 * Copyright (C) 2023 Patrick H Dussud
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.	 If not, see <http://www.gnu.org/licenses/>.
 */
#include "dirtyJtagConfig.h"

#if CDC_SOFT_UART_COUNT > 0

#include <pico/stdlib.h>
#include <pico/stdio/driver.h>
#include <pico/bootrom.h>
#include "tusb.h"
#include "cdc_soft_uart.h"

/* Length of a ring buffer.  Must be a power of 2. */
#define RING_LEN (1 << 11)
/* Bits to be used for indexing the buffer. */
#define RING_MASK (RING_LEN - 1)
/* Bits to be kept in the offsets.
   The offsets have one extra bit to distinguish between empty (when
   wptr == rptr) and full (when wptr == rptr + RING_LEN). */
#define RING_EXT_MASK (RING_LEN | (RING_LEN - 1))

struct ring_buf {
	unsigned char buf[RING_LEN];
	volatile unsigned wptr;
	volatile unsigned rptr;
};

struct ring_uart {
        /* TX: from pico to USB, RX: from USB to pico */
	struct ring_buf tx;
	struct ring_buf rx;
	unsigned char itf;
	unsigned char connected;
};

static struct ring_uart dbg_uart;

#if 0
static struct ring_uart echo_uart;
#endif

static void init_ring_buf(struct ring_buf *buf)
{
	buf->wptr = 0;
	buf->rptr = 0;
}

static int ring_buf_full(struct ring_buf *buf)
{
	return buf->wptr == (buf->rptr ^ RING_LEN);
}

static int ring_buf_empty(struct ring_buf *buf)
{
	return buf->rptr == buf->wptr;
}

static void init_ring_uart(struct ring_uart *uart, unsigned itf)
{
	uart->itf = itf;
	uart->connected = 0;
	init_ring_buf(&uart->tx);
	init_ring_buf(&uart->rx);
}

int ring_buf_putc(struct ring_buf *buf, char c)
{
	if (ring_buf_full(buf))
		return -1;
	buf->buf[buf->wptr & RING_MASK] = c;
	buf->wptr = (buf->wptr + 1) & RING_EXT_MASK;
	return 0;
}

int ring_buf_getc(struct ring_buf *buf)
{
	int res;
	
	if (ring_buf_empty(buf))
		return -1;
	res = buf->buf[buf->rptr & RING_MASK];
	buf->rptr = (buf->rptr + 1) & RING_EXT_MASK;
	return res;
}

int ring_uart_putc(struct ring_uart *uart, char c)
{
	return ring_buf_putc(&uart->tx, c);
}

int ring_uart_getc(struct ring_uart *uart)
{
	if (!uart->connected)
		return -2;
	return ring_buf_getc(&uart->rx);
}

static void cdc_soft_uart_ring_tx(struct ring_uart *uart)
{
	unsigned len = 0;
	int c;

	for (unsigned wa = tud_cdc_n_write_available(uart->itf); wa > 0; wa--) {
		int c = ring_buf_getc(&uart->tx);
		char b;
		if (c < 0)
			break;

		b = c;
		tud_cdc_n_write(uart->itf, &b, 1);
		len++;
	}
	if (len != 0)
		tud_cdc_n_write_flush(uart->itf);
}

static void cdc_soft_uart_ring_rx(struct ring_uart *uart)
{
	for (unsigned ra = tud_cdc_n_available(uart->itf);
	     ra > 0 && !ring_buf_full(&uart->rx); ra--) {
		char c;
		size_t len;
		len = tud_cdc_n_read(uart->itf, &c, 1);
		ring_buf_putc(&uart->rx, c);
	}
}


static void cdc_soft_uart_ring(struct ring_uart *uart)
{
	if (tud_cdc_n_connected(uart->itf))
	{
		uart->connected = 1;
		cdc_soft_uart_ring_tx(uart);
		cdc_soft_uart_ring_rx(uart);
	}
	else
	{
		/* Clear fifo */
		if (uart->connected) {
			tud_cdc_n_write_clear(uart->itf);
			uart->connected = 0;
		}
	}
}

static void cdc_soft_uart_echo(struct ring_uart *uart)
{
	while (1) {
		int c = ring_uart_getc(uart);
		if (c < 0)
			return;
		if (c > ' ')
			c ^= 0x20;
		ring_uart_putc(uart, c);
	}
}

static void cdc_out_func(const char *buf, int len)
{
    for (int i = 0; i < len; i++) {
	    char c = buf[i];
	    ring_uart_putc(&dbg_uart, c);
    }
}


static void cdc_out_flush(void)
{
	return;
}

static int cdc_in_chars(char *buf, int len)
{
	for (int res = 0; res < len; res++) {
		int c = ring_uart_getc(&dbg_uart);
		if (c < 0)
			return res;
		buf[res] = c;
	}
	return len;
}

#if 0
void echo_cdc_soft_init(void)
{
	init_ring_uart(&echo_uart, CDC_UART_INTF_COUNT);
}
#endif

static void cdc_set_chars_available_callback(void (*fn)(void*), void *param)
{
    return;
}


static stdio_driver_t stdio_cdc = {
    &cdc_out_func,
    &cdc_out_flush,
    &cdc_in_chars,
    &cdc_set_chars_available_callback,
    (stdio_driver_t *) 0,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    false,
    true,
#endif
};

void stdio_cdc_soft_init(void)
{
	init_ring_uart(&dbg_uart, CDC_UART_INTF_COUNT);
	stdio_set_driver_enabled(&stdio_cdc, true);
	stdio_filter_driver(&stdio_cdc);
}

void cdc_soft_uart_task(void)
{
#if CDC_UART_INTF_COUNT > 0
	cdc_soft_uart_ring(&dbg_uart);

	int c = ring_uart_getc(&dbg_uart);
	if (c == 'r')
		rom_reset_usb_boot(0,0);
	else if (c >= 0)
		stdio_puts("Hello");
#endif
}

#endif // CDC_UART_INTF_COUNT
