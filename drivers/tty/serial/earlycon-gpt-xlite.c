/*
 * Copyright (C) 2018, General Processor Techologies Inc.
 * Author: scxie <scxie@hxgpt.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <asm/io.h>

#define ULITE_RX                0x00
#define ULITE_TX                0x04
#define ULITE_STATUS            0x08
#define ULITE_CONTROL           0x0c

#define ULITE_STATUS_RXVALID    0x01
#define ULITE_STATUS_TXFULL     0x08

#define CONTROL_RST_TX    (0x1<<0)
#define CONTROL_RST_RX    (0x1<<1)
#define CONTROL_INT_EN    (0x1<<4)

void gpt_xlite_putc(struct uart_port *port, int c)
{
	volatile uint32_t reg = readl(port->membase + ULITE_STATUS);
	while (reg & ULITE_STATUS_TXFULL) /* spin on TXFULL bit */
		reg = readl(port->membase + ULITE_STATUS);

	writel(c, port->membase + ULITE_TX);
}

static void gpt_xlite_write(struct console *con, const char *s, unsigned n)
{
	struct earlycon_device *dev = con->data;
	uart_console_write(&dev->port, s, n, gpt_xlite_putc);	
}

int __init early_gpt_xlite_setup(struct earlycon_device *device, const char *opt)
{
	struct uart_port *port = &device->port;	
	
        if (!port->membase)
                return -ENODEV;
	
	device->con->write = gpt_xlite_write;

	return 0;
}

EARLYCON_DECLARE(gptxlite, early_gpt_xlite_setup);
