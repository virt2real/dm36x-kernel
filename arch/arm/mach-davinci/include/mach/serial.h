/*
 * DaVinci serial device definitions
 *
 * Author: Kevin Hilman, MontaVista Software, Inc. <source@mvista.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#ifndef __ASM_ARCH_SERIAL_H
#define __ASM_ARCH_SERIAL_H

#include <mach/hardware.h>

#define DAVINCI_MAX_NR_UARTS	3
#define DAVINCI_UART0_BASE	(IO_PHYS + 0x20000)
#define DAVINCI_UART1_BASE	(IO_PHYS + 0x20400)
#define DAVINCI_UART2_BASE	(IO_PHYS + 0x20800)

#define DM36X_MAX_NR_UARTS      2
#define DM36X_UART1_BASE        (IO_PHYS + 0x106000)

#define DA8XX_UART0_BASE	(IO_PHYS + 0x042000)
#define DA8XX_UART1_BASE	(IO_PHYS + 0x10c000)
#define DA8XX_UART2_BASE	(IO_PHYS + 0x10d000)

/* DaVinci UART register offsets */
#define UART_DAVINCI_PWREMU		0x0c
#define UART_DM646X_SCR			0x10
#define UART_DM646X_SCR_TX_WATERMARK	0x08

struct davinci_uart_config {
	/* Bit field of UARTs present; bit 0 --> UART1 */
	unsigned int enabled_uarts;
};

extern int davinci_serial_init(struct davinci_uart_config *);

#endif /* __ASM_ARCH_SERIAL_H */
