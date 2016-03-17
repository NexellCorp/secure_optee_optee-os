/*
 * Copyright (C) 2016  Nexell Co., Ltd.
 * Author: Bon-gyu, KOO <freestyle@nexell.co.kr>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <drivers/s5p6818_uart.h>
#include <drivers/s5p6818_tieoff.h>
#include <io.h>
#include <compiler.h>

/* NX_UART Registers */
#define UARTDLCON		0x00   /* Line Control */
#define UARTRUCON		0x04   /* Control */
#define UARTEFCON		0x08   /* FIFO Control */
#define UARTFMCON		0x0C   /* Modem Control */
#define UARTTRSTAT		0x10   /* Tx/Rx Status */
#define UARTERSTAT		0x14   /* Rx Error Status */
#define UARTFSTAT		0x18   /* FIFO Status */
#define UARTMSTAT		0x1C   /* Modem Status */
#define UARTTXH			0x20   /* Transmit Buffer */
#define UARTRXH			0x24   /* Receive Buffer */
#define UARTBRDR		0x28   /* Baud Rate Driver */
#define UARTFRACVAL		0x2C   /* Driver Fractional Value */
#define UARTINTP		0x30   /* Instrrupt Pending */
#define UARTINTSP		0x34   /* Instrrupt Source */
#define UARTINTM		0x38   /* Instrrupt Mask */

#define UART0_BASE		0xC00A1000
#define UART1_BASE		0xC00A0000
#define UART2_BASE		0xC00A2000
#define UART3_BASE		0xC00A3000
#define UART4_BASE		0xC006D000
#define UART5_BASE		0xC006F000

#define NUMBER_OF_RESET_MODULE_PIN      69
#define UART_RST_BASE	50

#define NX_UARTFSTAT_TXFF_BIT       (1 << 24)
			/* Transmit FIFO full bit in UARTFSTAT register */
#define NX_UARTFSTAT_RXFE_BIT       (1 << 2)
			/* Receive FIFO empty bit in UARTFR register */

#define NX_UARTTRSTAT_TX_EMPTY_BIT  (1 << 2)
			/* Transmit empty bit in UARTTRSTAT register */
#define NX_UARTTRSTAT_TXBE_BIT      (1 << 1)
			/* Transmit BUFFER empty bit in UARTTRSTAT register */

/* FIXME - Modify and use it after confirming the operation. */
#if 0
static int s5p6818_uart_get_ch_num(vaddr_t base)
{
	int ch = 0;

	switch (base) {
	case UART0_BASE:
		ch = 0;
		break;
	case UART1_BASE:
		ch = 1;
		break;
	case UART2_BASE:
		ch = 2;
		break;
	case UART3_BASE:
		ch = 3;
		break;
	case UART4_BASE:
		ch = 4;
		break;
	case UART5_BASE:
		ch = 5;
		break;
	default:
		break;
	};
	return ch;
}

/*
 * FIXME modify after clock control interface.
 */
static vaddr_t get_clock_base(uint32_t ch)
{
	vaddr_t base = 0;

	switch (ch) {
	case 0:
		base = 0xc00A9000;
		break;
	case 1:
		base = 0xc00A8000;
		break;
	case 2:
		base = 0xc00AA000;
		break;
	case 3:
		base = 0xc00AB000;
		break;
	case 4:
		base = 0xc00E0000;
		break;
	case 5:
		base = 0xc00B1000;
		break;
	default:
		break;
	};
	return base;
}

static void uart_clk_enb(uint32_t ch)
{
	vaddr_t base = get_clock_base(ch);

	write32(0x4, base);
	write32(0x68, base+0x4);
}

/* For Reset control */

struct  nx_rstcon_registerset {
	uint32_t     regrst[(NUMBER_OF_RESET_MODULE_PIN+31)>>5];
};

static struct nx_rstcon_registerset *nx_rstcon
= (struct nx_rstcon_registerset *)0xC0012000;

static void nx_rstcon_setrst(uint32_t rstindex, uint32_t status)
{
	uint32_t regnum, bitpos, curstat;

	regnum  = rstindex >> 5;
	curstat = (uint32_t)read32((vaddr_t)(&nx_rstcon->regrst[regnum]));
	bitpos  = rstindex & 0x1f;
	curstat &= ~(1UL << bitpos);
	curstat |= (status & 0x01) << bitpos;
	write32(curstat, (vaddr_t)(&nx_rstcon->regrst[regnum]));
}

/* tieoff setting */
static void uart_tieoff_set(uint32_t ch)
{
	switch (ch) {
	case 0:
		nx_tieoff_set(NX_TIEOFF_UART0_USESMC, 0);
		nx_tieoff_set(NX_TIEOFF_UART0_SMCTXENB, 0);
		nx_tieoff_set(NX_TIEOFF_UART0_SMCRXENB, 0);
		break;
	case 1:
		nx_tieoff_set(NX_TIEOFF_UART1_USESMC, 0);
		nx_tieoff_set(NX_TIEOFF_UART1_SMCTXENB, 0);
		nx_tieoff_set(NX_TIEOFF_UART1_SMCRXENB, 0);
		break;
	case 2:
		nx_tieoff_set(NX_TIEOFF_UART2_USESMC, 0);
		nx_tieoff_set(NX_TIEOFF_UART2_SMCTXENB, 0);
		nx_tieoff_set(NX_TIEOFF_UART2_SMCRXENB, 0);
		break;
	case 3:
		nx_tieoff_set(NX_TIEOFF_UART3_USESMC, 0);
		nx_tieoff_set(NX_TIEOFF_UART3_SMCTXENB, 0);
		nx_tieoff_set(NX_TIEOFF_UART3_SMCRXENB, 0);
		break;
	case 4:
		nx_tieoff_set(NX_TIEOFF_UART4_USESMC, 0);
		nx_tieoff_set(NX_TIEOFF_UART4_SMCTXENB, 0);
		nx_tieoff_set(NX_TIEOFF_UART4_SMCRXENB, 0);
		break;
	case 5:
		nx_tieoff_set(NX_TIEOFF_UART5_USESMC, 0);
		nx_tieoff_set(NX_TIEOFF_UART5_SMCTXENB, 0);
		nx_tieoff_set(NX_TIEOFF_UART5_SMCRXENB, 0);
		break;
	default:
		break;

	};
}

void s5p6818_uart_init(vaddr_t base, uint32_t uart_clk, uint32_t baud_rate)
{
	int ch = s5p6818_uart_get_ch_num(base);

	nx_rstcon_setrst(UART_RST_BASE+ch, 1); /* reset */
	uart_tieoff_set(ch);
	uart_clk_enb(ch);

	write32(0x3, base + UARTEFCON);
	write32(0, base + UARTFMCON);
	/* 8N1 */
	write32(0x3, base + UARTDLCON);
	/* No interrupts, no DMA, pure polling */
	write32(0x245, base + UARTRUCON);

	if (baud_rate) {
		int divisor = uart_clk / baud_rate;

		write32(divisor / 16 - 1, base + UARTBRDR);
		write32(divisor % 16, base + UARTFRACVAL);
	}
}
#endif

void s5p6818_uart_flush(vaddr_t base)
{
	while (!(read32(base + UARTTRSTAT) & NX_UARTTRSTAT_TX_EMPTY_BIT))
		;
}

void s5p6818_uart_init(vaddr_t base __unused,
		uint32_t uart_clk __unused,
		uint32_t baud_rate __unused)
{
	return;
}

void s5p6818_uart_putc(int ch, vaddr_t base)
{
	/*
	 * Wait until there is space in the FIFO
	 */
	while (read32(base + UARTFSTAT) & NX_UARTFSTAT_TXFF_BIT)
		;

	/* Send the character */
	write32(ch, base + UARTTXH);
}

bool s5p6818_uart_have_rx_data(vaddr_t base)
{
	return !(read32(base + UARTFSTAT) & NX_UARTFSTAT_RXFE_BIT);
}

int s5p6818_uart_getchar(vaddr_t base)
{
	while (!s5p6818_uart_have_rx_data(base))
		;
	return read32(base + UARTRXH) & 0xff;
}

