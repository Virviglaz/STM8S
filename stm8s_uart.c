/*
 * This file is provided under a MIT license.  When using or
 *   redistributing this file, you may do so under either license.
 *
 *   MIT License
 *
 *   Copyright (c) 2019 Pavel Nadein
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * STM8S open source driver
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

#include "stm8s_uart.h"
#include "stm8s_clk.h"

#define MAX_NOF_UARTS	1

#if defined (STM8S105) || defined (STM8S005) || defined (STM8AF626x)
#undef MAX_NOF_UARTS
#define MAX_NOF_UARTS	2
#define USE_UART2
#endif /* STM8S105 || STM8S005 || STM8AF626x */

#if defined(STM8S208) ||defined(STM8S207) || defined (STM8S007) || \
		defined (STM8AF52Ax) || defined (STM8AF62Ax)
#undef MAX_NOF_UARTS
#define MAX_NOF_UARTS	3
#define USE_UART3
#endif /* (STM8S208) ||(STM8S207) || (STM8AF62Ax) || (STM8AF52Ax) */

static struct
{
	volatile char *rx_buf;
	volatile uint8_t pos;
	uint8_t size;
	bool is_done;
} rx[MAX_NOF_UARTS];

void uart_init(enum uart uart, uint16_t freq)
{
	uint16_t uart_div = clk_get_freq_MHz() * (1000000u / freq);

	switch(uart) {
	case STM8_UART1:
		CLK->PCKENR1 |= CLK_PCKENR1_UART1;
		UART1->BRR1 = (uart_div & 0x0FF0) >> 4;
		UART1->BRR2 = (uart_div & 0xF000) >> 8 | (uart_div & 0x000F);
		UART1->CR2 = UART1_CR2_TEN;
		break;
#ifdef USE_UART2
	case STM8_UART2:
		/* TODO: uart2 init */
		break;
#endif
#ifdef USE_UART3
	case STM8_UART3:
		/* TODO: uart3 init */
		break;
#endif
	}
}

void uart_enable_rx_irq(enum uart uart, char *buf, uint16_t size)
{
	rx[uart].rx_buf = buf;
	rx[uart].size = size;
	rx[uart].pos = 0;
	rx[uart].is_done = false;

	switch(uart) {
	case STM8_UART1:
		UART1->CR2 |= UART1_CR2_RIEN | UART1_CR2_REN;
		break;
#ifdef USE_UART2
	case STM8_UART2:
		/* TODO: uart2 init */
		break;
#endif
#ifdef USE_UART3
	case STM8_UART3:
		/* TODO: uart3 init */
		break;
#endif
	}
}

uint16_t uart_check_rx(enum uart uart)
{
	u8 len = rx[uart].pos;
	if (!rx[uart].is_done)
		return 0;

	rx[uart].pos = 0;
	rx[uart].is_done = false;

	return len;
}

void uart_send_string(enum uart uart, const char *buf)
{
	switch(uart) {
	case STM8_UART1:
		while(*buf) {
			while(!(UART1->SR & UART1_SR_TXE));
			UART1->DR = *buf++;
		}
	}
}

void uart_send_line(enum uart uart, const char *buf)
{
	uart_send_string(uart, buf);
	uart_send_string(uart, "\r\n");
}

static void uart_rx_handler(enum uart uart)
{
	char data;

	switch(uart) {
	case STM8_UART1:
		data = UART1->DR;
		break;
#ifdef USE_UART2
	case STM8_UART2:
		data = UART2->DR;
		break;
#endif
#ifdef USE_UART3
	case STM8_UART3:
		data = UART3->DR;
		break;
#endif
	}

	/* If buffer is still full, do nothing */
	if (rx[uart].pos == rx[uart].size || rx[uart].is_done)
		return;

	/* Buffer full or newline received */
	if (data == '\n' || data == '\r' || data == 0) {
		rx[uart].rx_buf[rx[uart].pos] = 0; /* null terminate */
		rx[uart].is_done = true;
		return;
	}

	rx[uart].rx_buf[rx[uart].pos++] = data;
}

INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18)
{
	uart_rx_handler(STM8_UART1);
}

#ifdef USE_UART2
INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21)
{
	uart_rx_handler(STM8_UART2);
}
#endif

#ifdef USE_UART3
INTERRUPT_HANDLER(UART3_RX_IRQHandler, 21)
{
	uart_rx_handler(STM8_UART3);
}
#endif
