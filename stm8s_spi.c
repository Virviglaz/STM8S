/*
 * This file is provided under a MIT license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * MIT License
 *
 * Copyright (c) 2020 Pavel Nadein
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * STM8S open source driver
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

#include "stm8s_spi.h"

void spi_init(enum spi_clock clock, enum spi_freq freq)
{
	CLK->PCKENR1 |= CLK_PCKENR1_SPI;
	SPI->CR2 = SPI_CR2_SSI | SPI_CR2_SSM;
	SPI->CR1 = SPI_CR1_SPE | SPI_CR1_MSTR | (u8)freq | (u8)clock;
}

u8 spi_read(u8 value)
{
	/* Loop while DR register in not emplty */
	while (!(SPI->SR & SPI_SR_TXE));

	/* Send byte through the SPI peripheral */
	SPI->DR = value;

	/* Wait to receive a byte */
	while (!(SPI->SR & SPI_SR_RXNE));

	/* Return the byte read from the SPI bus */
	return SPI->DR;
}

static inline void spi_select(GPIO_TypeDef *gpio, enum gpio_pin pin)
{
	gpio_clr(gpio, pin);
}

static inline void spi_release(GPIO_TypeDef *gpio, enum gpio_pin pin)
{
	while (SPI->SR & SPI_SR_BSY);
	gpio_set(gpio, pin);
}

u8 spi_write_reg(GPIO_TypeDef *gpio, enum gpio_pin pin,
		 u8 reg, u8 *buf, u16 size)
{
	u8 res;

	spi_select(gpio, pin);
	res = spi_read(reg);

	while(size--)
		spi_read(*buf++);

	spi_release(gpio, pin);
	return res;
}

u8 spi_read_reg(GPIO_TypeDef *gpio, enum gpio_pin pin,
		 u8 reg, u8 *buf, u16 size)
{
	u8 res;

	spi_select(gpio, pin);
	res = spi_read(reg);

	while(size--)
		*buf++ = spi_read(reg);

	spi_release(gpio, pin);
	return res;
}
