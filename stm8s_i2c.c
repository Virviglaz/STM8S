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

#include "stm8s_i2c.h"
#include "stm8s_clk.h"

static void i2c_stop(void)
{
	I2C->CR2 |= I2C_CR2_STOP;
	while (I2C->SR1 & I2C_SR1_STOPF);

	I2C->CR1 &= ~I2C_CR1_PE;
}

static u8 try_recover(void)
{
	u16 i = 0xFFFF;

	I2C->CR1 &= ~I2C_CR1_PE;
	GPIOB->DDR |= (3 << 4);
	GPIOB->ODR &= ~(3 << 4);
	while (i--);
	GPIOB->DDR &= ~(3 << 4);
	GPIOB->ODR |= (3 << 4);
	I2C->CR1 = I2C_CR1_PE;

	i2c_stop();
	return I2C->SR3 & I2C_SR3_BUSY;
}

static u8 i2c_start(void)
{
	/* CCR = Fmaster / 2 * Fiic */
	u16 ccr = clk_get_freq_MHz();
	GPIOB->DDR &= ~(3 << 4);
	GPIOB->ODR |= (3 << 4);
	GPIOB->CR1 &= ~(3 << 4);
	GPIOB->CR2 &= ~(3 << 4);

	I2C->FREQR = (u8)ccr;
	ccr = ccr * 5;
	I2C->CCRL = (u8)ccr;
	I2C->CCRH = ccr >> 8;
	I2C->CR1 = I2C_CR1_PE;

	if (I2C->SR3 & I2C_SR3_BUSY) {
		if (try_recover()) {
			I2C->CR1 &= ~I2C_CR1_PE;
			return I2C_ERR_BUSY;
		}
	}

	I2C->CR2 = I2C_CR2_START;
	while (!(I2C->SR1 & I2C_SR1_SB));

	return 0;
}

static void i2c_restart(void)
{
	I2C->CR2 = I2C_CR2_START;
	while (!(I2C->SR1 & I2C_SR1_SB));
}

static u8 i2c_addr(u8 addr)
{
	I2C->DR = addr;
	while (!(I2C->SR1 & I2C_SR1_ADDR))
		if (I2C->SR2 & I2C_SR2_AF)
			return I2C_ERR_NACK;

	I2C->SR3;
	while (!I2C->SR1 & I2C_SR1_TXE);

	return 0;
}

static void i2c_write(u8 data)
{
	while (!(I2C->SR1 & I2C_SR1_TXE));
	I2C->DR = data;
}

u8 i2c_write_reg(u8 addr, u8 reg, u8 *buf, u16 size)
{
	u8 ret;
	if (i2c_start())
		return I2C_ERR_BUSY;

	ret = i2c_addr(addr << 1);
	if (!ret) {
		i2c_write(reg);
		while (size--)
			i2c_write(*buf++);
	}

	i2c_stop();
	return ret;
}

u8 i2c_read_reg(u8 addr, u8 reg, u8 *buf, u16 size)
{
	if (i2c_start())
		return I2C_ERR_BUSY;

	if (i2c_addr(addr << 1))
		goto noack;
	i2c_write(reg);
	i2c_restart();
	if (i2c_addr((addr << 1) | 1))
		goto noack;

	if (size == 1) {
		I2C->CR2 &= ~I2C_CR2_ACK;
		while (!(I2C->SR1 & I2C_SR1_ADDR));
		disableInterrupts();
		I2C->SR3;
		I2C->CR2 |= I2C_CR2_STOP;
		enableInterrupts();
		while (!(I2C->SR1 & I2C_SR1_RXNE));
		*buf = I2C->DR;
	} else if (size == 2) {
		I2C->CR2 |= I2C_CR2_POS;
		while (!(I2C->SR1 & I2C_SR1_ADDR));
		disableInterrupts();
		I2C->SR3;
		I2C->CR2 &= ~I2C_CR2_ACK;
		enableInterrupts();
		while (!(I2C->SR1 & I2C_SR1_BTF));
		disableInterrupts();
		I2C->CR2 |= I2C_CR2_STOP;
		*buf++ = I2C->DR;
		enableInterrupts();
		*buf++ = I2C->DR;
	} else {
		while (!(I2C->SR1 & I2C_SR1_ADDR));
		disableInterrupts();
		I2C->SR3;
		enableInterrupts();
		while (size-- > 3) {
			while (!(I2C->SR1 & I2C_SR1_BTF));
			*buf++ = I2C->DR;
		}

		while (!(I2C->SR1 & I2C_SR1_BTF));
		I2C->CR2 &= ~I2C_CR2_ACK;
		disableInterrupts();
		*buf++ = I2C->DR;
		I2C->CR2 |= I2C_CR2_STOP;
		*buf++ = I2C->DR;
		enableInterrupts();
		while (!(I2C->SR1 & I2C_SR1_RXNE));
		*buf = I2C->DR;
	}

	while (!(I2C->CR2 & I2C_CR2_STOP));
	I2C->CR2 &= ~I2C_CR2_POS;

	return 0;
noack:
	i2c_stop();
	return I2C_ERR_NACK;
}