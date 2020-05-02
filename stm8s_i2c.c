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

static struct {
	u8 **buf;
	u8 size;
	u8 rdy; /* Data ready flag */
	u8 upd_buf; /* Last updated buffer */
	u8 bytes_rcv;
} _i2c_slave;

static inline void stop(void)
{
	I2C->CR2 |= I2C_CR2_STOP;
	while (I2C->SR1 & I2C_SR1_STOPF);
}

u16 i2c_error_counter = 0;
static u8 inline try_recover(void)
{
	u16 i = 0xFFFF;
	if (i2c_error_counter < 0xFFFF)
		i2c_error_counter++;

	I2C->CR1 &= ~I2C_CR1_PE;
	GPIOB->DDR |= (3 << 4);
	GPIOB->ODR &= ~(3 << 4);
	while (i--);
	GPIOB->DDR &= ~(3 << 4);
	GPIOB->ODR |= (3 << 4);
	I2C->CR1 = I2C_CR1_PE;

	stop();
	return I2C->SR3 & I2C_SR3_BUSY;
}

static inline void gpio_setup(void)
{
	GPIOB->DDR &= ~(3 << 4);
	GPIOB->ODR |= (3 << 4);
	GPIOB->CR1 &= ~(3 << 4);
	GPIOB->CR2 &= ~(3 << 4);
}

static inline void clk_setup(void)
{
	/* CCR = Fmaster / 2 * Fiic */
	u16 ccr = clk_get_freq_MHz();
	CLK->PCKENR1 |= CLK_PCKENR1_I2C;
	I2C->FREQR = (u8)ccr;
	I2C->TRISER = ccr + 1;
	ccr = ccr * 5;
	I2C->CCRL = (u8)ccr;
	I2C->CCRH = ccr >> 8;
	I2C->CR1 = I2C_CR1_PE;
}

static u8 start(void)
{
	if (!(CLK->PCKENR1 & CLK_PCKENR1_I2C) || !(I2C->CR1 & I2C_CR1_PE)) {
		gpio_setup();
		clk_setup();
		stop();
	}

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

static inline void restart(void)
{
	I2C->CR2 = I2C_CR2_START;
	while (!(I2C->SR1 & I2C_SR1_SB));
}

static inline u8 send_addr(u8 addr)
{
	I2C->DR = addr;
	while (!(I2C->SR1 & I2C_SR1_ADDR))
		if (I2C->SR2 & I2C_SR2_AF)
			return I2C_ERR_NACK;

	I2C->SR3;
	while (!I2C->SR1 & I2C_SR1_TXE);

	return 0;
}

static inline void i2c_write(u8 data)
{
	while (!(I2C->SR1 & I2C_SR1_TXE));
	I2C->DR = data;
}

u8 i2c_write_reg(u8 addr, u8 reg, u8 *buf, u16 size)
{
	u8 ret;
	if (start())
		return I2C_ERR_BUSY;

	ret = send_addr(addr << 1);
	if (!ret) {
		i2c_write(reg);
		while (size--)
			i2c_write(*buf++);
	}

	while (!(I2C->SR1 & I2C_SR1_BTF));
	stop();
	return ret;
}

u8 i2c_read_reg(u8 addr, u8 reg, u8 *buf, u16 size)
{
	if (start())
		return I2C_ERR_BUSY;

	if (send_addr(addr << 1))
		goto noack;
	i2c_write(reg);
	restart();
	I2C->CR2 |= I2C_CR2_ACK;
	if (send_addr((addr << 1) | 1))
		goto noack;

	if (size == 1) {
		I2C->CR2 &= ~I2C_CR2_ACK;
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
		//while (!(I2C->SR1 & I2C_SR1_ADDR));
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
	stop();
	return I2C_ERR_NACK;
}

void i2c_slave(u8 addr, u8 **buf, u8 size)
{
	_i2c_slave.buf = buf;
	_i2c_slave.size = size;

	clk_setup();
	I2C->OARL = addr << 1;
	I2C->OARH = I2C_OARH_ADDCONF;
	I2C->ITR = I2C_ITR_ITBUFEN | I2C_ITR_ITEVTEN;
	I2C->CR1 = I2C_CR1_PE;
	I2C->CR2 |= I2C_CR2_ACK;
	enableInterrupts();
}

INTERRUPT_HANDLER(I2C_IRQHandler, 19)
{
	static u8 index_set = 0, i, num;
	if (I2C->SR1 & I2C_SR1_ADDR) {
		/* Address match */
		I2C->CR2 |= I2C_CR2_ACK;
		index_set = 0;
		num = 0;

	} else if (I2C->SR1 & I2C_SR1_RXNE) {
		/* Data received */
		if (!index_set) {
			index_set = !0;
			i = I2C->DR;
			if (i >= _i2c_slave.size)
				i = _i2c_slave.size - 1;
		} else {
			_i2c_slave.buf[i][num++] = I2C->DR;
		}
	} else if (I2C->SR1 & I2C_SR1_STOPF) {
		/* Stop condition */
		_i2c_slave.rdy = !0;
		_i2c_slave.upd_buf = i;
		_i2c_slave.bytes_rcv = num;
	} else {
		/* Reading the slave */
		I2C->DR = _i2c_slave.buf[i][num++];
	}

	I2C->SR3;
}

/* Returns number of bytes received last time */
u8 i2c_slave_check_data(u8 *buf_num)
{
	if (!_i2c_slave.rdy)
		return 0;

	if (buf_num)
		*buf_num = _i2c_slave.upd_buf;

	return _i2c_slave.bytes_rcv;
}