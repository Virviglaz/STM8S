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

#define I2C_DONE		BIT(0)
#define I2C_ACK_FAILURE		BIT(1)
#define I2C_SENDING_DATA	BIT(2)
#define I2C_READING_DATA	BIT(3)
#define I2C_REPEATING_START	BIT(4)

#define REG_SR1			sr[0]
#define REG_SR2			sr[1]
#define REG_SR3			sr[2]

static void (*irq_handler)(void);
static void master_tx_irq_handler(void);

static volatile struct {
	u8 res;
	u8 addr;
	u8 reg;
	u8 *buf;
	u16 size;
	u16 pos;
} master_buffer;

static volatile struct {
	u8 **buf;
	u8 size;
	u8 rdy; /* Data ready flag */
	u8 upd_buf; /* Last updated buffer */
	u8 bytes_rcv;
} slave_buffer;

static void master_irq_check_finish(u16 pos)
{
	master_buffer.pos = pos;
	if (pos == master_buffer.size) {
		I2C->CR2 = I2C_CR2_STOP;
		master_buffer.res = I2C_DONE;
		irq_handler = master_tx_irq_handler;
	}
}

static u8 mstart_irq_check_errors(u8 sr2)
{
	/* Arbitration lost */
	if (sr2 & I2C_SR2_ARLO) {
		I2C->SR2 = 0;
		return I2C_SR2_ARLO;
	}
	return 0;
}

static void master_rx_irq_handler(void)
{
	u8 sr[] = { I2C->SR1, I2C->SR2, I2C->SR3 };
	u16 pos = master_buffer.pos;

	/* Repeast start was send */
	if (REG_SR1 & I2C_SR1_SB) {
		I2C->DR = master_buffer.addr << 1 | 1;
		if (master_buffer.size > 1)
			I2C->CR2 = I2C_CR2_ACK;
		return;
	}

	/* Address has been send */
	if (REG_SR1 & (I2C_SR1_ADDR | I2C_SR1_RXNE)) {
		u8 data = I2C->DR;
		master_buffer.buf[pos] = data;
		pos++;
		master_irq_check_finish(pos);
		return;
	}

	if (mstart_irq_check_errors(REG_SR2))
		return;
}

static void master_tx_irq_handler(void)
{
	u8 sr[] = { I2C->SR1, I2C->SR2, I2C->SR3 };
	u16 pos = master_buffer.pos;

	/* Start condition done, send address */
	if (REG_SR1 & I2C_SR1_SB) {
		I2C->DR = master_buffer.addr << 1;
		return;
	}

	/* Acknowledge failure */
	if (REG_SR2 & I2C_SR2_AF) {
		I2C->SR2 = 0;
		I2C->CR2 = I2C_CR2_STOP;
		master_buffer.res = I2C_ACK_FAILURE;
		return;
	}

	/* Send reg address */
	if (REG_SR1 & I2C_SR1_ADDR) {
		I2C->DR = master_buffer.reg;
		if (master_buffer.res == I2C_READING_DATA) {
			irq_handler = master_rx_irq_handler;
			I2C->CR2 = I2C_CR2_START;
		}
		return;
	}

	/* Shift out the data */
	if (REG_SR1 & I2C_SR1_TXE) {
		I2C->DR = master_buffer.buf[pos];
		pos++;
		master_irq_check_finish(pos);
		return;
	}

	if (mstart_irq_check_errors(REG_SR2))
		return;

	if (REG_SR1 & I2C_SR1_RXNE) {
		I2C->DR;
		return;
	}
}

static void slave_irq_handler(void)
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
			if (i >= slave_buffer.size)
				i = slave_buffer.size - 1;
		} else {
			u8 data = I2C->DR;
			slave_buffer.buf[i][num] = data;
			num++;
		}
	} else if (I2C->SR1 & I2C_SR1_STOPF) {
		/* Stop condition */
		slave_buffer.rdy = !0;
		slave_buffer.upd_buf = i;
		slave_buffer.bytes_rcv = num;
	} else {
		/* Reading the slave */
		I2C->DR = slave_buffer.buf[i][num++];
	}

	I2C->SR3;
}

static void clk_setup(void)
{
	/* CCR = Fmaster / 2 * Fiic */
	u16 ccr = clk_get_freq_MHz();
	if (I2C->SR3 & I2C_SR3_BUSY)
		I2C->CR2 = I2C_CR2_SWRST;
	CLK->PCKENR1 |= CLK_PCKENR1_I2C;
	I2C->FREQR = (u8)ccr;
	I2C->TRISER = ccr + 1;
	ccr = ccr * 5;
	I2C->CCRL = (u8)ccr;
	I2C->CCRH = ccr >> 8;
}

static void init(void)
{
	if (I2C->CR1 & I2C_CR1_PE && I2C->SR3 & I2C_SR3_MSL)
		return;

	/* PB4-> SCL, PB5->SDA */
	GPIOB->DDR &= ~(3 << 4);
	GPIOB->ODR |= (3 << 4);
	GPIOB->CR1 &= ~(3 << 4);
	GPIOB->CR2 &= ~(3 << 4);

	clk_setup();
	irq_handler = master_tx_irq_handler;
	I2C->ITR = I2C_ITR_ITEVTEN | I2C_ITR_ITERREN;
	I2C->CR1 = I2C_CR1_PE;
	enableInterrupts();
}

INTERRUPT_HANDLER(I2C_IRQHandler, 19)
{
	irq_handler();
}

static u8 rw_reg(u8 addr, u8 reg, u8 *buf, u16 size, u8 flag)
{
	init();

	master_buffer.res = flag;
	master_buffer.pos = 0;
	master_buffer.addr = addr;
	master_buffer.reg = reg;
	master_buffer.buf = buf;
	master_buffer.size = size;

	if (I2C->SR3 & I2C_SR3_BUSY) {
		I2C->CR2 = I2C_CR2_STOP;
		while (I2C->CR2 & I2C_CR2_STOP);
	}

	/* Start condition */
	I2C->CR2 = I2C_CR2_START;

	/* Wait for interrupt to finish */
	while (!(master_buffer.res & (I2C_DONE | I2C_ACK_FAILURE)));

	/* Wait for stop condition to finish */
	while (I2C->CR2 & I2C_CR2_STOP);

	if (master_buffer.res & I2C_ACK_FAILURE)
		return I2C_ERR_NACK;

	return !(master_buffer.res & I2C_DONE);
}

u8 i2c_write_reg(u8 addr, u8 reg, u8 *buf, u16 size)
{
	return rw_reg(addr, reg, buf, size, I2C_SENDING_DATA);
}

u8 i2c_read_reg(u8 addr, u8 reg, u8 *buf, u16 size)
{
	return rw_reg(addr, reg, buf, size, I2C_READING_DATA);
}

void i2c_slave(u8 addr, u8 **buf, u8 size)
{
	slave_buffer.buf = buf;
	slave_buffer.size = size;
	irq_handler = slave_irq_handler;

	clk_setup();
	I2C->OARL = addr << 1;
	I2C->OARH = I2C_OARH_ADDCONF;
	I2C->ITR = I2C_ITR_ITBUFEN | I2C_ITR_ITEVTEN;
	I2C->CR1 = I2C_CR1_PE;
	I2C->CR2 |= I2C_CR2_ACK;
	enableInterrupts();
}

/* Returns number of bytes received last time */
u8 i2c_slave_check_data(u8 *buf_num)
{
	if (!slave_buffer.rdy)
		return 0;

	if (buf_num)
		*buf_num = slave_buffer.upd_buf;

	return slave_buffer.bytes_rcv;
}