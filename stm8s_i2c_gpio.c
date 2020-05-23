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

#include "stm8s_i2c_gpio.h"

#define ACK		0
#define NACK		1
#define HIGH		1
#define LOW		0

#if !defined(SCL_PIN) && !defined(SDA_PIN)
#define SCL_PIN		PB4
#define SDA_PIN		PB5
#endif

static enum i2c_gpio_res_t start(void)
{
	gpio_set(SDA_PIN);
	gpio_set(SCL_PIN);

	/*
	 * Check, if any BUS signal is low that meas that BUS is busy
	 * by other device or H/W issue
	 */
	if (!gpio_read(SDA_PIN))
		return I2C_BUS_BUSY;

	gpio_clr(SDA_PIN);
	gpio_clr(SCL_PIN);

	return I2C_SUCCESS;
}

static void restart(void)
{
	gpio_set(SDA_PIN);
	gpio_set(SCL_PIN);
	gpio_clr(SDA_PIN);
	gpio_clr(SCL_PIN);
}

static void stop(void)
{
	gpio_clr(SDA_PIN);
	gpio_set(SCL_PIN);
	gpio_set(SDA_PIN);
}

static uint8_t clock(void)
{
	uint8_t res;

	gpio_set(SCL_PIN);
	res = gpio_read(SDA_PIN);
	gpio_clr(SCL_PIN);

	return res;
}

static uint8_t write(uint8_t data)
{
	uint8_t mask = 0x80;

	while (mask) {
		(data & mask) ? gpio_set(SDA_PIN) : gpio_clr(SDA_PIN);
		clock();
		mask >>= 1;
	}

	gpio_set(SDA_PIN);
	return clock();
}

static uint8_t read(uint8_t ack)
{
	uint8_t data = 0, mask = 0x80;

	while(mask) {
		if (clock())
			data |= mask;
  		mask >>= 1;
	}

	if (ack) {
		gpio_clr(SDA_PIN);
		clock();
		gpio_set(SDA_PIN);
	} else {
		gpio_set(SDA_PIN);
		clock();
	}

	return data;
}

enum i2c_gpio_res_t i2c_gpio_write(uint8_t i2c_addr,
			      uint8_t *addr, uint8_t addr_len,
			      uint8_t *buf, uint16_t size)
{
	enum i2c_gpio_res_t res = I2C_ADD_NOT_EXIST;

	/* Check BUS */
	if (start() != I2C_SUCCESS)
		return I2C_BUS_BUSY;

	/* Send device address for write */
	if (write(i2c_addr << 1) == ACK) {
		res = I2C_SUCCESS;

		while (addr_len--)
			res = (enum i2c_gpio_res_t)write(*addr++);

		while(size-- && res == I2C_SUCCESS) //send buffer
			res = (enum i2c_gpio_res_t)write(*buf++);
	}

	stop();
	return res;
}

enum i2c_gpio_res_t i2c_gpio_read(uint8_t i2c_addr,
			      uint8_t *addr, uint8_t addr_len,
			      uint8_t *buf, uint16_t size)
{
	enum i2c_gpio_res_t res = I2C_ADD_NOT_EXIST;

	/* Check BUS */
	if (start() != I2C_SUCCESS)
		return I2C_BUS_BUSY;

	/* Send device address for write */
	if (write(i2c_addr << 1) == ACK) {
		res = (enum i2c_gpio_res_t)ACK;

		while(addr_len--)
			res = (enum i2c_gpio_res_t)write(*addr++);

		restart();

		if (write((i2c_addr << 1) | 0x01) == I2C_SUCCESS) {
			while (size--)
				*buf++ = read(size ? 1 : 0);
			res = I2C_SUCCESS;
		}
	}

	stop();
	return res;
}


