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

#include "stm8s_gpio.h"

/* Set pin direction INPUT/OUTPUT */
void gpio_set_dir(GPIO_TypeDef *gpio, enum gpio_pin pin, enum gpio_dir dir)
{
	gpio->DDR = dir == OUTPUT ?
		gpio->DDR | (u8)pin : gpio->DDR & (~(u8)pin);
}

void gpio_set_output(GPIO_TypeDef *gpio, enum gpio_pin pin,
		     enum gpio_output_type type)
{
 	gpio->CR1 = type == PUSH_PULL ?
		gpio->CR1 |= (u8)pin : gpio->CR1 & (~(u8)pin);
}

/* Get pin direction */
enum gpio_dir gpio_get_dir(GPIO_TypeDef *gpio, enum gpio_pin pin)
{
	return gpio->DDR & (u8)pin ? OUTPUT : INPUT;
}

/* Set pin value to latched register */
void gpio_set(GPIO_TypeDef *gpio, enum gpio_pin pin)
{
	gpio->ODR |= (u8)pin;
}

/* Reset pin value in latched register */
void gpio_reset(GPIO_TypeDef *gpio, enum gpio_pin pin)
{
	gpio->ODR &= ~(u8)pin;
}

/* Set pin value to state (true/false == 1/0) */
void gpio_pin_switch(GPIO_TypeDef *gpio, enum gpio_pin pin, bool state)
{
	if (state)
		gpio_set(gpio, pin);
	else
		gpio_reset(gpio, pin);
}

/* Get pin value from latched register */
u8 gpio_get_latch(GPIO_TypeDef *gpio, enum gpio_pin pin)
{
	return gpio->ODR & (u8)pin;
}

/* Get pin value from input register */
u8 gpio_get_value(GPIO_TypeDef *gpio, enum gpio_pin pin)
{
	return gpio->IDR & (u8)pin;
}

/* Sets the pullup on gpio_pin of port gpio */
void gpio_pullup(GPIO_TypeDef *gpio, enum gpio_pin pin, bool pullup)
{
	gpio->CR1 = pullup ? gpio->CR1 | (u8)pin : gpio->CR1 & (~(u8)pin);
}

/* Set speed capability of gpio_pin of gpio port */
void gpio_set_speed(GPIO_TypeDef *gpio, enum gpio_pin pin,
	enum gpio_speed speed)
{
	gpio->CR2 = speed == SPEED_10MHz ?
		gpio->CR2 | (u8)pin : gpio->CR2 & (~(u8)pin);
}

/* Enables the IRQ for gpio_pin at port gpio */
void gpio_irq(GPIO_TypeDef *gpio, enum gpio_pin pin, bool irq)
{
	gpio->CR2 = irq ? gpio->CR2 | (u8)pin : gpio->CR2 & (~(u8)pin);
}

/* Init pin with default settings */
void gpio_init(GPIO_TypeDef *gpio, enum gpio_pin pin, enum gpio_dir dir)
{
	gpio_set_dir(gpio, pin, dir);
	if (dir == OUTPUT) {
		gpio_set_output(gpio, pin, PUSH_PULL);
		gpio_set_speed(gpio, pin, SPEED_2MHz);
		gpio_reset(gpio, pin);
	}
	else {
		gpio_pullup(gpio, pin, false);
		gpio_irq(gpio, pin, false);
	}
}
