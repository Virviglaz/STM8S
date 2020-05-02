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

#ifndef STM8S_GPIO_H
#define STM8S_GPIO_H

#include "stm8s.h"

#define PA0			GPIOA, PIN_0
#define PA1			GPIOA, PIN_1
#define PA2			GPIOA, PIN_2
#define PA3			GPIOA, PIN_3
#define PA4			GPIOA, PIN_4
#define PA5			GPIOA, PIN_5
#define PA6			GPIOA, PIN_6
#define PA7			GPIOA, PIN_7

#define PB0			GPIOB, PIN_0
#define PB1			GPIOB, PIN_1
#define PB2			GPIOB, PIN_2
#define PB3			GPIOB, PIN_3
#define PB4			GPIOB, PIN_4
#define PB5			GPIOB, PIN_5
#define PB6			GPIOB, PIN_6
#define PB7			GPIOB, PIN_7

#define PC0			GPIOC, PIN_0
#define PC1			GPIOC, PIN_1
#define PC2			GPIOC, PIN_2
#define PC3			GPIOC, PIN_3
#define PC4			GPIOC, PIN_4
#define PC5			GPIOC, PIN_5
#define PC6			GPIOC, PIN_6
#define PC7			GPIOC, PIN_7

#define PD0			GPIOD, PIN_0
#define PD1			GPIOD, PIN_1
#define PD2			GPIOD, PIN_2
#define PD3			GPIOD, PIN_3
#define PD4			GPIOD, PIN_4
#define PD5			GPIOD, PIN_5
#define PD6			GPIOD, PIN_6
#define PD7			GPIOD, PIN_7

#define PE0			GPIOE, PIN_0
#define PE1			GPIOE, PIN_1
#define PE2			GPIOE, PIN_2
#define PE3			GPIOE, PIN_3
#define PE4			GPIOE, PIN_4
#define PE5			GPIOE, PIN_5
#define PE6			GPIOE, PIN_6
#define PE7			GPIOE, PIN_7

#define PF0			GPIOF, PIN_0
#define PF1			GPIOF, PIN_1
#define PF2			GPIOF, PIN_2
#define PF3			GPIOF, PIN_3
#define PF4			GPIOF, PIN_4
#define PF5			GPIOF, PIN_5
#define PF6			GPIOF, PIN_6
#define PF7			GPIOF, PIN_7

/* Legacy support */
#define PIN_ON(x)		gpio_set(x)
#define PIN_OFF(x)		gpio_clr(x)
#define gpio_reset(x)		gpio_clr(x)

enum gpio_pin
{
	PIN_0	= ((uint8_t)0x01),	/*!< Pin 0 selected */
	PIN_1	= ((uint8_t)0x02),	/*!< Pin 1 selected */
	PIN_2	= ((uint8_t)0x04),	/*!< Pin 2 selected */
	PIN_3	= ((uint8_t)0x08),	/*!< Pin 3 selected */
	PIN_4	= ((uint8_t)0x10),	/*!< Pin 4 selected */
	PIN_5	= ((uint8_t)0x20),	/*!< Pin 5 selected */
	PIN_6	= ((uint8_t)0x40),	/*!< Pin 6 selected */
	PIN_7	= ((uint8_t)0x80),	/*!< Pin 7 selected */
	PIN_LN 	= ((uint8_t)0x0F),	/*!< Low nibble pins selected */
	PIN_HN	= ((uint8_t)0xF0),	/*!< High nibble pins selected */
	PIN_ALL	= ((uint8_t)0xFF),	/*!< All pins selected */
};

enum gpio_dir { INPUT, OUTPUT };
enum gpio_speed { SPEED_2MHz, SPEED_10MHz };
enum gpio_output_type { OPEN_DRAIN, PUSH_PULL };

typedef struct { GPIO_TypeDef *gpio; enum gpio_pin pin; } io_pin;

static inline void gpio_set_dir(GPIO_TypeDef *gpio, enum gpio_pin pin,
				enum gpio_dir dir)
{
	gpio->DDR = dir == OUTPUT ?
		gpio->DDR | (u8)pin : gpio->DDR & (~(u8)pin);
}

static inline void gpio_set_output(GPIO_TypeDef *gpio, enum gpio_pin pin,
		  enum gpio_output_type type)
{
 	gpio->CR1 = type == PUSH_PULL ?
		gpio->CR1 |= (u8)pin : gpio->CR1 & (~(u8)pin);
}

static inline enum gpio_dir gpio_get_dir(GPIO_TypeDef *gpio, enum gpio_pin pin)
{
	return gpio->DDR & (u8)pin ? OUTPUT : INPUT;
}

static inline void gpio_set(GPIO_TypeDef *gpio, enum gpio_pin pin)
{
	gpio->ODR |= (u8)pin;
}

static inline void gpio_clr(GPIO_TypeDef *gpio, enum gpio_pin pin)
{
	gpio->ODR &= ~(u8)pin;
}

static inline void gpio_toggle(GPIO_TypeDef *gpio, enum gpio_pin pin)
{
	gpio->ODR ^= (u8)pin;
}

static inline void gpio_pin_switch(GPIO_TypeDef *gpio, enum gpio_pin pin,
				   bool state)
{
	if (state)
		gpio_set(gpio, pin);
	else
		gpio_clr(gpio, pin);
}

static inline u8 gpio_get_latch(GPIO_TypeDef *gpio, enum gpio_pin pin)
{
	return gpio->ODR & (u8)pin;
}

static inline u8 gpio_get_value(GPIO_TypeDef *gpio, enum gpio_pin pin)
{
	return gpio->IDR & (u8)pin;
}

static inline void gpio_pullup(GPIO_TypeDef *gpio, enum gpio_pin pin, bool pullup)
{
	gpio->CR1 = pullup ? gpio->CR1 | (u8)pin : gpio->CR1 & (~(u8)pin);
}

static inline void gpio_set_speed(GPIO_TypeDef *gpio, enum gpio_pin pin,
	enum gpio_speed speed)
{
	gpio->CR2 = speed == SPEED_10MHz ?
		gpio->CR2 | (u8)pin : gpio->CR2 & (~(u8)pin);
}

static inline void gpio_irq(GPIO_TypeDef *gpio, enum gpio_pin pin, bool irq)
{
	gpio->CR2 = irq ? gpio->CR2 | (u8)pin : gpio->CR2 & (~(u8)pin);
}

void gpio_init(GPIO_TypeDef *gpio, enum gpio_pin pin, enum gpio_dir dir);

#endif /* STM8S_GPIO_H */