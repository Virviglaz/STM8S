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

#ifndef STM8S_TIM2_H
#define STM8S_TIM2_H

#include "stm8s.h"

enum tim2_presc {
	TIM2_HSI_DIV_2,
	TIM2_HSI_DIV_4,
	TIM2_HSI_DIV_8,
	TIM2_HSI_DIV_16,
	TIM2_HSI_DIV_32,
	TIM2_HSI_DIV_64,
	TIM2_HSI_DIV_128,
	TIM2_HSI_DIV_256,
};

enum tim2_pwm {
	TIM2_PWM1,
	TIM2_PWM2,
	TIM2_PWM3,
};

static inline void tim2_deinit(void)
{
	TIM2->CR1 = 0;
	CLK->PCKENR1 &= ~CLK_PCKENR1_TIM2;
}

static inline void tim2_set_pwm1(u16 duty)
{
	TIM2->CCR1H = (u8)(duty >> 8);
	TIM2->CCR1L = (u8)(duty);
}

static inline void tim2_set_pwm2(u16 duty)
{
	TIM2->CCR2H = (u8)(duty >> 8);
	TIM2->CCR2L = (u8)(duty);
}

static inline void tim2_set_pwm3(u16 duty)
{
	TIM2->CCR3H = (u8)(duty >> 8);
	TIM2->CCR3L = (u8)(duty);
}

void tim2_init(enum tim2_presc prescaler, u16 period);
void tim2_set_freq(u16 period);
void tim2_enable_irq(void (*handler)(void));
void tim2_enable(bool enabled);
void tim2_pwm_init(enum tim2_pwm ch, u16 duty);

#endif /* STM8S_TIM2_H */