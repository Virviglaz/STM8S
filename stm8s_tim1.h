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

#ifndef STM8S_TIM1_H
#define STM8S_TIM1_H

#include "stm8s.h"

enum tim1_pwm {
	TIM1_PWM1,
	TIM1_PWM2,
	TIM1_PWM3,
	TIM1_PWM4,
};

static inline void tim1_deinit(void)
{
	TIM1->CR1 = 0;
	CLK->PCKENR1 &= ~CLK_PCKENR1_TIM1;
}

static inline void tim1_set_pwm1(u16 duty)
{
	TIM1->CCR1H = (u8)(duty >> 8);
	TIM1->CCR1L = (u8)(duty);
}

static inline void tim1_set_pwm2(u16 duty)
{
	TIM1->CCR2H = (u8)(duty >> 8);
	TIM1->CCR2L = (u8)(duty);
}

static inline void tim1_set_pwm3(u16 duty)
{
	TIM1->CCR3H = (u8)(duty >> 8);
	TIM1->CCR3L = (u8)(duty);
}

static inline void tim1_set_pwm4(u16 duty)
{
	TIM1->CCR4H = (u8)(duty >> 8);
	TIM1->CCR4L = (u8)(duty);
}

void tim1_init(u16 prescaler, u16 period);
void tim1_set_freq(u16 period);
void tim1_enable_irq(void (*handler)(void));
void tim1_enable(bool enabled);
void tim1_pwm_init(enum tim1_pwm ch, u16 duty);

#endif /* STM8S_TIM1_H */
