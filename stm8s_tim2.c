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

#include "stm8s_tim2.h"

static void (*tim2_irq_handler)(void);

/* NOTE: timer frequency == CPU freq / (2 * prescaler) */

void tim2_init(enum tim2_presc prescaler, u16 period)
{
	CLK->PCKENR1 |= CLK_PCKENR1_TIM2;
	TIM2->CR1 = 0;
	TIM2->PSCR = prescaler ? (u8)prescaler - 1 : 0;
	TIM2->ARRH = (u8)(period >> 8);
	TIM2->ARRL = (u8)(period);
	TIM2->EGR |= TIM2_EGR_UG;
	TIM2->CR1 |= TIM2_CR1_CEN;
}

void tim2_set_freq(u16 period)
{
	TIM2->ARRH = (u8)(period >> 8);
	TIM2->ARRL = (u8)(period);
	TIM2->EGR |= TIM2_EGR_UG;
}

void tim2_enable_irq(void (*handler)(void))
{
	tim2_irq_handler = handler;
	TIM2->IER = handler ? TIM2_IER_UIE : 0;
}

void tim2_enable(bool enabled)
{
	if (enabled)
		TIM2->CR1 |= TIM2_CR1_CEN;
	else
		TIM2->CR1 &= ~TIM2_CR1_CEN;
}

void tim2_pwm_init(enum tim2_pwm ch, u16 duty)
{
	switch(ch) {
	case TIM2_PWM1:
		/* PD4 (AFR0 => PC5) */
		TIM2->CCER1 |= TIM2_CCER1_CC1E;
		TIM2->CCMR1 = 0x60; /* PWM1, no preload */
  		TIM2->CCR1H = (u8)(duty >> 8);
  		TIM2->CCR1L = (u8)(duty);
		break;
	case TIM2_PWM2:
		/* PD3 */
		TIM2->CCER1 |= TIM2_CCER1_CC2E;
		TIM2->CCMR2 = 0x60; /* PWM1, no preload */
  		TIM2->CCR2H = (u8)(duty >> 8);
  		TIM2->CCR2L = (u8)(duty);
		break;
	case TIM2_PWM3:
		/* PA3 (AFR1 => PD2) */
		TIM2->CCER2 |= TIM2_CCER2_CC3E;
		TIM2->CCMR3 = 0x60; /* PWM1, no preload */
  		TIM2->CCR3H = (u8)(duty >> 8);
  		TIM2->CCR3L = (u8)(duty);
		break;
	}
}

INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13)
{
	tim2_irq_handler();
	TIM2->SR1 &= ~TIM2_SR1_UIF;
}
