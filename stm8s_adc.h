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

#ifndef STM8S_ADC_H
#define STM8S_ADC_H

#include "stm8s.h"

enum adc_channel
{
	ADC1_CHANNEL_0 = 0,
	ADC1_CHANNEL_1 = 1,
	ADC1_CHANNEL_2 = 2,
	ADC1_CHANNEL_3 = 3,
	ADC1_CHANNEL_4 = 4,
	ADC1_CHANNEL_5 = 5,
	ADC1_CHANNEL_6 = 6,
	ADC1_CHANNEL_7 = 7,
	ADC1_CHANNEL_8 = 8,
	ADC1_CHANNEL_9 = 9,
};

enum adc_prescaler
{
	ADC1_PRESSEL_FCPU_D2  = 0x00, /**< fADC1 = fcpu/2 */
	ADC1_PRESSEL_FCPU_D3  = 0x10, /**< fADC1 = fcpu/3 */
	ADC1_PRESSEL_FCPU_D4  = 0x20, /**< fADC1 = fcpu/4 */
	ADC1_PRESSEL_FCPU_D6  = 0x30, /**< fADC1 = fcpu/6 */
	ADC1_PRESSEL_FCPU_D8  = 0x40, /**< fADC1 = fcpu/8 */
	ADC1_PRESSEL_FCPU_D10 = 0x50, /**< fADC1 = fcpu/10 */
	ADC1_PRESSEL_FCPU_D12 = 0x60, /**< fADC1 = fcpu/12 */
	ADC1_PRESSEL_FCPU_D18 = 0x70, /**< fADC1 = fcpu/18 */
};

void adc_init (enum adc_channel ch, enum adc_prescaler pr);
uint16_t adc_read (enum adc_channel ch);

#endif // STM8S_ADC_H
