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

#ifndef STM8S_AWU_H
#define STM8S_AWU_H

#include "stm8s.h"

enum awu_timebase_t {
	AWU_Timebase_No_IT  = (uint8_t)0,    /*!< No AWU interrupt selected */
	AWU_Timebase_250us  = (uint8_t)1,    /*!< AWU Timebase equals 0.25 ms */
	AWU_Timebase_500us  = (uint8_t)2,    /*!< AWU Timebase equals 0.5 ms */
	AWU_Timebase_1ms    = (uint8_t)3,    /*!< AWU Timebase equals 1 ms */
	AWU_Timebase_2ms    = (uint8_t)4,    /*!< AWU Timebase equals 2 ms */
	AWU_Timebase_4ms    = (uint8_t)5,    /*!< AWU Timebase equals 4 ms */
	AWU_Timebase_8ms    = (uint8_t)6,    /*!< AWU Timebase equals 8 ms */
	AWU_Timebase_16ms   = (uint8_t)7,    /*!< AWU Timebase equals 16 ms */
	AWU_Timebase_32ms   = (uint8_t)8,    /*!< AWU Timebase equals 32 ms */
	AWU_Timebase_64ms   = (uint8_t)9,    /*!< AWU Timebase equals 64 ms */
	AWU_Timebase_128ms  = (uint8_t)10,   /*!< AWU Timebase equals 128 ms */
	AWU_Timebase_256ms  = (uint8_t)11,   /*!< AWU Timebase equals 256 ms */
	AWU_Timebase_512ms  = (uint8_t)12,   /*!< AWU Timebase equals 512 ms */
	AWU_Timebase_1s     = (uint8_t)13,   /*!< AWU Timebase equals 1 s */
	AWU_Timebase_2s     = (uint8_t)14,   /*!< AWU Timebase equals 2 s */
	AWU_Timebase_12s    = (uint8_t)15,   /*!< AWU Timebase equals 12 s */
	AWU_Timebase_30s    = (uint8_t)16    /*!< AWU Timebase equals 30 s */
};

static inline void awu_enable(void)
{
	AWU->CSR |= AWU_CSR_AWUEN;
}

static inline void awu_disable(void)
{
	AWU->CSR &= ~AWU_CSR_AWUEN;
}

void awu_init(enum awu_timebase_t timebase, void (*callback)(void));

#endif /* STM8S_AWU_H */