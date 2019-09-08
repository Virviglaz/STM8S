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

#include "stm8s_mem.h"

#define EEPROM_START_ADDRESS	0x4000
#define EEPROM_END_ADDRESS   	0x427F
#define EEPROM_SIZE		(EEPROM_END_ADDRESS - EEPROM_START_ADDRESS)
#define FLASH_BLOCK_SIZE	(uint8_t)64

#define FLASH_RASS_KEY1 ((uint8_t)0x56) /*!< First RASS key */
#define FLASH_RASS_KEY2 ((uint8_t)0xAE) /*!< Second RASS key */

#define OPTION_BYTE_START_PHYSICAL_ADDRESS  ((uint16_t)0x4800)
#define OPTION_BYTE_END_PHYSICAL_ADDRESS    ((uint16_t)0x487F)
#define FLASH_OPTIONBYTE_ERROR              ((uint16_t)0x5555)

static void mem_unlock(enum mem memory)
{
	if (memory == FLASH_MEMTYPE_PROG) {
		FLASH->PUKR = FLASH_RASS_KEY1;
    		FLASH->PUKR = FLASH_RASS_KEY2;
	} else {
		FLASH->DUKR = FLASH_RASS_KEY2;
		FLASH->DUKR = FLASH_RASS_KEY1;
	}
}

static void mem_lock(enum mem memory)
{
	FLASH->IAPSR &= (uint8_t)memory;
}

/**
  * @brief  Write buffer to EEPROM.
  * @param  offset: Offset from beginning of EEPROM
  * @param  buf: Pointer to beginning of buffer
  * @param  size: Number of bytes to write
  * @retval uint16_t: number of bytes that was not written
  */
uint16_t eeprom_write(uint16_t offset, uint8_t *buf, uint16_t size)
{
	uint8_t *p = (uint8_t *)(EEPROM_START_ADDRESS + offset);

	mem_unlock(FLASH_MEMTYPE_DATA);

	while ((uint16_t)p < EEPROM_END_ADDRESS && size) {
		*p++ = *buf++;
		size--;
	};

	mem_lock(FLASH_MEMTYPE_DATA);

	return size;
}

/**
  * @brief  Read data from EEPROM to buffer.
  * @param  offset: Offset from beginning of EEPROM
  * @param  buf: Pointer to beginning of buffer
  * @param  size: Number of bytes to read
  * @retval uint16_t: number of bytes that was not read
  */
uint16_t eeprom_read(uint16_t offset, uint8_t *buf, uint16_t size)
{
	uint8_t *p = (uint8_t *)(EEPROM_START_ADDRESS + offset);

	while ((uint16_t)p < EEPROM_END_ADDRESS && size) {
		*buf++ = *p++;
		size--;
	};

	return size;
}