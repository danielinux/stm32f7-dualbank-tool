/*  
 *      This is free software: you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License version 2, as 
 *      published by the Free Software Foundation.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *      Author: Daniele Lacamera
 *      
 *
 */  
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "system.h"


#ifdef SPI_FLASH
extern void spi_release(void);
#else
#define spi_release() do{}while(0)
#endif



#define CLOCK_SPEED (216000000)

#define APB2_CLOCK_ER           (*(volatile uint32_t *)(0x40023844))

#define AHB1_CLOCK_ER (*(volatile uint32_t *)(0x40023830))
#define GPIOA_AHB1_CLOCK_ER (1 << 0)
#define GPIOD_AHB1_CLOCK_ER (1 << 3)
#define GPIOA_BASE 0x40020000
#define GPIOD_BASE 0x40020c00

#define GPIOA_MODE  (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_AFL   (*(volatile uint32_t *)(GPIOA_BASE + 0x20))
#define GPIOA_AFH   (*(volatile uint32_t *)(GPIOA_BASE + 0x24))
#define GPIOA_BSRR  (*(volatile uint32_t *)(GPIOA_BASE + 0x18))
#define GPIOA_PUPD  (*(volatile uint32_t *)(GPIOA_BASE + 0x0c))

#define GPIOD_MODE  (*(volatile uint32_t *)(GPIOD_BASE + 0x00))
#define GPIOD_OTYPE (*(volatile uint32_t *)(GPIOD_BASE + 0x04))
#define GPIOD_OSPD  (*(volatile uint32_t *)(GPIOD_BASE + 0x08))
#define GPIOD_PUPD  (*(volatile uint32_t *)(GPIOD_BASE + 0x0c))
#define GPIOD_ODR   (*(volatile uint32_t *)(GPIOD_BASE + 0x14))
#define GPIOD_BSRR  (*(volatile uint32_t *)(GPIOD_BASE + 0x18))
#define GPIOD_AFL   (*(volatile uint32_t *)(GPIOD_BASE + 0x20))
#define GPIOD_AFH   (*(volatile uint32_t *)(GPIOD_BASE + 0x24))

#define FLASH_BASE          (0x40023C00)
#define FLASH_ACR           (*(volatile uint32_t *)(FLASH_BASE + 0x00))
#define FLASH_KEYR          (*(volatile uint32_t *)(FLASH_BASE + 0x04))
#define FLASH_OPTKEYR       (*(volatile uint32_t *)(FLASH_BASE + 0x08))
#define FLASH_SR            (*(volatile uint32_t *)(FLASH_BASE + 0x0C))
#define FLASH_CR            (*(volatile uint32_t *)(FLASH_BASE + 0x10))
#define FLASH_OPTCR         (*(volatile uint32_t *)(FLASH_BASE + 0x14))

/* Register values */
#define FLASH_ACR_ARTRST                     (1 << 11)
#define FLASH_ACR_PRFEN                       (1 << 9)
#define FLASH_ACR_ARTEN                       (1 << 8)

#define FLASH_SR_BSY                          (1 << 16)
#define FLASH_SR_PGSERR                       (1 << 7)
#define FLASH_SR_PGPERR                       (1 << 6)
#define FLASH_SR_PGAERR                       (1 << 5)
#define FLASH_SR_WRPERR                       (1 << 4)
#define FLASH_SR_OPERR                        (1 << 1)
#define FLASH_SR_EOP                          (1 << 0)

#define FLASH_CR_LOCK                         (1 << 31)
#define FLASH_CR_ERRIE                        (1 << 25)
#define FLASH_CR_EOPIE                        (1 << 24)
#define FLASH_CR_STRT                         (1 << 16)
#define FLASH_CR_MER2                         (1 << 15)
#define FLASH_CR_MER1                         (1 << 2)
#define FLASH_CR_SER                          (1 << 1)
#define FLASH_CR_PG                           (1 << 0)

#define FLASH_CR_SNB_SHIFT                      3
#define FLASH_CR_SNB_MASK                      0x1f

#define FLASH_CR_PROGRAM_X8                   (0 << 8)
#define FLASH_CR_PROGRAM_X16                  (1 << 8)
#define FLASH_CR_PROGRAM_X32                  (2 << 8)
#define FLASH_CR_PROGRAM_X64                  (3 << 8)

#define FLASH_OPTCR_nDBOOT                     (1 << 28)
#define FLASH_OPTCR_nDBANK                     (1 << 29)
#define FLASH_OPTCR_STRT                       (1 << 1)

#define FLASH_KEY1                            (0x45670123)
#define FLASH_KEY2                            (0xCDEF89AB)

#define FLASH_OPTKEY1                         (0x08192A3B)
#define FLASH_OPTKEY2                         (0x4C5D6E7F)


/* FLASH Geometry 
 *
 * per ST AN4826, two configurations are possible on STM32F7:
 *
 *  - Dual bank with swapping, 2 x 512KB banks, 8 sectors each
 *  - Single bank, 1 x 1MB, 8 sectors in total
 *
 *  The chosen configuration depends on the FLASH_OPTCR_DBANK value.
 *
 *
 * */

#define FLASH_SECTOR_UNUSED    0xFFFFFFFF
/*  This memory mapping is for 2MB flash (STM32F769). */

#   define SYSCFG_MEMRMP  (*(volatile uint32_t *)(0x40013800))
#   define MEMRMP_SWP_FB  (1 << 8)
#   define FLASH_SECTORS 28

/* Block 0: 0x8000000 */
#   define FLASH_SECTOR_0  0x8000000 /* 16 Kb   */
#   define FLASH_SECTOR_1  0x8004000 /* 16 Kb   */
#   define FLASH_SECTOR_2  0x8008000 /* 16 Kb   */
#   define FLASH_SECTOR_3  0x800C000 /* 16 Kb   */
#   define FLASH_SECTOR_4  0x8010000 /* 64 Kb   */
#   define FLASH_SECTOR_5  0x8020000 /* 128 Kb  */
#   define FLASH_SECTOR_6  0x8040000 /* 128 Kb  */
#   define FLASH_SECTOR_7  0x8060000 /* 128 Kb  */
#   define FLASH_SECTOR_8  0x8080000 /* 128 Kb  */
#   define FLASH_SECTOR_9  0x80A0000 /* 128 Kb  */
#   define FLASH_SECTOR_10 0x80C0000 /* 128 Kb  */
#   define FLASH_SECTOR_11 0x80E0000 /* 128 Kb  */

/* Block 1: 0x8100000 */
#   define FLASH_SECTOR_16 0x8100000 /* 16 Kb  */
#   define FLASH_SECTOR_17 0x8104000 /* 16 Kb  */
#   define FLASH_SECTOR_18 0x8108000 /* 16 Kb  */
#   define FLASH_SECTOR_19 0x810C000 /* 16 Kb  */
#   define FLASH_SECTOR_20 0x8110000 /* 64 Kb  */
#   define FLASH_SECTOR_21 0x8120000 /* 128 Kb  */
#   define FLASH_SECTOR_22 0x8140000 /* 128 Kb  */
#   define FLASH_SECTOR_23 0x8160000 /* 128 Kb  */
#   define FLASH_SECTOR_24 0x8180000 /* 128 Kb  */
#   define FLASH_SECTOR_25 0x81A0000 /* 128 Kb  */
#   define FLASH_SECTOR_26 0x81C0000 /* 128 Kb  */
#   define FLASH_SECTOR_27 0x81E0000 /* 128 Kb  */


#   define FLASH_TOP       0x8200000

const uint32_t flash_sector[FLASH_SECTORS + 1] = {
    FLASH_SECTOR_0,
    FLASH_SECTOR_1,
    FLASH_SECTOR_2,
    FLASH_SECTOR_3,
    FLASH_SECTOR_4,
    FLASH_SECTOR_5,
    FLASH_SECTOR_6,
    FLASH_SECTOR_7,
    FLASH_SECTOR_8,
    FLASH_SECTOR_9,
    FLASH_SECTOR_10,
    FLASH_SECTOR_11,
    FLASH_SECTOR_UNUSED,
    FLASH_SECTOR_UNUSED,
    FLASH_SECTOR_UNUSED,
    FLASH_SECTOR_UNUSED,
    FLASH_SECTOR_16,
    FLASH_SECTOR_17,
    FLASH_SECTOR_18,
    FLASH_SECTOR_19,
    FLASH_SECTOR_20,
    FLASH_SECTOR_21,
    FLASH_SECTOR_22,
    FLASH_SECTOR_23,
    FLASH_SECTOR_24,
    FLASH_SECTOR_25,
    FLASH_SECTOR_26,
    FLASH_SECTOR_27,
    FLASH_TOP
};


static void flash_wait_complete(void)
{
    while ((FLASH_SR & FLASH_SR_BSY) == FLASH_SR_BSY)
        ;
}

static void flash_erase_sector(uint32_t sec)
{
    uint32_t reg = FLASH_CR & (~(FLASH_CR_SNB_MASK << FLASH_CR_SNB_SHIFT));
    FLASH_CR = reg | (sec & FLASH_CR_SNB_MASK) << FLASH_CR_SNB_SHIFT;
    FLASH_CR |= FLASH_CR_SER;
    FLASH_CR |= FLASH_CR_STRT;
    flash_wait_complete();
    FLASH_CR &= ~FLASH_CR_SER;
    FLASH_CR &= ~(FLASH_CR_SNB_MASK << FLASH_CR_SNB_SHIFT);
}

static void clear_errors(void)
{
    FLASH_SR |= ( FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_OPERR | FLASH_SR_EOP );
}

int stm32f7_flash_write(uint32_t address, const uint8_t *data, int len)
{
    int i;
    uint32_t val;
    flash_wait_complete();
    clear_errors();
    /* Set 8-bit write */
    FLASH_CR &= (~(0x03 << 8));
    for (i = 0; i < len; i++) {
        FLASH_CR |= FLASH_CR_PG;
        *((uint8_t *)(address + i)) = data[i];
        flash_wait_complete();
        FLASH_CR &= ~FLASH_CR_PG;
    }
    return 0;
}

void stm32f7_flash_unlock(void)
{
    FLASH_CR |= FLASH_CR_LOCK;
    FLASH_KEYR = FLASH_KEY1;
    FLASH_KEYR = FLASH_KEY2;
}

void stm32f7_flash_lock(void)
{
    FLASH_CR |= FLASH_CR_LOCK;
}


int stm32f7_flash_erase(uint32_t address, int len)
{
    int start = -1, end = -1;
    uint32_t end_address;
    int i;
    int nxt;
    if (len == 0)
        return -1;
    end_address = address + len - 1;

    if (address < flash_sector[0] || end_address > FLASH_TOP)
        return -1;
    for (i = 0; i < FLASH_SECTORS; i++)
    {
        if (flash_sector[i] == FLASH_SECTOR_UNUSED)
            continue;
        nxt = i + 1;
        while (nxt < FLASH_SECTORS) {
            if (flash_sector[nxt] == FLASH_SECTOR_UNUSED)
                nxt++;
            else
                break;
        }
        if ((address >= flash_sector[i]) && (address < flash_sector[nxt])) {
            start = i;
        }
        if ((end_address >= flash_sector[i]) && (end_address < flash_sector[nxt])) {
            end = i;
        }
        if (start > 0 && end > 0)
            break;
    }
    if (start < 0 || end < 0)
        return -1;
    for (i = start; i <= end; i++)
        flash_erase_sector(i);
    return 0;
}

void clock_pll_off(void)
{
    uint32_t reg32;
    /* Enable internal high-speed oscillator. */
    RCC_CR |= RCC_CR_HSION;
    DMB();
    while ((RCC_CR & RCC_CR_HSIRDY) == 0) {};

    /* Select HSI as SYSCLK source. */
    reg32 = RCC_CFGR;
    reg32 &= ~((1 << 1) | (1 << 0));
    RCC_CFGR = (reg32 | RCC_CFGR_SW_HSI);
    DMB();

    /* Turn off PLL */
    RCC_CR &= ~RCC_CR_PLLON;
    DMB();
}

static void flash_set_waitstates(int waitstates)
{
    FLASH_ACR |=  waitstates | FLASH_ACR_PRFEN | FLASH_ACR_ARTEN;
}

void clock_pll_on(void)
{
    uint32_t reg32;
    uint32_t cpu_freq, plln, pllm, pllq, pllp,  hpre, ppre1, ppre2, flash_waitstates;

    /* Enable Power controller */
    APB1_CLOCK_ER |= PWR_APB1_CLOCK_ER_VAL;

    /* Select clock parameters (CPU Speed = 216MHz) */
    pllm = 25;
    plln = 432;
    pllp = 2;
    pllq = 9;
    hpre = RCC_PRESCALER_DIV_NONE;
    ppre1 = RCC_PRESCALER_DIV_4;
    ppre2 = RCC_PRESCALER_DIV_2;
    flash_waitstates = 7;
    cpu_freq = 216000000;

    //flash_set_waitstates(flash_waitstates);

    /* Enable internal high-speed oscillator. */
    RCC_CR |= RCC_CR_HSION;
    DMB();
    while ((RCC_CR & RCC_CR_HSIRDY) == 0) {};

    /* Select HSI as SYSCLK source. */
    reg32 = RCC_CFGR;
    ISB();
    reg32 &= ~((1 << 1) | (1 << 0));
    DMB();
    RCC_CFGR = (reg32 | RCC_CFGR_SW_HSI);
    DMB();

    /* Enable external high-speed oscillator 8MHz. */
    RCC_CR |= RCC_CR_HSEON;
    DMB();
    while ((RCC_CR & RCC_CR_HSERDY) == 0) {};

    /*
     * Set prescalers for AHB, ADC, ABP1, ABP2.
     */
    reg32 = RCC_CFGR;
    reg32 &= ~(0xF0);
    RCC_CFGR = (reg32 | (hpre << 4));
    DMB();
    reg32 = RCC_CFGR;
    reg32 &= ~(0x1C00);
    RCC_CFGR = (reg32 | (ppre1 << 10));
    DMB();
    reg32 = RCC_CFGR;
    reg32 &= ~(0x07 << 13);
    RCC_CFGR = (reg32 | (ppre2 << 13));
    DMB();

    /* Set PLL config */
    reg32 = RCC_PLLCFGR;
    reg32 &= ~(PLL_FULL_MASK);
    RCC_PLLCFGR = reg32 | RCC_PLLCFGR_PLLSRC | pllm |
        (plln << 6) | (((pllp >> 1) - 1) << 16) |
        (pllq << 24);
    DMB();
    /* Enable PLL oscillator and wait for it to stabilize. */
    RCC_CR |= RCC_CR_PLLON;
    DMB();
    while ((RCC_CR & RCC_CR_PLLRDY) == 0) {};

    /* Select PLL as SYSCLK source. */
    reg32 = RCC_CFGR;
    reg32 &= ~((1 << 1) | (1 << 0));
    RCC_CFGR = (reg32 | RCC_CFGR_SW_PLL);
    DMB();

    /* Wait for PLL clock to be selected. */
    while ((RCC_CFGR & ((1 << 1) | (1 << 0))) != RCC_CFGR_SW_PLL) {};

    /* Disable internal high-speed oscillator. */
    RCC_CR &= ~RCC_CR_HSION;
}

void sys_single_bank(void)
{
    /* Unlock OPTCR */
    FLASH_OPTKEYR = FLASH_OPTKEY1;
    FLASH_OPTKEYR = FLASH_OPTKEY2;

    /* Switch to single bank */
    FLASH_OPTCR |= (FLASH_OPTCR_nDBANK);
    FLASH_OPTCR |= FLASH_OPTCR_STRT;
}

void sys_dual_bank(void)
{
    uint32_t reg;

    /* Unlock OPTCR */
    FLASH_OPTKEYR = FLASH_OPTKEY1;
    FLASH_OPTKEYR = FLASH_OPTKEY2;

    /* Disable protection + Switch to dual bank */
    reg = FLASH_OPTCR;
    reg &= ~0x000FF00;
    reg |= 0x0000AA00;
    reg &= ~(FLASH_OPTCR_nDBANK);
    FLASH_OPTCR = reg;
    FLASH_OPTCR |= FLASH_OPTCR_STRT;
}

#define LED_BOOT_PIN (4)
#define LED_USR_PIN (12)
void boot_led_on(void)
{
    uint32_t reg;
    uint32_t pin = LED_BOOT_PIN;
    AHB1_CLOCK_ER |= GPIOD_AHB1_CLOCK_ER;
    reg = GPIOD_MODE & ~(0x03 << (pin * 2));
    GPIOD_MODE = reg | (1 << (pin * 2));
    reg = GPIOD_PUPD & ~(0x03 << (pin * 2));
    GPIOD_PUPD = reg | (1 << (pin * 2));
    GPIOD_BSRR |= (1 << pin);
}

void boot_led_off(void)
{
    GPIOD_BSRR |= (1 << (LED_BOOT_PIN + 16));
}

void usr_led_on(void)
{
    uint32_t reg;
    uint32_t pin = LED_USR_PIN;
    AHB1_CLOCK_ER |= GPIOA_AHB1_CLOCK_ER;
    reg = GPIOA_MODE & ~(0x03 << (pin * 2));
    GPIOA_MODE = reg | (1 << (pin * 2));
    reg = GPIOA_PUPD & ~(0x03 << (pin * 2));
    GPIOA_PUPD = reg | (1 << (pin * 2));
    GPIOA_BSRR |= (1 << pin);
}

void usr_led_off(void)
{
    GPIOA_BSRR |= (1 << (LED_USR_PIN + 16));
}

void *memset(void *s, int c, size_t n)
{
	unsigned char *d = (unsigned char *)s;

	while (n--) {
		*d++ = (unsigned char)c;
	}

	return s;
}

void *memcpy(void *dst, const void *src, size_t n)
{
    size_t i;
    const char *s = (const char *)src;
    char *d = (char *)dst;

    for (i = 0; i < n; i++) {
        d[i] = s[i];
    }

    return dst;
}

void stm32f7_flash_mass_erase_dual_block(void)
{
    FLASH_CR |= FLASH_CR_MER1 | FLASH_CR_MER2;
    FLASH_CR |= FLASH_CR_STRT;
    flash_wait_complete();
    FLASH_CR &= ~(FLASH_CR_MER1 | FLASH_CR_MER2);
}

void stm32f7_flash_mass_erase_single_block(void)
{
    FLASH_CR |= FLASH_CR_MER1;
    FLASH_CR |= FLASH_CR_STRT;
    flash_wait_complete();
    FLASH_CR &= ~(FLASH_CR_MER1);
}
