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
#define MSGSIZE 16
#define PAGESIZE (256)
static uint8_t page[PAGESIZE];
static const char ERR='!';
static const char START='*';
static const char UPDATE='U';
static const char ACK='#';
static uint8_t msg[MSGSIZE];

void main(void)
{
    clock_pll_on();
    sys_single_bank();
//    stm32f7_flash_mass_erase_single_block();
    boot_led_on();
    usr_led_on();
    boot_led_off();
    while(1)
        ;
}
