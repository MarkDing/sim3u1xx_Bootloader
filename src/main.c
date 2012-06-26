//------------------------------------------------------------------------------
// Copyright (c) 2011 by Silicon Laboratories Inc.  All rights reserved.
// The program contained in this listing is proprietary to Silicon Laboratories,
// headquartered in Austin, Texas, U.S.A. and is subject to worldwide copyright
// protection, including protection under the United States Copyright Act of 1976
// as an unpublished work, pursuant to Section 104 and Section 408 of Title XVII
// of the United States code.  Unauthorized copying, adaptation, distribution,
// use, or display is prohibited by this law.
//
// Silicon Laboratories provides this software solely and exclusively
// for use on Silicon Laboratories' microcontroller products.
//
// This software is provided "as is".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
// SILICON LABORATORIES SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
// INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
//------------------------------------------------------------------------------
// library
#include <stdio.h>
// hal
#include <si32_device.h>
#include <SI32_CLKCTRL_A_Type.h>
#include <SI32_WDTIMER_A_Type.h>
#include <SI32_PBCFG_A_Type.h>
#include <SI32_PBSTD_A_Type.h>
#include "config.h"
#include "uart.h"
#include "flash.h"
volatile uint32_t msTicks;
#define RECV_BUFFER_SIZE 512
unsigned char data_buf[RECV_BUFFER_SIZE];

/* other code*/
//==============================================================================
//1st LEVEL  INTERRUPT HANDLERS
//==============================================================================
void SysTick_Handler(void)
{
   msTicks++;
   /*NO SENCOND LEVEL HANDERL SPESIFIED*/
}

void mySystemInit(void)
{
   SI32_WDTIMER_A_stop_counter (SI32_WDTIMER_0);
   // Enable the APB clock to the PB registers
   SI32_CLKCTRL_A_enable_apb_to_modules_0(SI32_CLKCTRL_0, SI32_CLKCTRL_A_APBCLKG0_PB0);
   SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_1, 0x00000008);

   SI32_PBCFG_A_enable_crossbar_1(SI32_PBCFG_0);
   SI32_PBSTD_A_set_pins_digital_input(SI32_PBSTD_2,0x00000300);
   // Enable the LED drivers (P2.10, P2.11)
   SI32_PBSTD_A_set_pins_push_pull_output(SI32_PBSTD_2, 0x00000C00);
   SysTick_Config(SystemCoreClock / 1000);
   // set Priority for Cortex-M0 System Interrupts.
   NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

}

// Returns a non-zero value if an update is needed
unsigned int check_update_requirement(void)
{
   // PB2.8 high require update, low -- run application.
   if (SI32_PBSTD_A_read_pin(SI32_PBSTD_2,8))
   {
      return 1;
   }
   else
   {
      return 0;
   }
}

#define FLASH_SECTOR_SIZE 0x400

#define CMD_WRITE_FLASH		0x31
#define CMD_READ_FLASH     0x11
#define CMD_ERASE_FLASH 	0x43
#define CMD_UNKNOW         0xFF

#define RES_ACK            0x79
#define RES_NACK           0x1F
#define RES_UNKNOW         0xEE

void bl_send_ack(unsigned char ack)
{
   unsigned char res = ack;
   uart_send_data(&res, 1);
}

int bl_get_cmd(void)
{
   unsigned char *ptr = data_buf;
   if (uart_get_data(ptr, 2) == 0)
   {
      if (ptr[0] != (ptr[1] ^ 0xFF))
      {
         bl_send_ack(RES_NACK);
         return -2;
      }
      else
      {
         bl_send_ack(RES_ACK);
         return 0;
      }
   }
   return -1;
}

void bl_write_flash()
{
   int i;
   unsigned int addr, len;
   unsigned char crc;
   unsigned char *ptr = data_buf;
   // get address
   if (uart_get_data(ptr, 5) == 0)
   {
      crc = ptr[0] ^ ptr[1] ^ ptr[2] ^ ptr[3];
      if (crc == ptr[4])
      {
         addr = ptr[3] | (ptr[2] << 8) | (ptr[1] << 16) | (ptr[0] << 24);
      }
   }
   // send ack
   bl_send_ack(RES_ACK);
   // get data len
   uart_get_data(ptr, 1);
   len = ptr[0] + 1;
   // get data and crc
   if (uart_get_data(ptr, len + 1) == 0)
   {
      crc = 0xFF;
      for (i = 0; i < len; i++)
      {
         crc = crc ^ ptr[i];
      }
   }
   //write flash
   if (crc == ptr[len])
   {
      if ((addr >= USER_CODE_ADDRESS) && addr < TOP_CODE_ADDRESS)
      {
         flash_write_data(ptr, addr, len);
      }
   }
   // send ack
   bl_send_ack(RES_ACK);
}

void bl_read_flash()
{
   int i;
   unsigned int addr, len = 0;
   unsigned char crc;
   unsigned char *ptr = data_buf;
   // get address
   if (uart_get_data(ptr, 5) == 0)
   {
      crc = ptr[0] ^ ptr[1] ^ ptr[2] ^ ptr[3];
      if (crc == ptr[4])
      {
         addr = ptr[3] | (ptr[2] << 8) | (ptr[1] << 16) | (ptr[0] << 24);
      }
   }
   // send ack
   bl_send_ack(RES_ACK);
   // get data len and crc
   if(uart_get_data(ptr, 2) == 0)
   {
      if((ptr[0] ^ 0xFF) == ptr[1])
      {
         len = ptr[0] + 1;
      }
   }
   // send ack
   bl_send_ack(RES_ACK);
   // send data
   for(i =0; i < len; i++)
   {
      ptr[i]= *(unsigned char *)(addr++);
   }
   uart_send_data(ptr,len);
}

void bl_erase_flash()
{
   int i;
   unsigned int addr, len;
   unsigned char crc;
   unsigned char *ptr = data_buf;

   // get len
   if (uart_get_data(ptr, 1) == 0)
   {
      len = (ptr[0] + 1) & 0xFF; // maximum sector number is 256
   }
   // get sector sequence and crc
   if (uart_get_data(ptr, len + 1) == 0)
   {
      crc = 0xFF;
      for (i = 0; i < len; i++)
      {
         crc = crc ^ ptr[i];
      }
   }
   // erase flash sectors
   if (crc == ptr[len])
   {
      for (i = 0; i < len; i++)
      {
         addr = ptr[i] * FLASH_SECTOR_SIZE;
         if ((addr >= USER_CODE_ADDRESS) && addr < TOP_CODE_ADDRESS)
         {
            flash_erase_page(addr);
         }
      }
   }
   // send ack
   bl_send_ack(RES_ACK);
}
//==============================================================================
// myApplication.
//==============================================================================
int main()
{
   UART0_initialize();
   flash_initialize();
   while (1)
   {
      SI32_PBSTD_A_toggle_pins(SI32_PBSTD_2, 0xC00);
      // received CMD from host
      if (bl_get_cmd() == 0)
      {
         switch (data_buf[0])
         {
            case CMD_WRITE_FLASH:
               bl_write_flash();
               break;
            case CMD_READ_FLASH:
               bl_read_flash();
               break;
            case CMD_ERASE_FLASH:
               bl_erase_flash();
               break;
            default:
               break;
         }
      }
   }// while(1)

}
//---eof------------------------------------------------------------------------
